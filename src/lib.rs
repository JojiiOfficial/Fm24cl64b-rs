#![cfg_attr(not(test), no_std)]
//! Minimal sync `embedded-hal` 1.0 I²C driver for the Cypress FM24CL64B F-RAM.

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// Memory size in bytes (8 K × 8).
pub const MEMORY_SIZE: u16 = 8192;

/// 7-bit slave-ID base (`0b1010_xxx`); the 3 LSBs come from the A2/A1/A0 pins.
const SLAVE_ID_BASE: u8 = 0x50;

/// Max data bytes shipped per I²C write call. The chip itself has no page
/// boundary (F-RAM); this only bounds the internal buffer size.
const CHUNK: usize = 32;

/// Value applied to the A2/A1/A0 hardware pins.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum AddrPin {
    A000 = 0,
    A001 = 1,
    A010 = 2,
    A011 = 3,
    A100 = 4,
    A101 = 5,
    A110 = 6,
    A111 = 7,
}

#[derive(Debug)]
pub enum Error<E> {
    /// Underlying I²C bus error.
    I2c(E),
    /// `address + len` would exceed `MEMORY_SIZE`.
    OutOfRange,
    /// `self_test` readback did not match the written pattern.
    SelfTestFailed { expected: u8, actual: u8 },
}

pub struct Fm24cl64b<I2C> {
    i2c: I2C,
    addr: SevenBitAddress,
    cmd_buf: [u8; CHUNK + 2],
}

impl<I2C, E> Fm24cl64b<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C, addr_pin: AddrPin) -> Self {
        Self {
            i2c,
            addr: SLAVE_ID_BASE | (addr_pin as u8),
            cmd_buf: [0u8; CHUNK + 2],
        }
    }

    /// Release the underlying I²C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Read `buf.len()` bytes starting at `address`.
    pub fn read(&mut self, address: u16, buf: &mut [u8]) -> Result<(), Error<E>> {
        if address as usize + buf.len() > MEMORY_SIZE as usize {
            return Err(Error::OutOfRange);
        }
        let addr_bytes = [(address >> 8) as u8, (address & 0xFF) as u8];
        self.i2c
            .write_read(self.addr, &addr_bytes, buf)
            .map_err(Error::I2c)
    }

    /// Write `data` starting at `address`. Auto-chunks for buffering only;
    /// F-RAM has no page-write boundary so chunk size has no chip-level effect.
    pub fn write(&mut self, mut address: u16, data: &[u8]) -> Result<(), Error<E>> {
        if address as usize + data.len() > MEMORY_SIZE as usize {
            return Err(Error::OutOfRange);
        }

        let mut offset = 0;
        while offset < data.len() {
            let n = (data.len() - offset).min(CHUNK);
            self.cmd_buf[0] = (address >> 8) as u8;
            self.cmd_buf[1] = (address & 0xFF) as u8;
            self.cmd_buf[2..2 + n].copy_from_slice(&data[offset..offset + n]);

            self.i2c
                .write(self.addr, &self.cmd_buf[..2 + n])
                .map_err(Error::I2c)?;

            address += n as u16;
            offset += n;
        }
        Ok(())
    }

    /// Cheap presence check: reads (and discards) one byte from address `0`.
    /// Returns `Ok(())` if the chip ACKs and the Selective Read completes.
    /// Non-destructive.
    pub fn probe(&mut self) -> Result<(), Error<E>> {
        let mut buf = [0u8; 1];
        self.read(0, &mut buf)
    }

    /// Non-destructive read/write/compare integrity check at `address`.
    ///
    /// The FM24CL64B exposes no chip-ID command, so this is the closest
    /// equivalent: it saves the byte at `address`, writes two complementary
    /// patterns (`0xAA` then `0x55`), verifies each readback, and restores
    /// the original. Exercises the full read+write path and detects WP being
    /// asserted, stuck bits, and broken bus wiring.
    ///
    /// On a readback mismatch, returns `Error::SelfTestFailed` after a
    /// best-effort restore. On a bus error mid-flight, the byte at `address`
    /// may be left holding a test pattern.
    pub fn self_test(&mut self, address: u16) -> Result<(), Error<E>> {
        let mut original = [0u8; 1];
        self.read(address, &mut original)?;

        for &pattern in &[0xAAu8, 0x55u8] {
            self.write(address, &[pattern])?;
            let mut readback = [0u8; 1];
            self.read(address, &mut readback)?;
            if readback[0] != pattern {
                let _ = self.write(address, &original);
                return Err(Error::SelfTestFailed {
                    expected: pattern,
                    actual: readback[0],
                });
            }
        }

        self.write(address, &original)
    }
}
