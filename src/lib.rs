#![cfg_attr(not(test), no_std)]
//! Sync `embedded-hal` 1.0 I²C driver for the Cypress FM24CL64B F-RAM.

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_storage::{ReadStorage, Storage};

/// Memory size in bytes (8 K × 8).
pub const MEMORY_SIZE: u16 = 8192;
/// Exclusive upper bound on valid addresses; equal to `MEMORY_SIZE`.
pub const MEMORY_END: u16 = MEMORY_SIZE;
/// `0..MEMORY_SIZE`, e.g. for `MEMORY_RANGE.contains(&addr)`.
pub const MEMORY_RANGE: core::ops::Range<u16> = 0..MEMORY_SIZE;

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

macro_rules! driver_typed_accessors {
    ($($ty:ty, $size:literal, $read_le:ident, $write_le:ident);+ $(;)?) => {
        $(
            pub fn $read_le(&mut self, address: u16) -> Result<$ty, Error<E>> {
                let mut b = [0u8; $size];
                self.read(address, &mut b)?;
                Ok(<$ty>::from_le_bytes(b))
            }

            pub fn $write_le(&mut self, address: u16, value: $ty) -> Result<(), Error<E>> {
                self.write(address, &value.to_le_bytes())
            }
        )+
    };
}

macro_rules! cursor_typed_accessors {
    ($($ty:ty, $size:literal, $read_le:ident, $write_le:ident);+ $(;)?) => {
        $(
            pub fn $read_le(&mut self) -> Result<$ty, Error<E>> {
                let v = self.driver.$read_le(self.pos)?;
                self.pos += $size;
                Ok(v)
            }

            pub fn $write_le(&mut self, value: $ty) -> Result<(), Error<E>> {
                self.driver.$write_le(self.pos, value)?;
                self.pos += $size;
                Ok(())
            }
        )+
    };
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

    /// Read `buf.len()` bytes starting at `address` (Selective Read, datasheet
    /// Figure 11).
    pub fn read(&mut self, address: u16, buf: &mut [u8]) -> Result<(), Error<E>> {
        if address as usize + buf.len() > MEMORY_SIZE as usize {
            return Err(Error::OutOfRange);
        }
        let addr_bytes = [(address >> 8) as u8, (address & 0xFF) as u8];
        self.i2c
            .write_read(self.addr, &addr_bytes, buf)
            .map_err(Error::I2c)
    }

    /// Read `buf.len()` bytes starting at the chip's current internal address
    /// (Current Address / Sequential Read, datasheet Figures 9 and 10).
    /// Skips the address-write phase `read` performs; prefer it for streaming /
    /// sequential reads on a single-master bus where the latch position is known.
    /// The latch advances after every byte and wraps `0x1FFF → 0x0000`.
    pub fn read_current(&mut self, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c.read(self.addr, buf).map_err(Error::I2c)
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

    /// Clears/Zeros the entire memory.
    pub fn clear(&mut self) -> Result<(), Error<E>> {
        const CLEAR_CHUNK: usize = 128;
        assert_eq!(MEMORY_SIZE % CLEAR_CHUNK as u16, 0);

        let chunk = [0u8; CLEAR_CHUNK];
        let mut scratch = [0u8; CLEAR_CHUNK * 10];
        for addr_idx in 0..(MEMORY_SIZE / chunk.len() as u16) {
            let addr = addr_idx * chunk.len() as u16;
            self.write_with_scratch(addr, &chunk, &mut scratch)?;
        }
        Ok(())
    }

    /// Like [`Self::write`] but uses a caller-provided `scratch` buffer for
    /// the I²C transaction payload. Each transaction ships up to
    /// `scratch.len() - 2` data bytes (the first two hold the destination
    /// address). Larger `scratch` → fewer transactions → higher throughput.
    /// F-RAM has no page-write boundary so chunk size has no chip-level effect.
    ///
    /// Errors with `OutOfRange` if `scratch.len() < 3`.
    pub fn write_with_scratch(
        &mut self,
        mut address: u16,
        data: &[u8],
        scratch: &mut [u8],
    ) -> Result<(), Error<E>> {
        if scratch.len() < 3 {
            return Err(Error::OutOfRange);
        }
        if address as usize + data.len() > MEMORY_SIZE as usize {
            return Err(Error::OutOfRange);
        }
        let chunk = scratch.len() - 2;
        let mut offset = 0;
        while offset < data.len() {
            let n = (data.len() - offset).min(chunk);
            scratch[0] = (address >> 8) as u8;
            scratch[1] = (address & 0xFF) as u8;
            scratch[2..2 + n].copy_from_slice(&data[offset..offset + n]);
            self.i2c
                .write(self.addr, &scratch[..2 + n])
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

    // === Single-byte ergonomics ===

    pub fn read_byte(&mut self, address: u16) -> Result<u8, Error<E>> {
        let mut b = [0u8; 1];
        self.read(address, &mut b)?;
        Ok(b[0])
    }

    pub fn write_byte(&mut self, address: u16, value: u8) -> Result<(), Error<E>> {
        self.write(address, &[value])
    }

    pub fn read_u8(&mut self, address: u16) -> Result<u8, Error<E>> {
        self.read_byte(address)
    }

    pub fn read_i8(&mut self, address: u16) -> Result<i8, Error<E>> {
        self.read_byte(address).map(|b| b as i8)
    }

    pub fn write_u8(&mut self, address: u16, value: u8) -> Result<(), Error<E>> {
        self.write_byte(address, value)
    }

    pub fn write_i8(&mut self, address: u16, value: i8) -> Result<(), Error<E>> {
        self.write_byte(address, value as u8)
    }

    // === Fixed-size array read ===

    pub fn read_array<const N: usize>(&mut self, address: u16) -> Result<[u8; N], Error<E>> {
        let mut b = [0u8; N];
        self.read(address, &mut b)?;
        Ok(b)
    }

    // === Typed primitive accessors (le / be) ===

    driver_typed_accessors! {
        u16, 2, read_u16_le, write_u16_le;
        u32, 4, read_u32_le, write_u32_le;
        u64, 8, read_u64_le, write_u64_le;
        i16, 2, read_i16_le, write_i16_le;
        i32, 4, read_i32_le, write_i32_le;
        i64, 8, read_i64_le, write_i64_le;
    }

    // === Range operations ===

    /// Write `value` `count` times starting at `address`.
    pub fn fill(&mut self, address: u16, count: u16, value: u8) -> Result<(), Error<E>> {
        if count == 0 {
            return Ok(());
        }
        if address as usize + count as usize > MEMORY_SIZE as usize {
            return Err(Error::OutOfRange);
        }

        self.cmd_buf[2..2 + CHUNK].fill(value);

        let mut addr = address;
        let mut remaining = count as usize;
        while remaining > 0 {
            let n = remaining.min(CHUNK);
            self.cmd_buf[0] = (addr >> 8) as u8;
            self.cmd_buf[1] = (addr & 0xFF) as u8;
            self.i2c
                .write(self.addr, &self.cmd_buf[..2 + n])
                .map_err(Error::I2c)?;
            addr += n as u16;
            remaining -= n;
        }
        Ok(())
    }

    /// Read and compare against `expected`. `Ok(true)` on full match, `Ok(false)` on mismatch.
    pub fn verify(&mut self, address: u16, expected: &[u8]) -> Result<bool, Error<E>> {
        if address as usize + expected.len() > MEMORY_SIZE as usize {
            return Err(Error::OutOfRange);
        }
        let mut buf = [0u8; CHUNK];
        let mut offset = 0;
        while offset < expected.len() {
            let n = (expected.len() - offset).min(CHUNK);
            self.read(address + offset as u16, &mut buf[..n])?;
            if buf[..n] != expected[offset..offset + n] {
                return Ok(false);
            }
            offset += n;
        }
        Ok(true)
    }

    /// Write then read back and compare. Catches the WP-active silent-NACK case.
    pub fn write_verified(&mut self, address: u16, data: &[u8]) -> Result<bool, Error<E>> {
        self.write(address, data)?;
        self.verify(address, data)
    }

    /// Read-modify-write of one byte. Returns the new value.
    /// Not atomic on a multi-master bus.
    pub fn modify_byte<F>(&mut self, address: u16, f: F) -> Result<u8, Error<E>>
    where
        F: FnOnce(u8) -> u8,
    {
        let new = f(self.read_byte(address)?);
        self.write_byte(address, new)?;
        Ok(new)
    }

    /// Borrow this driver as a [`Cursor`] starting at `address`.
    pub fn cursor(&mut self, address: u16) -> Cursor<'_, I2C> {
        Cursor {
            driver: self,
            pos: address,
        }
    }
}

pub struct Cursor<'a, I2C> {
    driver: &'a mut Fm24cl64b<I2C>,
    pos: u16,
}

impl<'a, I2C, E> Cursor<'a, I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(driver: &'a mut Fm24cl64b<I2C>, address: u16) -> Self {
        Self {
            driver,
            pos: address,
        }
    }

    pub fn position(&self) -> u16 {
        self.pos
    }

    pub fn remaining(&self) -> u16 {
        MEMORY_SIZE - self.pos
    }

    /// Set the position. No I/O and no bounds check; subsequent reads/writes
    /// validate against `MEMORY_SIZE` and surface `Error::OutOfRange` if past.
    pub fn set_position(&mut self, address: u16) {
        self.pos = address;
    }

    /// Move the position by `delta` (signed). Errors with `OutOfRange` on
    /// overflow past `MEMORY_SIZE` or underflow below `0`.
    pub fn advance(&mut self, delta: i32) -> Result<(), Error<E>> {
        let new = self.pos as i32 + delta;
        if new < 0 || new > MEMORY_SIZE as i32 {
            return Err(Error::OutOfRange);
        }
        self.set_position(new as u16);
        Ok(())
    }

    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.driver.read(self.pos, buf)?;
        self.pos += buf.len() as u16;
        Ok(())
    }

    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error<E>> {
        self.driver.write(self.pos, data)?;
        self.pos += data.len() as u16;
        Ok(())
    }

    pub fn read_array<const N: usize>(&mut self) -> Result<[u8; N], Error<E>> {
        let arr = self.driver.read_array::<N>(self.pos)?;
        self.pos += N as u16;
        Ok(arr)
    }

    pub fn read_byte(&mut self) -> Result<u8, Error<E>> {
        let v = self.driver.read_byte(self.pos)?;
        self.pos += 1;
        Ok(v)
    }

    pub fn write_byte(&mut self, value: u8) -> Result<(), Error<E>> {
        self.driver.write_byte(self.pos, value)?;
        self.pos += 1;
        Ok(())
    }

    pub fn read_u8(&mut self) -> Result<u8, Error<E>> {
        self.read_byte()
    }

    pub fn read_i8(&mut self) -> Result<i8, Error<E>> {
        self.read_byte().map(|b| b as i8)
    }

    pub fn write_u8(&mut self, value: u8) -> Result<(), Error<E>> {
        self.write_byte(value)
    }

    pub fn write_i8(&mut self, value: i8) -> Result<(), Error<E>> {
        self.write_byte(value as u8)
    }

    cursor_typed_accessors! {
        u16, 2, read_u16_le, write_u16_le;
        u32, 4, read_u32_le, write_u32_le;
        u64, 8, read_u64_le, write_u64_le;
        i16, 2, read_i16_le, write_i16_le;
        i32, 4, read_i32_le, write_i32_le;
        i64, 8, read_i64_le, write_i64_le;
    }
}

impl<I2C, E> ReadStorage for Fm24cl64b<I2C>
where
    I2C: I2c<Error = E>,
{
    type Error = Error<E>;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Fm24cl64b::read(self, offset as u16, bytes)
    }

    fn capacity(&self) -> usize {
        MEMORY_SIZE as usize
    }
}

impl<I2C, E> Storage for Fm24cl64b<I2C>
where
    I2C: I2c<Error = E>,
{
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        Fm24cl64b::write(self, offset as u16, bytes)
    }
}
