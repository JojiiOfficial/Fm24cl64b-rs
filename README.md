# Fm24cl64b

`embedded-hal` 1.0 (sync) I²C driver for the Cypress FM24CLxx family of F-RAM chips.
Also implements `embedded-storage` traits.

## Supported chips

| Chip       | Memory   | Supported |
|------------|----------|-----------|
| FM24CL04B  | 512 B    | ❌        |
| FM24CL16B  | 2048 B   | ❌        |
| FM24CL64B  | 8192 B   | ✅        |

## Examples

Setup, assumed for every snippet below:

```rust
use fm24clxx::{Fm24cl64b, AddrPin};

// `i2c` is your platform's embedded-hal 1.0 I²C bus handle.
let mut fram = Fm24cl64b::new(i2c, AddrPin::A000);
```

### Writing

```rust
// Raw bytes.
fram.write(0x0000, &[0xDE, 0xAD, 0xBE, 0xEF])?;

// Typed primitives (little-endian).
fram.write_u32_le(0x0010, 0x1234_5678)?;

// Repeat one value across a range.
fram.fill(0x0100, 64, 0xFF)?;
```

`write` chunks internally at 32 bytes per I²C transaction; the chip itself has no page-write boundary, so the chunking is transparent.

### Reading

```rust
// Selective Read — driver sends the memory address first, then reads N bytes.
let mut buf = [0u8; 8];
fram.read(0x0000, &mut buf)?;

// Typed primitives.
let counter = fram.read_u32_le(0x0010)?;

// Fixed-size arrays, length inferred from the binding.
let header: [u8; 16] = fram.read_array(0x0020)?;
```

### Sequential streaming with `read_current`

After any `read` or `write`, the chip's internal address latch holds the address of the next byte. `read_current` reads from there without re-sending the address — one fewer START + 2 fewer address bytes per call. Prefer it for streaming reads or many small follow-on reads on a single-master bus. The chip itself imposes no length limit (it wraps at `0x1FFF`); the practical limit is your buffer.

```rust
// Position the latch with a normal read…
let mut head = [0u8; 16];
fram.read(0x0000, &mut head)?;          // latch is now at 0x0010

// …then stream the rest from the latch — no address bytes on the wire.
let mut body = [0u8; 240];
fram.read_current(&mut body)?;          // continues 0x0010..0x0100

// Equivalent, but slower (re-sends the address):
// fram.read(0x0010, &mut body)?;
```

Avoid `read_current` if another bus master could access the same chip between calls — it would silently move the latch and shift every byte you read. Use `Cursor` or plain `read` in that case.

### `embedded-storage` interop

The driver implements `ReadStorage` and `Storage`, so anything generic over those traits can drive the chip:

```rust
use embedded_storage::{ReadStorage, Storage};

fn save<S: Storage>(s: &mut S, offset: u32, data: &[u8]) -> Result<(), S::Error> {
    s.write(offset, data)
}

fn load<R: ReadStorage>(r: &mut R, offset: u32, out: &mut [u8]) -> Result<(), R::Error> {
    r.read(offset, out)
}

save(&mut fram, 0x0200, b"hello")?;

let mut buf = [0u8; 5];
load(&mut fram, 0x0200, &mut buf)?;
assert_eq!(&buf, b"hello");

// `ReadStorage::capacity()` returns the chip's MEMORY_SIZE (8192).
assert_eq!(ReadStorage::capacity(&fram), 8192);
```

### Other helpers

```rust
// Cursor: auto-advancing position, multi-master safe (always uses Selective Read).
let mut cur = fram.cursor(0x0100);
let magic   = cur.read_u32_le()?;
let payload = cur.read_array::<32>()?;

// Write then read back to catch WP-active silent-NACK.
if !fram.write_verified(0x0200, &record)? {
    // bus issue or write-protect asserted
}

// Bit-set on a flags byte via single-byte RMW.
fram.modify_byte(0x0300, |b| b | 0x04)?;
```
