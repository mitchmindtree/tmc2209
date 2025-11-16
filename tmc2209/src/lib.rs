#![no_std]

#[macro_use]
extern crate bitfield;

pub mod reg;

use core::convert::TryFrom;
use embedded_io::{Read, Write};

#[doc(inline)]
pub use self::reg::{ReadableRegister, Register, WritableRegister};

/// The frequency of the clock that is internal to the TMC2209.
///
/// Sometimes referred to as `fclk` in the datasheet.
pub const INTERNAL_CLOCK_HZ: f32 = 12_000_000.0;

/// A serial reader, for reading responses via the TMC2209's UART interface.
///
/// The `Reader` is stateful and stores its progress between calls to `read`.
#[derive(Default)]
pub struct Reader {
    /// The current index into the response data that we're looking for.
    index: usize,
    /// The data for a response being interpreted.
    response_data: ReadResponseData,
}

/// For observing the current state of the reader.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub enum ReaderAwaiting {
    Sync,
    MasterAddr,
    RegAddr,
    DataByte0,
    DataByte1,
    DataByte2,
    DataByte3,
    Crc,
}

type ReadRequestData = [u8; ReadRequest::LEN_BYTES];
type ReadResponseData = [u8; ReadResponse::LEN_BYTES];
type WriteRequestData = [u8; WriteRequest::LEN_BYTES];

/// The read access request datagram.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct ReadRequest(ReadRequestData);

/// The read access response datagram.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct ReadResponse(ReadResponseData);

/// The write access request datagram.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "hash", derive(hash32_derive::Hash32))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "ufmt", derive(ufmt::derive::uDebug))]
pub struct WriteRequest(WriteRequestData);

/// The first four bits for synchronisation, the last four are zeroed reserved bytes.
pub const SYNC_AND_RESERVED: u8 = 0b00000101;

/// Responses are always addressed to the master with this value.
pub const MASTER_ADDR: u8 = 0b11111111;

impl Reader {
    /// Read a **ReadResponse** from the given slice of bytes.
    ///
    /// The **Reader** will preserve its state between calls to `read`.
    ///
    /// This method is particularly useful when reading from a UART receiver in a non-blocking
    /// manner. For example, if the UART receiver does not have the full response buffered (perhaps
    /// within a UART interrupt), the `Reader` retains its progress and allows for other work to be
    /// performed before attempting to read the remainder of the message.
    ///
    /// Returns the number of bytes read and the **ReadResponse** if one could be read.
    ///
    /// This method works by reading through the bytes for a `SYNC_AND_RESERVED` byte. Once found,
    /// the next byte must be a `MASTER_ADDR` byte. If found, the rest of the data is read directly
    /// into the remainder of the byte slice. Otherwise, the state is rest and the reader goes back
    /// to searching for a `SYNC_AND_RESERVED` byte.
    ///
    /// This function does **not** check the validity of the CRC, the slave address or the register
    /// address.
    pub fn read_response(&mut self, mut bytes: &[u8]) -> (usize, Option<ReadResponse>) {
        let start_len = bytes.len();
        loop {
            // If we're at the first index, we're looking for the sync byte.
            while self.index == ReadResponse::SYNC_AND_RESERVED_IX {
                match bytes.first() {
                    Some(&SYNC_AND_RESERVED) => {
                        self.response_data[self.index] = SYNC_AND_RESERVED;
                        self.index += 1;
                    }
                    None => {
                        let read_bytes = start_len - bytes.len();
                        return (read_bytes, None);
                    }
                    _ => (),
                };
                bytes = &bytes[1..];
            }

            // Make sure the following byte is addressed to the master.
            if self.index == ReadResponse::MASTER_ADDR_IX {
                match bytes.first() {
                    Some(&MASTER_ADDR) => {
                        self.response_data[self.index] = MASTER_ADDR;
                        self.index += 1;
                        bytes = &bytes[1..];
                    }
                    None => {
                        let read_bytes = start_len - bytes.len();
                        return (read_bytes, None);
                    }
                    _ => {
                        self.index = ReadResponse::SYNC_AND_RESERVED_IX;
                        continue;
                    }
                }
            }

            // Copy the remaining data.
            let remaining_data = &mut self.response_data[self.index..];
            let to_copy = core::cmp::min(remaining_data.len(), bytes.len());
            remaining_data
                .iter_mut()
                .zip(bytes)
                .for_each(|(d, b)| *d = *b);
            self.index += to_copy;
            bytes = &bytes[to_copy..];

            // Return a response if we've read one.
            let read_bytes = start_len - bytes.len();
            return if self.index == ReadResponse::LEN_BYTES {
                self.index = 0;
                (read_bytes, Some(ReadResponse(self.response_data)))
            } else {
                (read_bytes, None)
            };
        }
    }

    /// Which byte the reader is currently awaiting.
    ///
    /// This method is purely to assist with debugging UART communication issues.
    pub fn awaiting(&self) -> ReaderAwaiting {
        match self.index {
            ReadResponse::SYNC_AND_RESERVED_IX => ReaderAwaiting::Sync,
            ReadResponse::MASTER_ADDR_IX => ReaderAwaiting::MasterAddr,
            ReadResponse::REG_ADDR_IX => ReaderAwaiting::RegAddr,
            3 => ReaderAwaiting::DataByte0,
            4 => ReaderAwaiting::DataByte1,
            5 => ReaderAwaiting::DataByte2,
            6 => ReaderAwaiting::DataByte3,
            ReadResponse::CRC_IX => ReaderAwaiting::Crc,
            _ => unreachable!(),
        }
    }
}

impl ReadRequest {
    /// The length of the message in bytes.
    pub const LEN_BYTES: usize = 4;

    /// Construct a new read access datagram.
    pub fn new<R>(slave_addr: u8) -> Self
    where
        R: reg::ReadableRegister,
    {
        Self::from_addr(slave_addr, R::ADDRESS)
    }

    /// Should this be exposed? Doesn't protect against specifying a non-readable register.
    pub fn from_addr(slave_addr: u8, register: reg::Address) -> Self {
        const READ: u8 = 0b00000000;
        let reg_addr_rw = (register as u8 | READ) & 0x7F;
        let mut bytes = [SYNC_AND_RESERVED, slave_addr, reg_addr_rw, 0u8];
        let crc_ix = bytes.len() - 1;
        bytes[crc_ix] = crc(&bytes[..crc_ix]);
        Self(bytes)
    }

    /// The request as a slice of bytes, ready to be written via the serial interface.
    pub fn bytes(&self) -> &[u8] {
        &self.0[..]
    }
}

impl ReadResponse {
    /// The length of the message in bytes.
    pub const LEN_BYTES: usize = 8;
    /// The first byte is the synchronisation byte.
    pub const SYNC_AND_RESERVED_IX: usize = 0;
    /// The index of the master address byte.
    pub const MASTER_ADDR_IX: usize = 1;
    /// The index of the register address.
    pub const REG_ADDR_IX: usize = 2;
    /// The range of byte representing the data field.
    pub const DATA_RANGE: core::ops::Range<usize> = 3..7;
    /// The index of the cyclic rendundency check.
    pub const CRC_IX: usize = 7;

    /// The master address.
    ///
    /// Should always be `0b11111111`.
    pub fn master_addr(&self) -> u8 {
        self.0[Self::MASTER_ADDR_IX]
    }

    /// Read the register address field.
    ///
    /// Returns an `Err` if the value does not correspond with any known register.
    pub fn reg_addr(&self) -> Result<reg::Address, reg::UnknownAddress> {
        reg::Address::try_from(self.0[Self::REG_ADDR_IX])
    }

    /// Produce the register state stored within the resonse.
    ///
    /// The specific state is determined by first checking the register address.
    pub fn reg_state(&self) -> Result<reg::State, reg::UnknownAddress> {
        self.reg_addr()
            .map(|addr| reg::State::from_addr_and_data(addr, self.data_u32()))
    }

    /// The data slice.
    pub fn data(&self) -> &[u8] {
        &self.0[Self::DATA_RANGE]
    }

    /// The bytes of the data slice shifted into a `u32` value.
    ///
    /// The `u32` can be converted directly into the `Register` type associated with the
    /// `reg::Address`. See the `register` method.
    pub fn data_u32(&self) -> u32 {
        let d = self.data();
        u32::from_be_bytes([d[0], d[1], d[2], d[3]])
    }

    /// Attempt to cast the `data` field to a register bitfield of the given type.
    ///
    /// Uses the `data_u32` method internally.
    pub fn register<R>(&self) -> Result<R, reg::UnknownAddress>
    where
        R: ReadableRegister,
    {
        match self.reg_addr() {
            Ok(addr) if addr == R::ADDRESS => Ok(R::from(self.data_u32())),
            _ => Err(reg::UnknownAddress),
        }
    }

    /// Returns `true` if the CRC is valid, `false` otherwise.
    pub fn crc_is_valid(&self) -> bool {
        self.0[Self::CRC_IX] == crc(&self.0[..Self::CRC_IX])
    }

    /// The inner slice of bytes.
    pub fn bytes(&self) -> &[u8; Self::LEN_BYTES] {
        &self.0
    }
}

impl WriteRequest {
    /// The length of the message in bytes.
    pub const LEN_BYTES: usize = 8;

    /// Construct a new `ReadAccess` datagram.
    pub fn new<R>(slave_addr: u8, register: R) -> Self
    where
        R: reg::WritableRegister,
    {
        Self::from_state(slave_addr, register.into())
    }

    /// A dynamic alternative to `new`, for when the exact register begin written to is not known
    /// at compile time.
    ///
    /// TODO: Return a `Result` where an `Err` is returned if a non-write-able register was
    /// specified.
    pub fn from_state(slave_addr: u8, state: reg::State) -> Self {
        const WRITE: u8 = 0b10000000;
        let reg_addr_rw = state.addr() as u8 | WRITE;
        let [b0, b1, b2, b3] = u32::to_be_bytes(state.into());
        let mut bytes = [
            SYNC_AND_RESERVED,
            slave_addr,
            reg_addr_rw,
            b0,
            b1,
            b2,
            b3,
            0u8,
        ];
        let crc_ix = bytes.len() - 1;
        bytes[crc_ix] = crc(&bytes[..crc_ix]);
        Self(bytes)
    }

    /// The request as a slice of bytes, ready to be written via the serial interface.
    pub fn bytes(&self) -> &[u8] {
        &self.0[..]
    }
}

/// Construct a new read access datagram for register `R` of the slave at the given address.
///
/// Shorthand for `ReadRequest::new`.
pub fn read_request<R>(slave_addr: u8) -> ReadRequest
where
    R: reg::ReadableRegister,
{
    ReadRequest::new::<R>(slave_addr)
}

/// Construct a new write access datagram for register `R` of the slave at the given address.
///
/// Shorthand for `WriteRequest::new`.
pub fn write_request<R>(slave_addr: u8, register: R) -> WriteRequest
where
    R: reg::WritableRegister,
{
    WriteRequest::new::<R>(slave_addr, register)
}

/// Construct a read access datagram for register `R` of the slave at the given address and send it
/// via UART.
///
/// This simply calls `read_request` internally before writing the request via `U::bwrite_all`.
pub fn send_read_request<R, U>(slave_addr: u8, uart_tx: &mut U) -> Result<(), U::Error>
where
    R: reg::ReadableRegister,
    U: Write,
{
    let req = read_request::<R>(slave_addr);
    uart_tx.write_all(req.bytes())
}

/// Construct a write access datagram for register `R` of the slave at the given address and
/// send it via UART.
///
/// This simply calls `write_request` internally before writing the request via `U::bwrite_all`.
pub fn send_write_request<R, U>(slave_addr: u8, reg: R, uart_tx: &mut U) -> Result<(), U::Error>
where
    R: WritableRegister,
    U: Write,
{
    let req = write_request(slave_addr, reg);
    uart_tx.write_all(req.bytes())
}

/// Blocks and attempts to read a response from the given UART receiver.
///
/// The response might be from any register, so the `reg_addr` on the returned response should be
/// checked.
///
/// **NOTE:** Currently, this function will block until a response can be successfully read. This
/// means the function may block forever in the case some communication error occurs. Once a
/// generic timer API lands in the `embedded-hal` crate, this should be changed to take a timeout
/// duration and return with a timeout error in the case that the duration is exceeded before a
/// response can be read. In the meantime, consider using the `Reader` and its `read_response`
/// method directly to avoid blocking or apply your own timeout logic.
pub fn await_read_response<U>(uart_rx: &mut U) -> ReadResponse
where
    U: Read,
{
    let mut reader = Reader::default();
    let mut buf = [0u8; 128];
    loop {
        if let Ok(_) = uart_rx.read(&mut buf) {
            if let (_, Some(response)) = reader.read_response(&buf) {
                return response;
            }
        }
    }
}

/// Blocks and attempts to read a register from the given UART receiver.
///
/// Expects the response to be addressed to the register of type `R` and returns an error
/// otherwise.
///
/// Internally, this calls `await_read_response` and then the `register` method of the resulting
/// response.
///
/// **NOTE:** Currently, this function will block until a response can be successfully read. This
/// means the function may block forever in the case some communication error occurs. Once a
/// generic timer API lands in the `embedded-hal` crate, this should be changed to take a timeout
/// duration and return with a timeout error in the case that the duration is exceeded before a
/// response can be read. In the meantime, consider using the `Reader` and its `read_response`
/// method directly to avoid blocking or apply your own timeout logic.
pub fn await_read<R, U>(uart_rx: &mut U) -> Result<R, reg::UnknownAddress>
where
    R: ReadableRegister,
    U: Read,
{
    let res = await_read_response(uart_rx);
    res.register::<R>()
}

/// Cyclic redundancy check.
///
/// Given all data of a datagram apart from the final CRC byte, return what the CRC byte should be.
pub fn crc(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut byte in data.iter().cloned() {
        for _ in 0..8 {
            if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            byte >>= 1;
        }
    }
    crc
}

/// Use the selected sense resistor value (in ohms) and the motor's rated current (in mA) to
/// determine the recommended vsense and "current scale" (for IRUN) values.
// Code referenced from `TMCStepper.cpp` `rms_current` function in TMC demo source.
pub fn rms_current_to_vsense_cs(rsense: f32, ma: f32) -> (bool, u8) {
    let mut cs: u8 = (32.0 * 1.41421 * ma / 1_000.0 * (rsense + 0.02) / 0.325 - 1.0) as u8;

    // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
    if cs < 16 {
        cs = (32.0 * 1.41421 * ma / 1_000.0 * (rsense + 0.02) / 0.180 - 1.0) as u8;
        (true, cs)
    } else {
        cs = core::cmp::min(cs, 31);
        (false, cs)
    }
}

/// Use the selected sense resistor value (in ohms) and the current `vsense` setting to convert
/// the given "current scale" value to an RMS current in mA.
///
/// Useful for converting `CS_ACTUAL` to a human readable value.
// Code referenced from `TMCStepper.cpp` `cs2rms` function in TMC demo source.
pub fn vsense_cs_to_rms_current(rsense: f32, vsense: bool, cs: u8) -> f32 {
    let vsense = if vsense { 0.180 } else { 0.325 };
    (cs + 1) as f32 / 32.0 * vsense / (rsense + 0.02) / 1.41421 * 1_000.0
}

/// Convert the given frequency to the closest TOFF value.
///
/// Returns `None` if the TOFF value would be 0, as this value disables motor.
///
/// Note that the TMC only supports a discrete collection of 15 chopper frequencies in the range
/// between 5,952.381hz and 53,571.426hz.
pub fn chopper_hz_to_toff(fclk: f32, hz: f32) -> Option<u8> {
    let toff_secs = chopper_hz_to_toff_secs(hz);
    let toff_f = toff_secs_to_toff_reg(fclk, toff_secs);
    let toff = core::cmp::min(round_f32(toff_f) as u8, 15);
    if toff == 0 {
        None
    } else {
        Some(toff)
    }
}

/// Convert the given TOFF value to the associated chopper frequency.
///
/// Returns `None` if the given value is `< 1` or `> 15`.
pub fn toff_to_chopper_hz(fclk: f32, toff: u8) -> Option<f32> {
    if !(1..=15).contains(&toff) {
        None
    } else {
        let toff_secs = toff_reg_to_toff_secs(fclk, toff as f32);
        let hz = toff_secs_to_chopper_hz(toff_secs);
        Some(hz)
    }
}

fn chopper_hz_to_toff_secs(hz: f32) -> f32 {
    1.0 / hz * 0.25
}

fn toff_secs_to_chopper_hz(toff_secs: f32) -> f32 {
    1.0 / toff_secs / 4.0
}

fn toff_secs_to_toff_reg(fclk: f32, toff_secs: f32) -> f32 {
    (toff_secs * fclk - 24.0) / 32.0
}

fn toff_reg_to_toff_secs(fclk: f32, toff_reg: f32) -> f32 {
    (toff_reg * 32.0 + 24.0) / fclk
}

// `#[no_std]` friendly round.
fn round_f32(f: f32) -> f32 {
    let floor = (f as i32) as f32;
    let ceil = match (f as i32).checked_add(1) {
        None => return floor,
        Some(c) => c as f32,
    };
    let dist_floor = f - floor;
    let dist_ceil = ceil - f;
    if dist_floor <= dist_ceil {
        floor
    } else {
        ceil
    }
}
