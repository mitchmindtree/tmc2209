#![no_std]

#[macro_use]
extern crate bitfield;

pub mod reg;

use core::convert::TryFrom;
use embedded_hal as hal;

#[doc(inline)]
pub use self::reg::{ReadableRegister, Register, WritableRegister};

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

type ReadRequestData = [u8; ReadRequest::LEN_BYTES];
type ReadResponseData = [u8; ReadResponse::LEN_BYTES];
type WriteRequestData = [u8; WriteRequest::LEN_BYTES];

/// The read access request datagram.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct ReadRequest(ReadRequestData);

/// The read access response datagram.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct ReadResponse(ReadResponseData);

/// The write access request datagram.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
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
            while self.index == 0 {
                match bytes.get(0) {
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
                match bytes.get(0) {
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
                        self.index = 0;
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
        self.reg_addr().map(|addr| reg::State::from_addr_and_data(addr, self.data_u32()))
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
        bytes_to_u32([d[0], d[1], d[2], d[3]])
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
}

impl WriteRequest {
    /// The length of the message in bytes.
    pub const LEN_BYTES: usize = 8;

    /// Construct a new `ReadAccess` datagram.
    pub fn new<R>(slave_addr: u8, register: R) -> Self
    where
        R: reg::WritableRegister,
    {
        const WRITE: u8 = 0b10000000;
        let reg_addr_rw = R::ADDRESS as u8 | WRITE;
        let [b0, b1, b2, b3] = u32_to_bytes(register.into());
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
    U: hal::blocking::serial::Write<u8>,
{
    let req = read_request::<R>(slave_addr);
    uart_tx.bwrite_all(req.bytes())
}

/// Construct a write access datagram for register `R` of the slave at the given address and
/// send it via UART.
///
/// This simply calls `write_request` internally before writing the request via `U::bwrite_all`.
pub fn send_write_request<R, U>(slave_addr: u8, reg: R, uart_tx: &mut U) -> Result<(), U::Error>
where
    R: WritableRegister,
    U: hal::blocking::serial::Write<u8>,
{
    let req = write_request(slave_addr, reg);
    uart_tx.bwrite_all(req.bytes())
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
    U: hal::serial::Read<u8>,
{
    let mut reader = Reader::default();
    loop {
        if let Ok(b) = uart_rx.read() {
            if let (_, Some(response)) = reader.read_response(&[b]) {
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
    U: hal::serial::Read<u8>,
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
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    crc
}

// Helper function for converting a `u32` to bytes in the order they should be written in an access
// request datagram.
fn u32_to_bytes(u: u32) -> [u8; 4] {
    let b0 = (u >> 24) as u8;
    let b1 = (u >> 16) as u8;
    let b2 = (u >> 8) as u8;
    let b3 = u as u8;
    [b0, b1, b2, b3]
}

// Helper function for converting the bytes of the data field of an access response datagram to a
// `u32` value ready for conversion to a register bitfield.
fn bytes_to_u32([b0, b1, b2, b3]: [u8; 4]) -> u32 {
    let mut u = 0u32;
    u |= (b0 as u32) << 24;
    u |= (b1 as u32) << 16;
    u |= (b2 as u32) << 8;
    u |= b3 as u32;
    u
}
