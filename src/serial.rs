use core::fmt::Write as FWrite;
use embedded_io::{Read as ERead, Write as EWrite, ReadExactError};
use microbit::{
    hal::uarte::{
        self, Baudrate, Error as UError, Parity, Instance,
        Uarte, UarteRx, UarteTx
    },
    pac::UARTE0,
    board::UartPins,
};


pub fn serial(uarte0: UARTE0, uart: UartPins) -> UartePort<UARTE0> {
    let serial = uarte::Uarte::new(
        uarte0,
        uart.into(),
        Parity::EXCLUDED,
        Baudrate::BAUD115200,
    );
    UartePort::new(serial)
}

static mut TX_BUF: [u8; 1] = [0; 1];
static mut RX_BUF: [u8; 1] = [0; 1];

pub struct UartePort<T: Instance>(UarteTx<T>, UarteRx<T>);

impl<T: Instance> UartePort<T> {
    pub fn new(serial: Uarte<T>) -> UartePort<T> {
        #[allow(static_mut_refs)]
        let (tx, rx) = serial
            .split(unsafe { &mut TX_BUF }, unsafe { &mut RX_BUF })
            .unwrap();
        UartePort(tx, rx)
    }

    pub fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.write_str(s)
    }

    pub fn write(&mut self, b: u8) -> Result<usize, UError> {
        self.0.write(&[b])
    }

    pub fn flush(&mut self) -> Result<(), UError> {
        self.0.flush()
    }

    pub fn read(&mut self) -> nb::Result<u8, ReadExactError<UError>> {
        let mut buf = [0u8];
        self.1.read_exact(&mut buf)?;
        Ok(buf[0])
    }
}

impl<T: Instance> FWrite for UartePort<T> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_str(s)
    }
}

pub fn ding<T: Instance>(serial: &mut UartePort<T>, count: u64) {
    write!(serial, "ding: {}\r\n", count).unwrap();
    serial.flush().unwrap();
}
