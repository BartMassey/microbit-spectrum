use microbit::{
    hal::uarte::{self, Baudrate, Parity, Error, Instance, Uarte, UarteRx, UarteTx},
    board::UartPins,
};
use embedded_hal::blocking::serial as bserial;
use embedded_hal::serial;

pub use microbit::hal::pac::UARTE0;

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
        let (tx, rx) = serial
            .split(unsafe { &mut TX_BUF }, unsafe { &mut RX_BUF })
            .unwrap();
        UartePort(tx, rx)
    }
}

impl<T: Instance> core::fmt::Write for UartePort<T> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.write_str(s)
    }
}

impl<T: Instance> serial::Write<u8> for UartePort<T> {
    type Error = Error;

    fn write(&mut self, b: u8) -> nb::Result<(), Self::Error> {
        self.0.write(b)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.0.flush()
    }
}

impl<T: Instance> bserial::write::Default<u8> for UartePort<T> {}

impl<T: Instance> serial::Read<u8> for UartePort<T> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.1.read()
    }
}

pub fn try_me<T: Instance>(serial: &mut UartePort<T>) {
    use core::fmt::Write;
    use embedded_hal::prelude::_embedded_hal_serial_Write;
    write!(serial, "The quick brown fox jumps over the lazy dog.\r\n").unwrap();
    nb::block!(serial.flush()).unwrap();
}
