#![no_std]
#![no_main]

use core::cell::RefCell;
use core::f32::consts::PI;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use microbit::{
    board::Board,
    display::nonblocking::{Display, GreyscaleImage},
    gpio::MicrophonePins,
    hal::prelude::*,
    hal::{
        gpio::{p0::P0_05, Floating, Input, Level, OpenDrainConfig},
        pac::SAADC,
        saadc::{Oversample, Resolution, SaadcConfig, Time},
        Saadc,
    },
    pac::{self, interrupt, TIMER0},
};
use microfft::real::rfft_32;
use nb::Error;
use num_complex::ComplexFloat;
use panic_halt as _;

static DISPLAY: Mutex<RefCell<Option<Display<TIMER0>>>> = Mutex::new(RefCell::new(None));

struct Microphone {
    saadc: Saadc,
    mic_in: P0_05<Input<Floating>>,
}

impl Microphone {
    fn new(saadc: SAADC, microphone_pins: MicrophonePins) -> Self {
        // initialize adc
        let saadc_config = SaadcConfig {
            resolution: Resolution::_12BIT,
            oversample: Oversample::OVER4X,
            time: Time::_40US,
            ..SaadcConfig::default()
        };
        let saadc = Saadc::new(saadc, saadc_config);
        let mic_in = microphone_pins.mic_in.into_floating_input();

        // enable microphone
        microphone_pins
            .mic_run
            .into_open_drain_output(OpenDrainConfig::Disconnect0HighDrive1, Level::High);

        Self { saadc, mic_in }
    }

    fn read(&mut self) -> Result<i16, Error<()>> {
        self.saadc.read(&mut self.mic_in)
    }
}

#[interrupt]
fn TIMER0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(display) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
            display.handle_display_event();
        }
    });
}

#[entry]
fn main() -> ! {
    let mut board = Board::take().expect("board?");

    let mut mic = Microphone::new(board.SAADC, board.microphone_pins);

    let display = Display::new(board.TIMER0, board.display_pins);
    cortex_m::interrupt::free(move |cs| {
        *DISPLAY.borrow(cs).borrow_mut() = Some(display);
    });
    unsafe {
        board.NVIC.set_priority(pac::Interrupt::TIMER0, 128);
        pac::NVIC::unmask(pac::Interrupt::TIMER0);
    }

    let mut led_display = [[0; 5]; 5];
    let mut sample_buf = [0.0f32; 32];
    let mut dc = 0.0f32;
    let mut peak = 0.0f32;
    let mut window = [0.0f32; 32];
    // Use a Hann window, which is easy to compute and reasonably good.
    for (n, w) in window.iter_mut().enumerate() {
        let s = (PI * (n as f32 + 1.0) / 34.0).sin();
        *w = s * s;
    }

    loop {
        for (i, s) in sample_buf.iter_mut().enumerate() {
            let sample = mic.read().expect("mic?") as f32 / 2048.0;
            dc = (7.0 * dc + sample) / 8.0;
            let sample = sample - dc;
            peak = peak.max(sample.abs());
            *s = sample * window[i] / peak;
        }
        let freqs: &mut [num_complex::Complex<f32>; 16] = rfft_32(&mut sample_buf);
        for f in 0..5 {
            let power = 20.0 * (freqs[f + 2].norm() / 16.0).log10();
            for (a, row) in led_display.iter_mut().enumerate() {
                row[f] = (55.0 + power > 5.0 * (4.0 - a as f32)) as u8;
            }
        }
        cortex_m::interrupt::free(|cs| {
            if let Some(display) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
                display.show(&GreyscaleImage::new(&led_display));
            }
        });
    }
}
