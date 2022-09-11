#![no_std]
#![no_main]

use core::cell::RefCell;
use core::f32::consts::PI;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use microbit::hal::prelude::*;
use microbit::{
    board::Board,
    display::nonblocking::{Display, GreyscaleImage},
    gpio::MicrophonePins,
    hal::{
        gpio::{p0::P0_05, Floating, Input, Level, OpenDrainConfig},
        pac::SAADC,
        saadc::{Oversample, Resolution, SaadcConfig, Time},
        Saadc,
        timer::Timer,
    },
    pac::{self, interrupt, Interrupt, TIMER1},
};
use nb::Error;
use num_complex::Complex;
use num_traits::Float;
use ordered_float::NotNan;
use panic_rtt_target as _;
use rtt_target::rtt_init_print;
#[cfg(not(feature = "defmt-default"))]
use rtt_target::{rprint, rprintln};

const FFT_WIDTH: usize = 64;
use microfft::real::rfft_64 as rfft;
// 10 bits. See also below.
const ADC_MAX: usize = 512;

const BANDS: [(usize, usize); 5] = [(1, 2), (2, 3), (3, 4), (4, 7), (8, 20)];

static DISPLAY: Mutex<RefCell<Option<Display<TIMER1>>>> = Mutex::new(RefCell::new(None));

struct Microphone {
    saadc: Saadc,
    mic_in: P0_05<Input<Floating>>,
}

impl Microphone {
    fn new(saadc: SAADC, microphone_pins: MicrophonePins) -> Self {
        // initialize adc
        let saadc_config = SaadcConfig {
            // Should match ADC_MAX above.
            resolution: Resolution::_10BIT,
            // Limited by resolution and sampling rate
            // including oversampling. See chip docs.
            oversample: Oversample::OVER2X,
            time: Time::_10US,
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

    fn read(&mut self) -> Result<[i16; FFT_WIDTH], Error<()>> {
        let mut result = [0; FFT_WIDTH];
        #[cfg(feature = "defmt-trace")]
        rprint!("starting read...");

        #[cfg(feature = "adc-multiread")]
        self.saadc.read_mut(&mut self.mic_in, &mut result)?;

        #[cfg(not(feature = "adc-multiread"))]
        for r in &mut result {
            *r = self.saadc.read(&mut self.mic_in)?;
        }

        #[cfg(feature = "defmt-trace")]
        rprintln!("read complete");
        Ok(result)
    }
}

#[interrupt]
fn TIMER1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(display) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
            display.handle_display_event();
        }
    });
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    #[cfg(feature = "defmt-trace")]
    rprintln!("starting...");
    let mut board = Board::take().expect("board?");

    let mut timer = Timer::new(board.TIMER4);

    let mut mic = Microphone::new(board.SAADC, board.microphone_pins);

    let display = Display::new(board.TIMER1, board.display_pins);
    cortex_m::interrupt::free(move |cs| {
        *DISPLAY.borrow(cs).borrow_mut() = Some(display);
    });
    unsafe {
        board.NVIC.set_priority(Interrupt::TIMER1, 128);
        pac::NVIC::unmask(Interrupt::TIMER1);
    }

    let mut led_display = [[0; 5]; 5];
    let mut sample_buf = [0.0f32; FFT_WIDTH];
    let mut dc = 0.0f32;
    let mut peak = 0.0f32;
    let mut window = [0.0f32; FFT_WIDTH];
    // Use a Hann window, which is easy to compute and reasonably good.
    for (n, w) in window.iter_mut().enumerate() {
        let s = f32::sin(PI * n as f32 / (FFT_WIDTH - 1) as f32);
        *w = s * s;
    }

    loop {
        let samples = mic.read().expect("mic?");
        for (i, s) in sample_buf.iter_mut().enumerate() {
            let sample = samples[i] as f32 / ADC_MAX as f32;
            dc = (7.0 * dc + sample) / 8.0;
            let sample = sample - dc;
            peak = peak.max(sample.abs());
            *s = sample * window[i] / peak;
        }

        let freqs: &mut [Complex<f32>; FFT_WIDTH / 2] = rfft(&mut sample_buf);
        let bandvals = BANDS.into_iter().map(|(start, end)| {
            freqs[start..end]
                .iter()
                .map(|&f| {
                    let p = f.norm() / FFT_WIDTH as f32;
                    const MIN_DB: NotNan<f32> = unsafe { NotNan::new_unchecked(-120.0) };
                    NotNan::new(20.0 * p.log10()).unwrap_or(MIN_DB)
                })
                .max()
                .unwrap()
                .into_inner()
        });

        for (f, power) in bandvals.enumerate() {
            for (a, row) in led_display.iter_mut().enumerate() {
                let threshold = -3.0 * a as f32 - 50.0;
                let light = power - threshold;
                row[f] = light.clamp(0.0, 9.0) as u8;
            }
        }

        cortex_m::interrupt::free(|cs| {
            if let Some(display) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
                display.show(&GreyscaleImage::new(&led_display));
            }
        });

        timer.delay_ms(500_u16);
    }
}
