#![no_std]
#![no_main]

use rtic::app;

#[cfg(not(feature = "adc-multiread"))]
use microbit::hal::prelude::*;
use microbit::{
    board::Board,
    display::nonblocking::{Display, GreyscaleImage},
    gpio::MicrophonePins,
    hal::{
        clocks::Clocks,
        gpio::{p0::P0_05, Floating, Input, Level, OpenDrainConfig},
        pac::SAADC,
        rtc::{Rtc, RtcInterrupt},
        saadc::{Oversample, Resolution, SaadcConfig, Time},
        Saadc,
    },
    pac::{Interrupt, TIMER1},
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
const FFTS_PER_SAMPLE: usize = 16;
// 10 bits. See also below.
const ADC_MAX: usize = 512;
const FRAME_RATE: usize = 30;

const BANDS: [(usize, usize); 5] = [(1, 2), (2, 3), (3, 4), (4, 7), (8, 20)];

pub struct Microphone {
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

#[app(device=microbit::pac, peripherals=true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        display: Display<TIMER1>,
    }

    #[local]
    struct Local {
        mic: Microphone,
        dc: f32,
        peak: f32,
        window: [f32; FFT_WIDTH],
    }

    #[task(binds = TIMER1, shared = [display], priority = 8)]
    fn timer1(mut cx: timer1::Context) {
        cx.shared.display.lock(|display| display.handle_display_event());
    }

    #[task(binds = RTC0, priority = 1, shared = [display], local = [mic, dc, peak, window])]
    fn spectrum(mut cx: spectrum::Context) {
        const MIN_DB: f32 = -120.0;
        const MIN_DB_NOT_NAN: NotNan<f32> = unsafe { NotNan::new_unchecked(MIN_DB) };
        loop {
            let mut bandpowers = [MIN_DB; BANDS.len()];
            for _ in 0..FFTS_PER_SAMPLE {
                let cx = &mut cx.local;
                let samples = cx.mic.read().expect("mic?");
                let mut sample_buf = [0.0f32; FFT_WIDTH];
                for (i, s) in sample_buf.iter_mut().enumerate() {
                    let sample = samples[i] as f32 / ADC_MAX as f32;
                    *cx.dc = (7.0 * *cx.dc + sample) / 8.0;
                    let sample = sample - *cx.dc;
                    *cx.peak = cx.peak.max(sample.abs());
                    *s = sample * cx.window[i] / *cx.peak;
                }

                let freqs: &mut [Complex<f32>; FFT_WIDTH / 2] = rfft(&mut sample_buf);
                let bandvals = BANDS.into_iter().map(|(start, end)| {
                    freqs[start..end]
                        .iter()
                        .map(|&f| {
                            let p = f.norm() / FFT_WIDTH as f32;
                            NotNan::new(20.0 * p.log10()).unwrap_or(MIN_DB_NOT_NAN)
                        })
                        .max()
                        .unwrap()
                        .into_inner()
                });
                for (bp, bv) in bandpowers.iter_mut().zip(bandvals) {
                    *bp = bp.max(bv);
                }
            }

            let mut led_display = [[0; 5]; 5];
            for (f, power) in bandpowers.iter().enumerate() {
                for (a, row) in led_display.iter_mut().enumerate() {
                    let threshold = -3.0 * a as f32 - 45.0;
                    let light = 3.0 * (power - threshold);
                    row[f] = light.clamp(0.0, 9.0) as u8;
                }
            }

            cx.shared.display.lock(|display| display.show(&GreyscaleImage::new(&led_display)));
        }
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        use core::f32::consts::PI;

        rtt_init_print!();
        #[cfg(feature = "defmt-trace")]
        rprintln!("starting...");

        let board = Board::new(cx.device, cx.core);
        // Starting the low-frequency clock (needed for RTC to work)
        Clocks::new(board.CLOCK).start_lfclk();
        // Default RTC at about 29.98Hz (32_768 / (1092 + 1))
        // about 33.4ms period
        const FRAME_TIME: u32 = 32768 / FRAME_RATE as u32 - 1;
        let mut rtc0 = Rtc::new(board.RTC0, FRAME_TIME).unwrap();
        rtc0.enable_event(RtcInterrupt::Tick);
        rtc0.enable_interrupt(RtcInterrupt::Tick, None);
        rtc0.enable_counter();

        let display = Display::new(board.TIMER1, board.display_pins);
        rtic::pend(Interrupt::TIMER1);

        let shared = Shared { display };

        let mic = Microphone::new(board.SAADC, board.microphone_pins);
        let dc = 0.0f32;
        let peak = 0.0f32;
        let mut window = [0.0f32; FFT_WIDTH];
        // Use a Hann window, which is easy to compute and reasonably good.
        for (n, w) in window.iter_mut().enumerate() {
            let s = f32::sin(PI * n as f32 / (FFT_WIDTH - 1) as f32);
            *w = s * s;
        }

        let local = Local { mic, dc, peak, window };

        (shared, local, init::Monotonics())
    }
}
