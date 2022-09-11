//! Realtime spectral display for BBC Microbit2. Shows audio
//! spectrum computed from the microphone as a bar graph on
//! the LED matrix.

#![no_std]
#![no_main]

/// This is an RTIC app.
use rtic::app;

/// There must be some better way to get at this trait with
/// a better name and place.
#[cfg(not(feature = "adc-multiread"))]
use microbit::hal::prelude::_embedded_hal_adc_OneShot;

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

#[cfg(feature = "defmt")]
use panic_rtt_target as _;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use rtt_target::{rtt_init_print, rprint, rprintln};

/// FFT input width in samples. Output is half this width + 1 for DC.
const FFT_WIDTH: usize = 64;
/// XXX Match this to FFT_WIDTH or fail.
use microfft::real::rfft_64 as rfft;

/// Number of FFTS to accumulate over before displaying a value.
const FFTS_PER_FRAME: usize = 16;
/// ADC is signed; it is configured for a sample precision of 10 bits. Must match below.
const ADC_MAX: usize = 512;

/// Display rate in frames per second. Cannot be too slow,
/// lest hardware overflow. Cannot be too fast, lest display
/// fail to go.  10-30FPS is known to work.
const FRAME_RATE: usize = 10;

/// FFT bins to accumulate over for each of the five bars. Each range is
/// an integer interval [l,u).
const BANDS: [(usize, usize); 5] = [(1, 2), (2, 3), (3, 4), (4, 7), (8, 20)];

/// Struct for dealing with the microphone. Special since
/// it can use a custom read method.
pub struct Microphone {
    /// Successive-approximation ADC
    saadc: Saadc,
    /// Unfortunate hardwired type.
    mic_in: P0_05<Input<Floating>>,
}

impl Microphone {
    /// Configure the microphone.
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

    /// Read a chunk of samples from the microphone.
    fn read(&mut self) -> Result<[i16; FFT_WIDTH], Error<()>> {
        // Setup.
        let mut result = [0; FFT_WIDTH];
        #[cfg(feature = "defmt-trace")]
        rprint!("starting read...");

        // For multiread, just have all the samples read in one go.
        #[cfg(feature = "adc-multiread")]
        self.saadc.read_mut(&mut self.mic_in, &mut result)?;

        // For non-multiread, have them read one at a time.
        #[cfg(not(feature = "adc-multiread"))]
        for r in &mut result {
            *r = self.saadc.read(&mut self.mic_in)?;
        }

        // All done.
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
        /// The display is shared between the renderer and the spectrum calc.
        display: Display<TIMER1>,
    }

    #[local]
    struct Local {
        /// The microphone is used for the spectrum calc.
        mic: Microphone,
        /// The window modifies the input samples to the FFT.
        window: [f32; FFT_WIDTH],
    }

    /// Refreshing the display is high-priority and outside our control.
    #[task(binds = TIMER1, shared = [display], priority = 8)]
    fn timer1(mut cx: timer1::Context) {
        cx.shared.display.lock(|display| display.handle_display_event());
    }

    /// Where the real work happens.
    #[task(binds = RTC0, priority = 1, shared = [display], local = [mic, window])]
    fn spectrum(mut cx: spectrum::Context) {
        // Need a small non-zero number of dB to treat as a pseudo-floor.
        const MIN_DB: f32 = -120.0;
        // Convert a known-non-NaN number to a `NotNaN`. Ugh.
        const fn nn(f: f32) -> NotNan<f32> {
            unsafe { NotNan::new_unchecked(f) }
        }

        // Accumulated power information for this cycle.
        let mut bandpowers = [MIN_DB; BANDS.len()];
        for _ in 0..FFTS_PER_FRAME {
            // Set up the sample buffer.
            let cx = &mut cx.local;
            let samples = cx.mic.read().expect("mic?");
            let mut sample_buf = [0.0f32; FFT_WIDTH];
            for (i, s) in sample_buf.iter_mut().enumerate() {
                *s = samples[i].abs() as f32 / ADC_MAX as f32;
            }
            let dc = sample_buf.iter().copied().sum::<f32>() / sample_buf.len() as f32;
            let peak = sample_buf.iter().copied().map(nn).max().unwrap().into_inner();
            for (i, s) in sample_buf.iter_mut().enumerate() {
                *s = cx.window[i] * (*s - dc) / peak;
            }

            // Take the FFT and process the result.
            let freqs: &mut [Complex<f32>; FFT_WIDTH / 2] = rfft(&mut sample_buf);
            let bandvals = BANDS.into_iter().map(|(start, end)| {
                freqs[start..end]
                    .iter()
                    .map(|&f| {
                        let p = f.norm() / FFT_WIDTH as f32;
                        NotNan::new(20.0 * p.log10()).unwrap_or(nn(MIN_DB))
                    })
                    .max()
                    .unwrap()
                    .into_inner()
            });
            for (bp, bv) in bandpowers.iter_mut().zip(bandvals) {
                *bp = bp.max(bv);
            }
        }

        // Create the display frame from the power data.
        let mut led_display = [[0; 5]; 5];
        for (f, power) in bandpowers.iter().enumerate() {
            for (a, row) in led_display.iter_mut().enumerate() {
                let threshold = -3.0 * a as f32 - 40.0;
                let light = 3.0 * (power - threshold);
                row[f] = light.clamp(0.0, 9.0) as u8;
            }
        }

        // Load up the display.
        cx.shared.display.lock(|display| display.show(&GreyscaleImage::new(&led_display)));
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        use core::f32::consts::PI;

        // Display debugging info if enabled.
        #[cfg(feature = "defmt")]
        rtt_init_print!();
        #[cfg(feature = "defmt-trace")]
        rprintln!("starting...");

        let board = Board::new(cx.device, cx.core);

        // Set up the RTC interrupt.  First, start the
        // low-frequency clock (needed for RTC to work)
        Clocks::new(board.CLOCK).start_lfclk();
        // For 30FPS the RTC will be about 29.98Hz (32_768 /
        // (1092 + 1)), about 33.4ms period.
        const FRAME_TIME: u32 = 32768 / FRAME_RATE as u32 - 1;
        let mut rtc0 = Rtc::new(board.RTC0, FRAME_TIME).unwrap();
        rtc0.enable_event(RtcInterrupt::Tick);
        rtc0.enable_interrupt(RtcInterrupt::Tick, None);
        rtc0.enable_counter();

        // Turn the display on.
        let display = Display::new(board.TIMER1, board.display_pins);
        rtic::pend(Interrupt::TIMER1);

        let shared = Shared { display };

        // Turn the microphone on.
        let mic = Microphone::new(board.SAADC, board.microphone_pins);

        // Compute the FFT window.
        let mut window = [0.0f32; FFT_WIDTH];
        // Use a Hann window, which is easy to compute and reasonably good.
        for (n, w) in window.iter_mut().enumerate() {
            let s = f32::sin(PI * n as f32 / (FFT_WIDTH - 1) as f32);
            *w = s * s;
        }

        let local = Local { mic, window };

        (shared, local, init::Monotonics())
    }
}
