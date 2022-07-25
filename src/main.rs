#![no_std]
#![no_main]

use num_complex::ComplexFloat;

use nb::Error;

use microfft::real::rfft_32;

use panic_halt as _;

use cortex_m_rt::entry;

use microbit::{
    board::Board,
    display::blocking::Display,
    gpio::MicrophonePins,
    hal::{
        prelude::*,
        gpio::{
            Level,
            OpenDrainConfig,
            p0::P0_05,
            Input,
            Floating,
        },
        Timer,
        pac::SAADC,
        Saadc,
        saadc::{
            SaadcConfig,
            Resolution,
            Oversample,
            Reference,
            Gain,
            Resistor,
            Time,
        }
    },
};

// use rtt_target::{rprintln, rtt_init_print};

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
            reference: Reference::VDD1_4,
            gain: Gain::GAIN1_4,
            resistor: Resistor::BYPASS,
            time: Time::_40US,
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

#[entry]
fn main() -> ! {
    let board = Board::take().expect("board?");
    let mut timer = Timer::new(board.TIMER0);
    let mut display = Display::new(board.display_pins);
    let mut mic = Microphone::new(board.SAADC, board.microphone_pins);

    let mut led_display = [[0; 5]; 5];
    let mut sample_buf = [0.0f32; 32];
    let mut dc = 0.0f32;
    let mut peak = 0.0f32;

    loop {
        for s in &mut sample_buf {
            let sample = mic.read().expect("mic?") as f32 / 2048.0;
            dc = (7.0 * dc + sample) / 8.0;
            let sample = sample - dc;
            peak = peak.max(sample.abs());
            *s = sample / peak;
        }
        let freqs: &mut [num_complex::Complex<f32>; 16] = rfft_32(&mut sample_buf);
        for f in 0..5 {
            let power = 20.0 * (freqs[f + 2].norm() / 16.0).log10();
            for a in 0..5 {
                led_display[a][f] = (55.0 + power > 5.0 * (4.0 - a as f32)) as u8;
            }
        }
        display.show(&mut timer, led_display, 10);
    }
}
