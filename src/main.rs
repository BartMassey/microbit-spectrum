#![no_std]
#![no_main]

use nb::Error;

use defmt_rtt as _;
use panic_halt as _;

use cortex_m_rt::entry;

use microbit::{
    board::Board,
    display::blocking::Display,
    gpio::MicrophonePins,
    hal::{
        prelude::*,
        gpio::{Level, OpenDrainConfig},
        Timer,
        pac::SAADC,
        Saadc,
        saadc::SaadcConfig,
    },
};

use microbit::{
    hal::gpio::{
        p0::P0_05,
        Input,
        Floating,
    }
};

struct Microphone {
    saadc: Saadc,
    mic_in: P0_05<Input<Floating>>,
}

impl Microphone {
    fn new(saadc: SAADC, microphone_pins: MicrophonePins) -> Self {
        // initialize adc
        let saadc_config = SaadcConfig::default();
        let saadc = Saadc::new(saadc, saadc_config);
        let mic_in = microphone_pins.mic_in.into_floating_input();

        // enable microphone
        microphone_pins
            .mic_run
            .into_open_drain_output(OpenDrainConfig::Disconnect0HighDrive1, Level::High);

        Self { saadc, mic_in }
    }

    fn read(&mut self) -> Result<i16, Error<()>> {
        Ok(self.saadc.read(&mut self.mic_in)?)
    }
}

#[entry]
fn main() -> ! {
    if let Some(board) = Board::take() {
        let mut timer = Timer::new(board.TIMER0);
        let mut display = Display::new(board.display_pins);
        let mut mic = Microphone::new(board.SAADC, board.microphone_pins);

        let mut led_display = [[0; 5]; 5];
        let mut dc = 0u64;
        let mut count = 0u64;

        loop {
            let mut peak = 0;
            for _ in 0..256 {
                let sample = mic.read().expect("mic?");
                dc += sample as u64;
                count += 1;
                peak = peak.max(sample.abs());
            }
            if count % 1024 == 0 {
                let dcavg = dc / count;
                for j in 0..5 {
                    led_display[2][j] = ((peak as u64 - dcavg) > 20 * j as u64) as u8;
                }
                display.show(&mut timer, led_display, 10);
            }
        }
    }
    panic!("board?");
}
