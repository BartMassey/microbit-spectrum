# microbit-spectrum: Show micro:bit v2 microphone spectrum
Bart Massey 2022

This Rust app for the BBC micro:bit v2 uses its microphone
to listen and displays spectral power. The spectral bar
centers correspond very roughly to 360, 720, 1080, 1440, and
1800 Hz.  The bar height is dB in unspecified units of audio
power: the base LED is 60 dB and each successive LED is 3dB.

## Build and Run

You can follow the instructions from the embedded micro:bit
[*Discovery Book*](https://docs.rust-embedded.org/discovery/microbit/index.html)
to set up your build environment.  Then you can say

    cargo embed --release

to flash and run this.

You can also follow the setup instructions in the `README`
on the `microbit` crate
[repo](https://github.com/nrf-rs/microbit). You can then say

    cargo run --release

Note that this app requires `--release` to work properly:
the FFT is just too slow otherwise.

## License

This work is made available under the "MIT License". Please
see the file `LICENSE.txt` in this distribution for license
terms.

## Acknowledgements

The hardware-using code and build infrastructure here were
heavily derived from the examples in the amazing `microbit`
crate. The `microfft` crate is used for FFT.
