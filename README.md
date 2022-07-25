# microbit-spectrum: Show micro:bit v2 microphone spectrum
Bart Massey 2022

This Rust app for the BBC micro:bit v2 uses its microphone
to listen and displays spectral power. The spectral bars
correspond roughly to 280, 460, 650, 830 and 1020 Hz. The
bar height is dB with some unspecified offset and scale.
