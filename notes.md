Suggested workflow for staying in sync:
git remote add upstream https://github.com/lyusupov/SoftRF.git
git checkout -b vario-nrf52     # do your work on a branch
git fetch upstream && git rebase upstream/master

Build succeeded — nRF52840
- Toolchain fully isolated inside ./.arduino-data/ (gitignored)
- Adafruit nRF52 BSP 1.3.0 (per official docs), plus adafruit-nrfutil via pipx
- Vendored software/firmware/source/libraries/ symlinked as arduino-cli's libraries dir
- Sketch: software/firmware/source/SoftRF/SoftRF.ino
- FQBN: adafruit:nrf52:pca10056 (Nordic nRF52840DK — the target the source declares)

Artifacts in .arduino-data/build/:
- SoftRF.ino.hex — 2.0 MB — for SWD/J-Link
- SoftRF.ino.zip — 713 KB — for OTA (adafruit-nrfutil or Bluefruit Connect)
- SoftRF.ino.elf — for symbolic debugging
- Flash usage: 87% (712 KB / 815 KB) — noteworthy: not much headroom for your vario additions
- RAM usage: 22%

To rebuild anytime
arduino-cli --config-file .arduino-data/arduino-cli.yaml compile \
  --fqbn adafruit:nrf52:pca10056 \
  --build-path "$PWD/.arduino-data/build" \
  software/firmware/source/SoftRF

# Notes for your vario work
- Flash headroom is tight. With 87% used, adding features may require disabling optional features via SoftRF.h #defines (e.g., EXCLUDE_UAT978, EXCLUDE_LK8EX1, MAVLink, etc.).
- The nRF52840 code lives in software/firmware/source/SoftRF/src/platform/nRF52.cpp. Baro/vario driver is at software/firmware/source/SoftRF/src/driver/Baro.cpp.
- To flash later: DFU over USB with adafruit-nrfutil dfu serial --package .arduino-data/build/SoftRF.ino.zip -p /dev/ttyACM0 -b 115200 (adjust port).
