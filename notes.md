# SoftRF-Vario-Upgrade — Agent Conventions

## Build (nRF52 T-Echo)

Always use this exact command from repo root:

```bash
arduino-cli --config-file .arduino-data/arduino-cli.yaml compile \
  --fqbn adafruit:nrf52:pca10056 \
  --build-path .arduino-data/build \
  software/firmware/source/SoftRF
```

Output: `.arduino-data/build/SoftRF.ino.zip` (DFU package for flashing)

**Never** use `--libraries` flag — the config file's user directory has a symlink to the project libraries.

## Flash (DFU over USB)

```bash
newgrp uucp << 'EOF'
/home/ax/.local/bin/adafruit-nrfutil dfu serial \
  -pkg .arduino-data/build/SoftRF.ino.zip \
  -p /dev/ttyACM0 -b 115200 -t 1200
EOF
```

`newgrp uucp` is needed because `/dev/ttyACM0` is owned by group `uucp` (shell needs group refresh).

## Monitor Serial Output

```bash
arduino-cli monitor -p /dev/ttyACM0 -b adafruit:nrf52:feather52840
```

## Cache Issues

If compilation shows weird errors after code changes:
```bash
rm -rf /home/ax/.cache/arduino/sketches
```

## Code Layout

- Drivers live in `software/firmware/source/SoftRF/src/driver/*.cpp` (auto-compiled)
- Vendored libraries in `software/firmware/source/libraries/` (auto-discovered via symlink)
- **Every new `.cpp` must have a matching `.h` included by all callers** — forward declarations break C/C++ linkage matching (see PiezoBeeper.h `extern "C"` pattern)
