# Flysky IBUS Protocol Parser Library
This library provides support for parsing serial messages containing telemetry data generated by Flysky devices. 

This library currently supports the following platforms:
* Linux i2c subsystem
* esp-idf

# Usage

## API

The API for this library can be found in the `include/ibus/ibus.h` header file.

## Portability

Portability among various host platforms (e.g. Linux UART driver interface vs. the esp-idf UART driver interface) is accomplished via a platform-specific `ibus_lowlevel_config` structure which is defined at compile-time for the project based on build environment and/or toolchain hints. An example configuration for `uart_lowlevel_config` for Linux is:

```bash
uart_lowlevel_config config;
config.device = "/dev/ttyS0";
```

An example configuration for `uart_lowlevel_config` for esp-idf is:

```bash
uart_lowlevel_config config;
config.port = UART_NUM_1; 
config.tx_pin = 4;
config.rx_pin = 5;
```

Note that the members of the `uart_lowlevel_config` change (at compile-time) based on the target platform.

# Example Applications

Example applications are provided for each of the supported platforms.

# License
All files delivered with this library are copyright 2024 Zorxx Software and released under the MIT license. See the `LICENSE` file for details.
