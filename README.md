# Robotont mbed mainboard firmware

Simplistic firmware for the Robotex platform mainboard.

## Serial protocol

The virtual serial port on the mbed USB bus is used for bidirectional communication. Approximately 100 times a second, motor speeds are given (from -100 to 100 in arbitrary units, sorry...). The format is: "motor1:motor2:motor3\n".

In the same format You can also send data to the mainboard to set the motor speeds (from -100 to 100).

## Development

Repository contains PlatformIO project. PlatformIO is a development ecosystem for embedded systems and eases working on the code a lot.

To work on this code just clone the repo and open it in the PlatformIO IDE (or build and upload it with the PlatformIO core tools from the command line). All the dependencies and build tools are installed automatically on the first build.

### CLI

1. Install PlatformIO [CLI](https://docs.platformio.org/en/latest/core/installation.html).    
2. Make sure to add [the standard udev rules](https://docs.platformio.org/en/latest/faq.html#faq-udev-rules) of platformIO and also ST-link 2.1 specific ones:    
```curl -fsSL https://raw.githubusercontent.com/stlink-org/stlink/develop/config/udev/rules.d/49-stlinkv2-1.rules | sudo tee /etc/udev/rules.d/49-stlinkv2-1.rules```

To build and upload, in project root, do
`pio run -t upload`
