# DHBW IoT Temperature-Sensor (Arduino MKR WiFi 1010)

## About

This project reads the temperature values from an ADT7410 sensor and pushes it
to a MQTT broker.

## Usage

1. Clone this repository
2. Go to /src directory
3. Rename secrets_template.h to secrets.h
4. Add your WiFi SSID and password to secrets.h
5. Open /src/main.ino and add your MQTT Broker information
6. Install plugins with [PlatformIO](https://platformio.org)
7. Build and flash project to Arduino with PlatformIO

## License

Distributed under the MIT License. See LICENSE for more information.

## Contact

Kai Anter - [@tanikai29](https://twitter.com/tanikai29)
