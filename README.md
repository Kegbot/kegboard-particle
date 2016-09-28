# kegboard-particle

Flow sensor controller firmware for [Particle](https://www.particle.io/)
[Photon](https://www.particle.io/products/hardware/photon-wifi-dev-kit) and
[Electron](https://www.particle.io/products/hardware/electron-cellular-dev-kit)
hardware. Provides meter readings over serial port and Wifi (Photon) or cellular
(Electron) network connections.

## Build and Flash

To build and flash your particle device, load the `main.ino` file into the [Particle IDE](https://build.particle.io/build) and flash it to your device. That's it!

## Wiring

The following pins are configured (all optional):

* `D1` - Flow meter `meter0`
* `D2` - Flow meter `meter1`
* `D3` - Flow meter `meter2`
* `D4` - Flow meter `meter3`
* `D5` - OneWire DS1820 temperature sensor.
* `A2` - RFID Reader (MFRC522)
* `D7` - RFID Reset 

Most flow meters, along with DS1820 temperature sensors, require a [pull-up resistor](https://learn.sparkfun.com/tutorials/pull-up-resistors) for proper operation. Consult hardware documentation.

## Reading Data

There are three ways to read meter data from the device:

* Serial interface: Connect directly to the serial port. This interface is always running.
* TCP interface: Connect to the device on a TCP port.
* Cloud interface: Publishes a message to the Particle cloud using [Particle.publish()](https://community.particle.io/t/tutorial-getting-started-with-spark-publish/3422). This interface is active when no TCP clients are connected.

### TCP Interface

The firmware runs a TCP server on port `8321` where meter readings and
RFID present and RFID removed events are published. A single client at 
a time can connect to this port.

Example:

```
$ telnet 192.168.1.7 8321
info: kegboard-particle device_id=1234abcd1234abcd00001111 version=0.1.0
kb-status: rfidp.token=F1E2D3C4 rfidp.status=present rfidp.ts=2016-09-26T13:15:30Z
kb-status: meter0.ticks=0 meter1.ticks=0 meter2.ticks=0 meter3.ticks=0
kb-status: meter0.ticks=16 meter1.ticks=0 meter2.ticks=0 meter3.ticks=0
kb-status: rfidp.token=F1E2D3C4 rfidp.status=removed rfidp.ts=2016-09-26T13:16:15Z
```

### Serial Interface

Meter updates and RFID present and RFID removed eventsare also published on 
the serial port, in the same message format as the TCP server.
Currently, temperature reports are only published on the serial port, per 
the following example:

...
THERMO id=1a2b3c4d5e6f7081  temp_c=12.34
...

### Cloud Interface

If there is no TCP client connected, the device will instead report status, up to once a second, to the Particle Cloud.

Log in to the [Particle logs console](https://console.particle.io/logs) to see events.


## License

Offered free and open source under the MIT license. See `LICENSE.txt`.
