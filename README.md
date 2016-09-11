# kegboard-particle

Flow sensor controller firmware for [Particle](https://www.particle.io/)
[Photon](https://www.particle.io/products/hardware/photon-wifi-dev-kit) and
[Electron](https://www.particle.io/products/hardware/electron-cellular-dev-kit)
hardware. Provides meter readings over serial port and Wifi (Photon) or cellular
(Electron) network connections.

## TCP Interface

The firmware runs a TCP server on port `8321` where meter readings are
published. A single client at a time can connect to this port.

Example:

```
$ telnet 192.168.1.7 8321
info: kegboard-particle device_id=1234abcd1234abcd00001111 version=0.1.0
kb-status: meter0.ticks=0 meter1.ticks=0 meter2.ticks=0 meter3.ticks=0
kb-status: meter0.ticks=16 meter1.ticks=0 meter2.ticks=0 meter3.ticks=0
```

## Serial Interface

Meter updates are also published on the serial port, in the same message format
as the TCP server.
