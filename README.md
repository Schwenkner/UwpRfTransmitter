# UwpRfTransmitter
Controls Rf transmitters on Windows 10 IoT 

This is a library for controlling RF-433 switches with Windows 10 IoT.

The library works with either using GPIO or SPI. Since the signal to send has periods of 250 microseconds, this is the most problem to produce such periodic pulses in .Net since it is managed code and you do not have timers for such frequency. In GPIO mode a loop with the Stopwatch timer is used. This works but is some cases the loop may be interrupted by the OS and then the signal is not recognized. By default the signal is send 3 times, if you have problems in case of other threads running then try to increase the count. In SPI mode (GPIO port 0), the signal is produced by hardware. This is much more secure.
