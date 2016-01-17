# UwpRfTransmitter
Controls Rf transmitters on Windows 10 IoT 

This is a library for controlling RF-433 switches with Windows 10 IoT.

I do not know, if the library works with all kind of switches, I have tested it only with HomeEasy switches since they are here very common in Europe. The code for other manufacturers is included and taken from a C++ library. If someone can test this with other switches and gives me some feedback it would be great.

The library works with either using GPIO or SPI. Since the signal to send has periods of 250 microseconds, this is the most problem to produce such periodic pulses in .Net since it is managed code and you do not have timers for such frequency. In GPIO mode a loop with the Stopwatch timer is used. This works but is some cases the loop may be interrupted by the OS and then the signal is not recognized. By default the signal is send 3 times, if you have problems in case of other threads running then try to increase the count. In SPI mode (GPIO port 0), the signal is produced by hardware. This is much more secure.

There are two kind of switches from InterTechno. One kind is able to define the device code by jumpers and one kind is self learning. The self learning ones using the HomeEasy coding. For the self learning switches use the HomeEasy code instead of InterTechno code.
