using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;
using Windows.System.Threading;

namespace UwpRcTransmitter
{
    public enum Protocol
    {
        Intertechno,
        REV,
        FLS,
        DIP,
        HomeEasy,
    }

    public class RcTransmitter
    {
        /// <summary>
        /// possible state for bit representations
        /// Die Pulsfolge 
        /// <kurz> <kurz> wird mit "0" codiert.
        /// <kurz> <lang> wir zu "F" und
        /// <lang> <lang> wird zu "1". 
        /// </summary>
        private byte lowBitType = 0x88;                     // 0
        private byte floatBitType = 0x8e;                   // F
        private byte highBitType = 0xee;                    // 1
        private byte syncBitType = 0x80;                    // S
        private byte spaceBitType = 0x00;                   //
        private byte latchBitType = 0x08;                   // L
        private byte easy1BitType = 0x82;                   // EasyHome 1
        private byte easy0BitType = 0xa0;                   // EasyHome 0

        private static readonly int defaultPin = 0;         // use SPI
        private static readonly Protocol defaultProtocol = Protocol.Intertechno;
        private const int defaultRepeatTransmit = 5;

        private GpioController gpioController;
        private GpioPin gpioPin;
        private Protocol protocol;
        private int pulseLength;
        private int spiFrequency;
        private int repeatTransmit;
        private static SpiDevice spiDevice;
        private Stopwatch stopwatch;

        private const string SPI_CONTROLLER_NAME = "SPI0";  // For Raspberry Pi 2, use SPI0
        private const int SPI_CHIP_SELECT_LINE = 0;         // Line 0 maps to physical pin number 24 on the Rpi2

        public static long DefaultEasyHomeController = 0x262a1;

        public RcTransmitter()
            : this(defaultPin, defaultProtocol)
        {
        }

        public RcTransmitter(int pin)
            : this(pin, defaultProtocol)
        {
        }

        public RcTransmitter(Protocol protocol)
            : this(defaultPin, protocol)
        {
        }

        public RcTransmitter(int pin, Protocol protocol, int repeat = defaultRepeatTransmit)
        {
            this.protocol = protocol;
            this.repeatTransmit = repeat;
            switch (protocol)
            {
                case Protocol.REV:
                    pulseLength = 360;
                    spiFrequency = 10000000;
                    break;
                case Protocol.HomeEasy:
                    pulseLength = 250;
                    spiFrequency = 10000000;
                    break;
                case Protocol.Intertechno:
                default:
                    pulseLength = 350;
                    spiFrequency = 10000000;
                    break;
            }

            if (pin == 0 && spiDevice == null)
            {
                spiDevice = InitSpi().Result;
            }
            else
            {
                gpioController = GpioController.GetDefault();
                gpioPin = gpioController.OpenPin(pin, GpioSharingMode.Exclusive);
                gpioPin.Write(GpioPinValue.Low);
                gpioPin.SetDriveMode(GpioPinDriveMode.Output);
            }
        }

        /// <summary>
        /// Sets an EasyHome device to given state
        /// </summary>
        /// <param name="controller">Number of the controller</param>
        /// <param name="device">Number of the deviice</param>
        /// <param name="group">Indicates if whole group is affected</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        public void Switch(long controller, int device, bool group, bool state)
        {
            if (protocol != Protocol.HomeEasy)
            {
                throw new NotSupportedException("Not supported for this type of protocol");
            }
            Send(GetRcCode(controller, device, group, state));
        }

        /// <summary>
        /// Sets an Intertechno device to given state
        /// </summary>
        /// <param name="family">family code 'a' - 'p'</param>
        /// <param name="group">group code 1 - 4</param>
        /// <param name="device">device code 1 - 4</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        public void Switch(char family, int group, int device, bool state)
        {
            if (protocol != Protocol.Intertechno)
            {
                throw new NotSupportedException("Not supported for this type of protocol");
            }

            Send(GetRcCode(family, group, device, state));
        }

        /// <summary>
        /// Sets an REV type device to given state
        /// </summary>
        /// <param name="group">group code 'A', 'B', 'C' or 'D'</param>
        /// <param name="device">device code 1 - 3</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        public void Switch(char group, int device, bool state)
        {
            if (protocol != Protocol.REV)
            {
                throw new NotSupportedException("Not supported for this type of protocol");
            }

            Send(GetRcCode(group, device, state));
        }

        /// <summary>
        /// Sets an REV type device to given state
        /// </summary>
        /// <param name="group">group code 1 - 32</param>
        /// <param name="device">device code 1|2|3|4|5</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        public void Switch(Device device, int group, bool state)
        {
            if (protocol != Protocol.DIP)
            {
                throw new NotSupportedException("Not supported for this type of protocol");
            }

            Send(GetRcCode(group, device, state));
        }

        /// <summary>
        /// Sets an FLS type device to given state
        /// </summary>
        /// <param name="group">group code 1 - 4</param>
        /// <param name="device">device code 1 - 4</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        public void Switch(int group, int device, bool state)
        {
            if (protocol != Protocol.FLS)
            {
                throw new NotSupportedException("Not supported for this type of protocol");
            }

            Send(GetRcCode(group, device, state));
        }

        /// <summary>
        /// Decoding for the Intertechno Switch Type
        ///
        /// Returns a char[13], representing the Tristate to be send.
        /// A Code Word consists of 8 address bits, two unused bits and 2 data bits.
        /// A Code Bit can have 4 different states: "F" (floating), "0" (low), "1" (high), "S" (sync)
        ///
        /// +--------------------------------+--------------------------------+-------------------------------+-----------------------+------------+
        /// | 4 bits address (switch family) | 2 bits address (device number) | 2 bits address (group number) | 4 bits (command data) | 1 sync bit |
        /// | A=0000 B=F000 C=0F00 D=FF00    | 1=00 2=F0 3=0F 4=FF            | 1=00 2=F0 3=0F 4=FF           | on=00FF off=00F0      | S          |
        /// | E=00F0 F=F0F0 G=0FF0 H=FFF0    |                                |                               |                       |            |
        /// | I=000F J=F00F K=0F0F L=FF0F    |                                |                               |                       |            |
        /// | M=00FF N=F0FF O=0FFF P=FFFF    |                                |                               |                       |            |
        /// +--------------------------------+--------------------------------+-------------------------------+-----------------------+------------+
        ///
        /// Source: http://www.fhemwiki.de/wiki/Intertechno_Code_Berechnung
        /// </summary>
        /// <param name="family">family code 'a' - 'p'</param>
        /// <param name="device">device code 1 - 4</param>
        /// <param name="group">group code 1 - 4</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        /// <returns>the Intertechno code</returns>
        private byte[] GetRcCode(char family, int device, int group, bool state)
        {
            byte familyCode = (byte)(char.ToLower(family) - 'a');
            if (familyCode > 0x0f) return new byte[0];
            if (device < 1 || device > 4) return new byte[0];
            if (group < 1 || group > 4) return new byte[0];

            int pos = 0;
            byte[] code = new byte[16];
            foreach (byte b in Decode2Bin(familyCode, 4, floatBitType, lowBitType, lowBitType)) code[pos++] = b;
            foreach (byte b in Decode2Bin(device - 1, 2, floatBitType, lowBitType, lowBitType)) code[pos++] = b;
            foreach (byte b in Decode2Bin(group - 1, 2, floatBitType, lowBitType, lowBitType)) code[pos++] = b;

            code[pos++] = lowBitType;
            code[pos++] = floatBitType;

            code[pos++] = floatBitType;
            code[pos++] = state ? floatBitType : lowBitType;
            code[pos++] = syncBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;

            return code;
        }

        /// <summary>
        /// Decoding for the REV Switch Type
        ///
        /// Returns a char[13], representing the Tristate to be send.
        /// A Code Word consists of 7 address bits and 5 command data bits.
        /// A Code Bit can have 4 different states: "F" (floating), "0" (low), "1" (high), "S" (sync)
        ///
        /// +-------------------------------+--------------------------------+-----------------+-----------------------+------------+
        /// | 4 bits address (switch group) | 3 bits address (device number) | 5 bits (static) | 2 bits (command data) | 1 sync bit |
        /// | A=1FFF B=F1FF C=FF1F D=FFF1   | 1=1FF 2=F1F 3=FF1              | 0FF             | on=FF off=00          | S          |
        /// +-------------------------------+--------------------------------+-----------------+-----------------------+------------+
        ///
        /// Source: http://www.the-intruder.net/funksteckdosen-von-rev-uber-arduino-ansteuern/
        /// or: http://www.fhemwiki.de/wiki/Intertechno_Code_Berechnung
        /// </summary>
        /// <param name="group">group code 'a' - 'd'</param>
        /// <param name="device">device code 1 - 3</param>
        /// <param name="state"></param>
        /// <returns>the REV code to send</returns>
        private byte[] GetRcCode(char group, int device, bool state)
        {
            byte groupCode = (byte)(char.ToLower(group) - 'a');
            if (groupCode > 0x03) return new byte[0];
            if (device < 1 || device > 3) return new byte[0];

            int pos = 0;
            byte[] code = new byte[16];
            foreach (byte b in Decode2Bin(1 << groupCode, 4, floatBitType, highBitType, highBitType)) code[pos++] = b;
            foreach (byte b in Decode2Bin(1 << (device - 1), 3, floatBitType, highBitType, highBitType)) code[pos++] = b;
            code[pos++] = lowBitType;
            code[pos++] = lowBitType;
            code[pos++] = lowBitType;
            code[pos++] = state ? floatBitType : lowBitType;
            code[pos++] = state ? lowBitType : floatBitType;
            code[pos++] = syncBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;

            return code;
        }

        /// <summary>
        /// Decoding for the FLS Switch Type
        ///
        /// Returns a char[13], representing the Tristate to be send.
        /// A Code Word consists of 7 address bits and 5 command data bits.
        /// A Code Bit can have 4 different states: "F" (floating), "0" (low), "1" (high), "S" (sync)
        ///
        /// +-------------------------------+--------------------------------+----------------------+------------+
        /// | 4 bits address (switch group) | 4 bits address (device number) | 1 bit (command data) | 1 sync bit |
        /// | A=0FFF B=F0FF C=FF0F D=FFF0   | 1=0FFF 2=F0FF 3=FF0F 4=FFF0    | on=F off=0           | S          |
        /// +-------------------------------+--------------------------------+----------------------+------------+
        ///
        /// or: http://www.fhemwiki.de/wiki/Intertechno_Code_Berechnung
        /// </summary>
        /// <param name="group"></param>
        /// <param name="device"></param>
        /// <param name="state"></param>
        /// <returns></returns>
        private byte[] GetRcCode(int group, int device, bool state)
        {
            if (group < 1 || group > 4) return new byte[0];
            if (device < 1 || device > 4) return new byte[0];

            int pos = 0;
            byte[] code = new byte[13];
            foreach (byte b in Decode2Bin(1 << (group - 1), 4, lowBitType, floatBitType, floatBitType)) code[pos++] = b;
            foreach (byte b in Decode2Bin(1 << (device - 1), 4, lowBitType, floatBitType, floatBitType)) code[pos++] = b;
            code[pos++] = state ? floatBitType : lowBitType;
            code[pos++] = syncBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;

            return code;
        }

        [Flags]
        public enum Device
        {
            Device1 = 0x01,
            Device2 = 0x02,
            Device3 = 0x03,
            Device4 = 0x08,
            Device5 = 0x10,
        }

        /// <summary>
        /// Decoding for the Intertechno Switch Type
        ///
        /// Returns a byte[16], representing the Tristate to be send.
        /// A Code Word consists of 10 address bits and 2 data bits.
        /// A Code Bit can have 4 different states: "F" (floating), "0" (low), "1" (high), "S" (sync)
        ///
        /// +-------------------------------+--------------------------------+-----------------------+------------+
        /// | 5 bits address (switch group) | 5 bits address (device number) | 2 bits (command data) | 1 sync bit |
        /// |  1=00000                      | 1 = F0000                      | on=0F off=F0          | S          |
        /// |  2=F0000                      | 2 = 0F000                      |                       |            |
        /// |     ...                       | 3 = 00F00                      |                       |            |
        /// | 31=0FFFF                      | 4 = 000F0                      |                       |            |
        /// | 32=FFFFF                      | 5 = 0000F                      |                       |            |
        /// +-------------------------------+--------------------------------+-----------------------+------------+
        ///
        /// Source: http://www.fhemwiki.de/wiki/Intertechno_Code_Berechnung
        /// </summary>
        /// <param name="device">device code 1 - 4</param>
        /// <param name="group">group code 1 - 4</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        /// <returns>the Intertechno code</returns>
        private byte[] GetRcCode(int group, Device device, bool state)
        {
            if (group < 1 || group > 32) return new byte[0];

            int pos = 0;
            byte[] code = new byte[16];
            foreach (byte b in Decode2Bin(1 << (group - 1), 5, lowBitType, floatBitType, floatBitType)) code[pos++] = b;
            foreach (byte b in Decode2Bin((byte)device, 5, lowBitType, floatBitType, floatBitType)) code[pos++] = b;
            code[pos++] = state ? lowBitType : floatBitType;
            code[pos++] = state ? floatBitType : lowBitType;
            code[pos++] = syncBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;
            code[pos++] = spaceBitType;

            return code;
        }

        /// <summary>
        /// Decoding for the EasyHome switch
        /// 
        /// Return byte[46] code
        ///
        /// +--------+--------+-------------------+----------------+--------------+----------------+--------+
        /// | Latch  | Latch  | 26 bit Controller | 1 bit (group)  | 1 bit (data) | 4 bit (device) | Latch  |
        /// | 9750µs | 2750µs | 0 = 0000000       | individual = 0 | on = 1       | 0 = 0000       | 3750µs |
        /// |        |        | 1 = 8000000       | group = 1      | off = 0      | 1 = 1000       |        |
        /// |        |        |     ...           |                |              |    ...         |        |
        /// |        |        | x = FFFFFF1       |                |              | 1 = 1110       |        |
        /// |        |        | y = FFFFFF3       |                |              | 1 = 1111       |        |
        /// +--------+--------+-------------------+----------------+--------------+----------------+--------+
        ///
        /// Source:
        /// http://homeeasyhacking.wikia.com/wiki/Advanced_Protocol
        /// http://www.sweetpi.de/blog/329/ein-ueberblick-ueber-433mhz-funksteckdosen-und-deren-protokolle
        /// </summary>
        /// <param name="controller">Number of the controller</param>
        /// <param name="device">Number of the deviice</param>
        /// <param name="group">Indicates if whole group is affected</param>
        /// <param name="state">state, true = ON, false = OFF</param>
        /// <returns>the EasyHome code</returns>
        private byte[] GetRcCode(long controller, int device, bool group, bool state)
        {
            int pos = 0;
            byte[] code = new byte[46];
            foreach (byte b in new byte[] { latchBitType, spaceBitType, spaceBitType, spaceBitType, spaceBitType }) code[pos++] = b;     // Latch 9750
            foreach (byte b in new byte[] { latchBitType, spaceBitType }) code[pos++] = b;                                               // Latch 2750
            foreach (byte b in Decode2Bin(controller, 26, easy1BitType, easy0BitType, easy0BitType)) code[pos++] = b;
            code[pos++] = group ? easy1BitType : easy0BitType;
            code[pos++] = state ? easy1BitType : easy0BitType;
            foreach (byte b in Decode2Bin(device, 4, easy1BitType, easy0BitType, easy0BitType)) code[pos++] = b;                         // device
            foreach (byte b in new byte[] { syncBitType, spaceBitType }) code[pos++] = b;                                                // Latch 3750
            foreach (byte b in new byte[] { spaceBitType, spaceBitType, spaceBitType, spaceBitType, spaceBitType }) code[pos++] = b;     // Latch 10000                                          // Latch 3750

            return code;
        }

        private async Task<SpiDevice> InitSpi()
        {
            try
            {
                SpiConnectionSettings settings = new SpiConnectionSettings(SPI_CHIP_SELECT_LINE);
                //settings.ClockFrequency = 32000;                                        // Datasheet specifies maximum SPI clock frequency of 10MHz
                settings.ClockFrequency = spiFrequency;                                        // Datasheet specifies maximum SPI clock frequency of 10MHz
                settings.Mode = SpiMode.Mode3;                                          // ??
                settings.SharingMode = SpiSharingMode.Exclusive;

                string spiAqs = SpiDevice.GetDeviceSelector(SPI_CONTROLLER_NAME);       // Find the selector string for the SPI bus controller
                var devicesInfo = await DeviceInformation.FindAllAsync(spiAqs);         // Find the SPI bus controller device with our selector string
                return await SpiDevice.FromIdAsync(devicesInfo[0].Id, settings);        // Create an SpiDevice with our bus controller and SPI settings
            }
            catch (Exception ex)
            {
                throw new Exception("SPI initialization falied", ex);
            }
        }

        private byte[] Decode2Bin(long code, int length, byte high, byte low, byte fill)
        {
            byte[] bytes = new byte[length];
            for (int i = 0; i < length; i++)
            {
                bytes[i] = code == 0x00 ? fill : (code & 0x01) != 0 ? high : low;
                code >>= 1;
            }
            return bytes;
        }

        private async void Send(byte[] code)
        {
#if DEBUG
            DebugPulses(code);
#endif
            if (gpioPin != null)
            {
                await ThreadPool.RunAsync(handler =>
                {
                    stopwatch = Stopwatch.StartNew();
                    for (int repeat = 0; repeat < repeatTransmit; repeat++)
                    {
                        ticks = 0;
                        foreach (byte b in code)
                        {
                            for (int i = 7; i >= 0; i--)
                            {
                                gpioPin.Write((b & 0x01 << i) != 0x00 ? GpioPinValue.High : GpioPinValue.Low);
                                Delay(pulseLength);
                            }
                        }
                    }
                }, WorkItemPriority.High, WorkItemOptions.None);
        }
            else
            {
                byte[] zoomedCode = ZoomBits(code, (long)spiFrequency * pulseLength / 1000000);
                for (int repeat = 0; repeat < repeatTransmit; repeat++)
                {
                    spiDevice.Write(zoomedCode);
                }
            }
        }

        private static byte[] ZoomBits(byte[] code, long zoomFactor)
        {
            byte[] zoomedBits = new byte[code.Length * zoomFactor];
            int index = 0;
            byte outMask = 0x80;
            foreach (byte b in code)
            {
                byte inMask = 0x80;
                while (inMask != 0x00)
                {
                    bool bit = (b & inMask) != 0x00;
                    for (int i = 0; i < zoomFactor; i++)
                    {
                        zoomedBits[index] |= bit ? outMask : (byte)0x00;
                        outMask >>= 1;
                        if (outMask == 0x00)
                        {
                            outMask = 0x80;
                            index++;
                        }
                    }
                    inMask >>= 1;
                }
            }

            return zoomedBits;
        }

        private long ticks;

        private void Delay(long microSeconds)
        {
            if (ticks == 0) ticks = stopwatch.ElapsedTicks;
            long finalTicks = ticks + microSeconds * Stopwatch.Frequency / 1000000;
            do
            {
                ticks = stopwatch.ElapsedTicks;
            }
            while (stopwatch.ElapsedTicks < finalTicks);
        }

#if DEBUG
        private void DebugPulses(byte[] bytes)
        {
            StringBuilder debugMessage1 = new StringBuilder();
            StringBuilder debugMessage2 = new StringBuilder();
            int bitState = 0x00;
            int bitCount = 0;
            foreach (byte b in bytes)
            {
                for (int i = 7; i >= 0; i--)
                {
                    bitState |= (b & 0x01 << i) != 0x00 ? 0x01 : 0x00;
                    switch (bitState)
                    {
                        case 0x00:
                            debugMessage1.Append(' ');
                            debugMessage2.Append('_');
                            break;
                        case 0x01:
                            debugMessage1.Append(' ');
                            debugMessage2.Append('|');
                            debugMessage1.Append('_');
                            debugMessage2.Append(' ');
                            break;
                        case 0x02:
                            debugMessage1.Append(' ');
                            debugMessage2.Append('|');
                            debugMessage1.Append(' ');
                            debugMessage2.Append('_');
                            break;
                        case 0x03:
                            debugMessage1.Append('_');
                            debugMessage2.Append(' ');
                            break;
                    }
                    bitState |= (b & 0x01 << i) != 0x00 ? 0x02 : 0x00;
                    bitState &= (b & 0x01 << i) != 0x00 ? 0x02 : 0x00;

                    if (++bitCount == 80)
                    {
                        bitCount = 0;
                        Debug.WriteLine(debugMessage1.ToString());
                        Debug.WriteLine(debugMessage2.ToString());
                        debugMessage1.Clear();
                        debugMessage2.Clear();
                    }
                }
            }

            Debug.WriteLine(debugMessage1.ToString());
            Debug.WriteLine(debugMessage2.ToString());
        }
#endif
    }
}
