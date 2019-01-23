using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;

namespace TeensyPilot.SerialTransceiver
{
	public class SerialTransceiverBuilder
	{
		string portName;
		int baudRate;
		Parity parity;
		int portDataBits;
		StopBits portStopBits;
		Handshake portHandshake;

		public SerialTransceiverBuilder()
		{
			var serialPort = new SerialPort();
			portName = serialPort.PortName;
			baudRate = serialPort.BaudRate;
			parity = serialPort.Parity;
			portDataBits = serialPort.DataBits;
			portStopBits = serialPort.StopBits;
			portHandshake = serialPort.Handshake;
		}

		public SerialTransceiverBuilder PortName(string portName)
		{
			this.portName = portName;
			return this;
		}

		public SerialTransceiverBuilder BaudRate(int baudRate)
		{
			this.baudRate = baudRate;
			return this;
		}

		public SerialTransceiverBuilder Parity (Parity parity)
		{
			this.parity = parity;
			return this;
		}

		public SerialTransceiverBuilder PortDataBits(int portDataBits)
		{
			this.portDataBits = portDataBits;
			return this;
		}

		public SerialTransceiverBuilder PortStopBits(StopBits portStopBits)
		{
			this.portStopBits = portStopBits;
			return this;
		}

		public SerialTransceiverBuilder PortHandshake(Handshake portHandshake)
		{
			this.portHandshake = portHandshake;
			return this;
		}

		public static implicit operator SerialTransceiver(SerialTransceiverBuilder b)
		{
			return new SerialTransceiver(
				b.portName,
				b.baudRate,
				b.parity,
				b.portDataBits,
				b.portStopBits,
				b.portHandshake);
		}
	}
}
