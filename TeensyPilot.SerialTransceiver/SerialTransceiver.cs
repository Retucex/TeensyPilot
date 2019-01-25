using System;
using System.IO.Ports;
using NLog;

namespace TeensyPilot.SerialTransceiver
{
	public class SerialTransceiver
	{
		Logger logger = LogManager.GetCurrentClassLogger();
		SerialPort serialPort = new SerialPort();

		public SerialTransceiver(string portName, int baudRate, Parity parity, int portDataBits, StopBits portStopBits, Handshake portHandshake, int portReadTimeout, int portWriteTimeout)
		{
			serialPort.PortName = portName;
			serialPort.BaudRate = baudRate;
			serialPort.Parity = parity;
			serialPort.DataBits = portDataBits;
			serialPort.StopBits = portStopBits;
			serialPort.Handshake = portHandshake;
			serialPort.ReadTimeout = portReadTimeout;
			serialPort.WriteTimeout = portWriteTimeout;

			logger.Trace("New instance of SerialTransceiver; portName:{0}; baudRate:{1}; parity:{2}; portDataBits{3}; portStopBits{4}; portHandshake:{5}; portReadTimeout:{6}; portWriteTimeout:{7}",
				portName,
				baudRate,
				parity,
				portDataBits,
				portStopBits,
				portHandshake,
				portReadTimeout,
				portWriteTimeout);
		}

		public string ReadLine()
		{
			string s;
			serialPort.Open();
			s = serialPort.ReadLine();
			serialPort.Close();
			return s;
		}
	}
}
