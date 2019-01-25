using NUnit.Framework;
using TeensyPilot.SerialTransceiver;
using System.IO.Ports;

namespace Tests
{
	public class SerialTransceiverBuilderTests
	{
		SerialTransceiverBuilder serialTransceiverBuilder;

		[SetUp]
		public void Setup()
		{
			serialTransceiverBuilder = new SerialTransceiverBuilder();
		}

		[Test]
		public void Ctor_EqualToDefaultSerialPort ()
		{
			var serialPort = new SerialPort();
			Assert.AreEqual(serialPort.PortName, serialTransceiverBuilder.PortName);
			Assert.AreEqual(serialPort.BaudRate, serialTransceiverBuilder.BaudRate);
			Assert.AreEqual(serialPort.Parity, serialTransceiverBuilder.Parity);
			Assert.AreEqual(serialPort.DataBits, serialTransceiverBuilder.DataBits);
			Assert.AreEqual(serialPort.StopBits, serialTransceiverBuilder.StopBits);
			Assert.AreEqual(serialPort.Handshake, serialTransceiverBuilder.Handshake);
			Assert.AreEqual(serialPort.ReadTimeout, serialTransceiverBuilder.ReadTimeout);
			Assert.AreEqual(serialPort.WriteTimeout, serialTransceiverBuilder.WriteTimeout);
		}
	}
}