using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using TeensyPilot.SerialTransceiver;

namespace TeensyPilot.CommandCenter
{
	/// <summary>
	/// Interaction logic for MainWindow.xaml
	/// </summary>
	public partial class MainWindow : Window
	{
		private SerialTransceiver.SerialTransceiver serialTransceiver;

		public MainWindow()
		{
			InitializeComponent();

			serialTransceiver = new SerialTransceiverBuilder()
				.PortName("COM6")
				.BaudRate(115200);

			var t = (TextBox)FindName("DebugTextBox");
			t.Text = serialTransceiver.ReadLine();
		}
	}
}