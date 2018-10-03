using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Windows.Forms.DataVisualization.Charting;

namespace KRPC_test
{
	partial class Program
	{

		/// <summary>
		/// 
		/// </summary>
		/// <param name="args"></param>
		/// 
		static void Main(string[] args)
		{
			Connection conn = new Connection("Main");
			//Vessel vessel = conn.SpaceCenter().ActiveVessel;
			Functions.Landing.Land();
			//while (true)
			//{
			//}
			Environment.Exit(0);
		}
	}
}
