using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Threading;
using System.Windows.Forms.DataVisualization.Charting;

namespace KRPC_autopilot
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
            UserInterface ui = new UserInterface();
            ui.Enable();
            //Connection conn = new Connection("Main");
            //Vessel vessel = conn.SpaceCenter().ActiveVessel;

            Functions.Landing.Land();
            Environment.Exit(0);
        }
    }
}
