using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Threading;
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
            //Functions.Landing.Land();
            //Functions.Launch.Equatorial();
            while (true)
            {
                Vector3 surfaceNormal = Radar.SurfaceNormal();
                //Thread.Sleep(100);
            }
            Environment.Exit(0);
        }
    }
}
