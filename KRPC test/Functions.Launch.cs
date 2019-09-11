using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;

namespace KRPC_autopilot
{
	public static partial class Functions
    {
		public static class Launch
        {
            public static void Equatorial()
            {

                Connection conn = new Connection("Launch into orbit");
                var vessel = conn.SpaceCenter().ActiveVessel;

                // Set up streams for telemetry
                Stream<double> ut = conn.AddStream(() => conn.SpaceCenter().UT); //Universal time
                Flight flight = vessel.Flight();
                Stream<double> altitude = (conn.AddStream(() => flight.MeanAltitude));
                var velocityStream = conn.AddStream(() => vessel.Velocity(vessel.Orbit.Body.ReferenceFrame));
                Stream<double> apoapsis = conn.AddStream(() => vessel.Orbit.ApoapsisAltitude);
                Stream<int> currentStage = conn.AddStream(() => vessel.Control.CurrentStage);
                Stream<float> availableThrust = conn.AddStream(() => vessel.AvailableThrust);
                Stream<float> maxThrust = conn.AddStream(() => vessel.MaxThrust); // Maxthrust can be read only once per staging
                var nextStageResources = vessel.ResourcesInDecoupleStage(stage: currentStage.Get() - 2, cumulative: false);

                double alpha = 90;
                vessel.Control.SAS = false;
                vessel.Control.RCS = false;
                vessel.AutoPilot.Engage();
                vessel.AutoPilot.ReferenceFrame = vessel.SurfaceReferenceFrame;
                int flightMode = 0; //Prelaunch
                while (flightMode < 2)
                {
                    //If the ship is on the launchpad, start engines
                    if (flightMode == 0)
                    {
                        vessel.Control.Throttle = 1;
                        vessel.Control.ActivateNextStage();
                        flightMode = 1;
                    }
                    //Control twr

                    //Turn
                    if (flightMode == 1)
                    {
                        vessel.AutoPilot.TargetPitchAndHeading((float)alpha, 90);

                        if (altitude.Get() < 12000)
                        {
                            alpha = 45 + 45 * (1 - (altitude.Get() / (12000)));
                        }
                        else if (altitude.Get() < 65000 && altitude.Get() > 12000)
                        {
                            alpha = Math.Max(5, 45 * (1 - ((altitude.Get() - 12000) / (70000))));


                        }
                        else
                        {
                            flightMode = 2;
                        }
                        if (apoapsis.Get() > 75000)
                        {
                            vessel.Control.Throttle = 0;
                            //  WriteProgress("Throttle 2", 0, 10);

                            vessel.Control.RCS = true;
                            flightMode = 2;
                        }
                    }


                    //Staging
                    if (currentStage.Get() > 0)
                    {
                        Readings.Stage(vessel);
                    }
                }

                //orient to prograde
                vessel.AutoPilot.ReferenceFrame = vessel.OrbitalReferenceFrame;
                vessel.AutoPilot.TargetDirection = Tuple.Create(0.0, 1.0, 0.0);

                //Circularisation
                Readings.AddNode(vessel, ut);
                flightMode = 3;
                List<Node> nodes = new List<Node>(vessel.Control.Nodes);
                Node node = nodes[0];

                // Calculate burn time (using rocket equation)
                double burnTime = Readings.GetBurnTime(vessel, node);

                // Warp to burn
                Console.WriteLine("Waiting until circularization burn");
                double burnUT = ut.Get() + node.TimeTo - burnTime / 2;
                Readings.WarpToNode(conn, altitude, node, burnTime + 4, burnUT);

                //Orient to node prograde
                vessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
                vessel.AutoPilot.TargetDirection = Tuple.Create(0.0, 1.0, 0.0);


                //Wait for the perfect time to burn
                while (node.TimeTo > (burnTime + 1) / 2)
                {
                    //do nothing
                }
                vessel.Control.RCS = false;
                Readings.ExeNode(vessel);


                node.Remove();

                vessel.AutoPilot.Disengage();
                vessel.Control.Throttle = 0;
            }
            private static void Turn(Vessel vessel, double altitude)
            {

            }
        }

    }
}
