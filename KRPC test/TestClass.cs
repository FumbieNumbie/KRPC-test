using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace KRPC_autopilot
{
	 class TestClass
	 {
		  public static void Launch(int target)
		  {
				Connection conn = new Connection("Launch into orbit");

				var vessel = conn.SpaceCenter().ActiveVessel;

				// Set up streams for telemetry
				var ut = conn.AddStream(() => conn.SpaceCenter().UT);
				var flight = vessel.Flight();
				var altitude = (conn.AddStream(() => flight.MeanAltitude));
				var velocityStream = conn.AddStream(() => vessel.Velocity(vessel.Orbit.Body.ReferenceFrame));
				var apoapsis = conn.AddStream(() => vessel.Orbit.ApoapsisAltitude);
				var currentStage = conn.AddStream(() => vessel.Control.CurrentStage);
				var availableThrust = conn.AddStream(() => vessel.AvailableThrust);
				var maxThrust = conn.AddStream(() => vessel.MaxThrust);
				var currentStageResources = vessel.ResourcesInDecoupleStage(stage: currentStage.Get() - 1, cumulative: false); //
				var nextStageResources = vessel.ResourcesInDecoupleStage(stage: currentStage.Get() - 2, cumulative: false);




				//Console.Write("Enter desired apoapsis. ");
				target *= 1000;



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
						  if (apoapsis.Get() > target)
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
					 //



					 // WriteProgress("Speed: " + velocity.Length().ToString(), 0, 4);
					 //WriteProgress("Fuel in current stage: " + Compute.GetFuel(currentStageResources), 0, 5);
					 //WriteProgress("Current stage: " + currentStage.Get(), 0, 6);
					 // WriteProgress("Flight mode: "+flightMode, 0, 9);


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
				conn.Dispose();
		  }
		  public static void Land()
		  {
				Connection conn = new Connection("Land");

				var vessel = conn.SpaceCenter().ActiveVessel;
				// Set up streams for telemetry
				var ut = conn.AddStream(() => conn.SpaceCenter().UT);
				var flight = vessel.Flight();
				var altitude = (conn.AddStream(() => flight.MeanAltitude));
				var velocity = conn.AddStream(() => vessel.Flight(vessel.Orbit.Body.ReferenceFrame).Speed);

				var launchpad = vessel.Position(vessel.Orbit.Body.ReferenceFrame);
				while (true)
				{

					 //Console.Write(Math.Round(velocity.Get()) + "  " + Math.Round(Magnitude(vessel.Velocity(vessel.Orbit.Body.ReferenceFrame)))+"  " + "\r");
					 Console.Write(Math.Abs((TupleToVector(launchpad) - TupleToVector(vessel.Position(vessel.Orbit.Body.ReferenceFrame))).Length()) + "\r");

				}
		  }
		  /// <summary>
		  /// Convert tuple to vector
		  /// </summary>
		  /// <param name="tuple">Takes tuple of doubles as a parameter</param>
		  /// <returns>Returnes a Vector3</returns>
		  public static Vector3 TupleToVector(Tuple<double, double, double> tuple)
		  {
				return new Vector3((float)tuple.Item1, (float)tuple.Item2, (float)tuple.Item3);
		  }

		  public static Vector3 GetGroundVector(Tuple<double, double, double> tuple)
		  {
				return new Vector3(0, (float)tuple.Item2, (float)tuple.Item3);
		  }

		  /// <summary>
		  /// Your vessel's surface speed
		  /// </summary>
		  /// <param name="conn">Needs a connection and a vessel objects as parameters</param>
		  /// <param name="vessel"></param>
		  /// <returns>Returnes a tuple of doubles</returns>
		  public static Tuple<double, double, double> GetSurfaceSpeed(Connection conn, Vessel vessel)
		  {
				var refFrame = ReferenceFrame.CreateHybrid(conn,
																		 vessel.Orbit.Body.ReferenceFrame,
																		 vessel.SurfaceReferenceFrame);
				return vessel.Flight(refFrame).Velocity;
		  }


		  public static Tuple<double,double,double> LaunchpadPos(Connection conn)
		  {
				var vessel = conn.SpaceCenter().ActiveVessel;
				var body = vessel.Orbit.Body;

				// Launchpad coordinates
				double landingLatitude = -0.0972069432271304;
				double landingLongitude = -300.557630926517;

			// Determine landing site reference frame
			// (orientation: x=zenith, y=north, z=east)
			var qLong = Tuple.Create(
			  0.0,
			  Math.Sin(-landingLongitude * 0.5 * Math.PI / 180.0),
			  0.0,
			  Math.Cos(-landingLongitude * 0.5 * Math.PI / 180.0)
			);
			var qLat = Tuple.Create(
			  0.0,
			  0.0,
			  Math.Sin(landingLatitude * 0.5 * Math.PI / 180.0),
			  Math.Cos(landingLatitude * 0.5 * Math.PI / 180.0)
			);
			Tuple<double, double, double> landingPosition = Tuple.Create(
				qLong.Item2 + qLat.Item2,
				qLong.Item3 + qLat.Item3,
				qLong.Item4 + qLat.Item4);
			return landingPosition;
		  }

		  

		  public static double PID(double P, double I, double D)
		  {

				throw new NotImplementedException();
		  }


	 }
}
