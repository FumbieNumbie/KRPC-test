using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using KRPC.Client.Services;
using Trajectories = KRPC.Client.Services.Trajectories.ExtensionMethods;
using Tuple3 = System.Tuple<double, double, double>;

namespace KRPC_test
{
	public static class Functions
	{
		public static class Landing
		{
			/// <summary>
			/// Land at a random place along the trajectory as efficiently as possible.
			/// </summary>
			public static void Land()
			{
				Connection conn = new Connection("Landing");
				Vessel vessel = conn.SpaceCenter().ActiveVessel;
				ReferenceFrame hybridFrame = RefFrames.CreateHybrid(conn);
				ReferenceFrame bodyFrame = RefFrames.BodyFrame(conn);
				ReferenceFrame surfFrame = vessel.SurfaceReferenceFrame;
				ReferenceFrame surfVelFrame = vessel.SurfaceVelocityReferenceFrame;
				//var launchPad = RefFrames.LaunchPad;
				ReferenceFrame currentFrame = bodyFrame;
				Stream<double> velVStream = conn.AddStream(() => vessel.Flight(currentFrame).VerticalSpeed);
				Stream<Tuple<double, double, double>> posStream = conn.AddStream(() => vessel.Position(currentFrame));
				Stream<Tuple<double, double, double>> velStream = conn.AddStream(() => vessel.Flight(currentFrame).Velocity);
				Stream<float> mass = conn.AddStream(() => vessel.Mass);
				Stream<float> thrust = conn.AddStream(() => vessel.AvailableThrust);
				CelestialBody body = vessel.Orbit.Body;



				//PID section
				PID.SetPoint = -1000;
				PID.Enabled = false;
				PID.RunInNewThread(thrust, mass, velVStream, vessel);

				//Control section
				vessel.Control.SAS = false;
				vessel.Control.RCS = false;
				vessel.AutoPilot.Engage();
				vessel.AutoPilot.ReferenceFrame = vessel.SurfaceReferenceFrame;
				vessel.AutoPilot.TargetDirection = Vectors.ToTuple(Vector3.UnitX);
				Tuple<double, double, double> steeringDir = Tuple.Create(.0, .0, .0);

				//Actual execution
				Start:
				PID.Enabled = true;

				DoLanding(vessel); // Set vertical speed to -5
				while (vessel.Situation.ToString() != "Landed")
				{

					//Manage steering
					steeringDir = HoverslamSteering(conn, vessel, surfVelFrame);
					vessel.AutoPilot.TargetDirection = steeringDir;
					//Define position variables
					var position = posStream.Get();
					var velocity = velStream.Get();
					Tuple3 trajImpactPos = Trajectories.Trajectories(conn).GetImpactPosition();
					Vector3 posVector = Vectors.ToVector(position);
					Physics.GetImpactPosition(position, velocity, Trajectories.Trajectories(conn).GetTimeTillImpact()+3);
					//Tuple3 landingPos = Physics.GetImpactPosition(position,velocity,Radar.ImpactTime());
					Tuple3 landingPos = Physics.ImpactPosition;
					Vector3 landingSpot = Vectors.ToVector(landingPos);
					//Vector3 landingSpot = ((posVector - Vectors.ToVector(landingPos) ) * 2 + posVector);
					//Slope
					var surfaceNormal = Radar.SurfaceNormal(landingPos);
					var slope = Radar.Slope(surfaceNormal, position);
					//Visualize landing spot
					Vector3 norm = Radar.MSLNormal(position);
					
					//Vector3 landingSpot = VectorMath.TupleToVector(body.SurfacePosition(predictedLatLon.Item1, predictedLatLon.Item2, bodyFrame));
					//Vector3 landingSpot = Vectors.ToVector(landingPos);
					Tuple<double, double> currentLatLon = Tuple.Create(body.LatitudeAtPosition(position, bodyFrame), body.LongitudeAtPosition(position, bodyFrame));
					Visual.Draw(conn, landingSpot, landingSpot + 50 * norm, bodyFrame);
					//Output
					double tTI = Trajectories.Trajectories(conn).GetTimeTillImpact() ;
					Output.Print("Slope: " + slope, 1);
					Output.Print("Time till impact: " + tTI, 2);
					Output.Print("My method: "+landingPos, 3);
					Output.Print("Trajectories: "+trajImpactPos, 4);
					//Save fuel and run hoverslam again if the first time was an overshoot.
					if (Radar.RealAltitude() > 55 && Vectors.Length(velocity) < 6)
					{
						PID.SetPoint = -35;
						PID.Enabled = false;
						goto Start;

					}
					//Console.Clear();
					System.Threading.Thread.Sleep(50);
					conn.Drawing().Clear();
				}
				Console.Clear();
				PID.SetPoint = -20;
				PID.Enabled = false;
				vessel.Control.Throttle = 0;
				System.Threading.Thread.Sleep(500);
			}
			/// <summary>
			/// Steering during landing.
			/// </summary>
			private static Tuple<double, double, double> HoverslamSteering(Connection conn, Vessel vessel, ReferenceFrame referenceFrame)
			{
				Vector3 steeringDirVec = Vector3.Zero;
				double verticalSpeed = vessel.Flight(referenceFrame).VerticalSpeed;
				if (verticalSpeed > 0)
				{
					vessel.AutoPilot.ReferenceFrame = vessel.SurfaceReferenceFrame;
					steeringDirVec = Vector3.UnitX;
				}
				else
				{
					vessel.AutoPilot.ReferenceFrame = vessel.SurfaceVelocityReferenceFrame;
					steeringDirVec = -Vector3.UnitY;
				}
				return Vectors.ToTuple(steeringDirVec);

			}
			/// <summary>
			/// Subscribe to burn decider event.
			/// </summary>
			private static void DoLanding(Vessel vessel)
			{
				Radar.RunDeciderInNewThread();
				Radar.OnDecision += () => { PID.SetPoint = -5; vessel.Control.Gear = true; };
			}
		}

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
