using Compute;
using KRPC.Client;
using KRPC.Client.Services.Drawing;
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
			Vessel vessel = conn.SpaceCenter().ActiveVessel;
			ReferenceFrame surfaceBodyHybrid = RefFrames.SurfaceBodyHybrid(conn);
			ReferenceFrame padRef = RefFrames.CreateLaunchPadRef(conn);
			ReferenceFrame hybridFrame = RefFrames.CreateHybrid(conn);
			ReferenceFrame bodyFrame = RefFrames.BodyFrame(conn);
			ReferenceFrame surfFrame = vessel.SurfaceReferenceFrame;
			ReferenceFrame surfVelFrame = vessel.SurfaceVelocityReferenceFrame;
			//padRef = vessel.Orbit.Body.ReferenceFrame;
			//var launchPad = RefFrames.LaunchPad;
			ReferenceFrame currentFrame = bodyFrame;
			Tuple<double, double, double> launchPadPos = vessel.Position(bodyFrame); //saves the position of the vessel on the latic aunchpad
			Stream<double> velVStream = conn.AddStream(() => vessel.Flight(currentFrame).VerticalSpeed);
			Stream<double> altStream = conn.AddStream(() => vessel.Flight(currentFrame).MeanAltitude);
			Stream<Tuple<double, double, double>> posStream = conn.AddStream(() => vessel.Position(currentFrame));
			Stream<double> surfElevation = conn.AddStream(() => vessel.Flight(currentFrame).Elevation);

			Stream<Tuple<double, double, double>> velStream = conn.AddStream(() => vessel.Flight(currentFrame).Velocity);
			Stream<Tuple<double, double, double>> dragStream = conn.AddStream(() => vessel.Flight(surfVelFrame).Drag);
			Stream<float> mass = conn.AddStream(() => vessel.Mass);
			Stream<float> thrust = conn.AddStream(() => vessel.AvailableThrust);
			CelestialBody body = vessel.Orbit.Body;
			launchPadPos = RefFrames.LaunchPad;



			//PID section

			//altPID pID = new altPID(0.411, 0.469714286, 0.08990625,1,0);
			PID.KP = 0.728;
			PID.KI = 1.093;
			PID.KD = 0.0323;
			PID.SetPoint = -500;
			PID.Enabled = false;

			//Control section
			var zero = Tuple.Create(0.0, 0.0, 0.0);
			Vector3 up = new Vector3(1, 0, 0);
			var groundVelVec = new Vector3(0, 0, 0);
			var groundDistVec = new Vector3(0, 0, 0);
			var vecDifference = new Vector3(0, 0, 0);
			Vector3 anticipatedDist = Vector3.Zero;
			Vector3 sideSteering = Vector3.Zero;
			vessel.Control.SAS = false;
			vessel.Control.RCS = false;
			vessel.AutoPilot.Engage();
			vessel.AutoPilot.ReferenceFrame = vessel.SurfaceReferenceFrame;
			vessel.AutoPilot.TargetDirection = VectorMath.VectorToTuple(Vector3.UnitX);
			Tuple<double, double, double> steeringDir = Tuple.Create(.0, .0, .0);

			//vessel.AutoPilot.TargetPitchAndHeading(45, 270);
			//vessel.Control.Throttle = 1;

			//System.Threading.Thread.Sleep(5000);
			//vessel.AutoPilot.TargetDirection = VectorMath.VectorToTuple(Vector3.UnitX);
			//vessel.Control.Throttle = 0;


			PID.RunInNewThread(thrust, mass, velVStream, vessel);

			Start:
			PID.Enabled = true;

			//PID.SetPoint = -3;
			DoLanding(conn, vessel, surfVelFrame);
			while (vessel.Situation.ToString() != "Landed")
			{
				var velocity = velStream.Get();
				//Manage steering
				steeringDir = HoverslamSteering(conn, vessel, surfVelFrame);
				vessel.AutoPilot.TargetDirection = steeringDir;
				//Define position variables
				var position = posStream.Get();
				var predictedPos = Physics.FuturePosition(position, velocity, Radar.ImpactTime());
				//Slope
				var surfaceNormal = Radar.SurfaceNormal(predictedPos);
				var slope = Radar.Slope(surfaceNormal, position);
				//Visualize landing spot
				Vector3 norm = Radar.MSLNormal(position);
				Tuple<double, double> predictedLatLon = Tuple.Create(body.LatitudeAtPosition(predictedPos, bodyFrame), body.LongitudeAtPosition(predictedPos, bodyFrame));
				Vector3 landingSpot = VectorMath.TupleToVector(body.SurfacePosition(predictedLatLon.Item1, predictedLatLon.Item2, bodyFrame));
				Tuple<double, double> currentLatLon = Tuple.Create(body.LatitudeAtPosition(position, bodyFrame), body.LongitudeAtPosition(position, bodyFrame));
				VectorMath.Visualize(conn, landingSpot, landingSpot + 50 * norm, bodyFrame);
				//Output
				Output.Print(slope, 1);
				//Save fuel and run hoverslam again if the first time was an overshoot.
				if (Radar.RealAltitude() > 75 && VectorMath.Length(velocity) < 6)
				{
					PID.SetPoint = -35;
					PID.Enabled = false;
					goto Start;

				}
				//Console.Clear();
				System.Threading.Thread.Sleep(100);
				conn.Drawing().Clear();
			}
			Console.Clear();
			PID.SetPoint = -20;
			PID.Enabled = false;
			vessel.Control.Throttle = 0;
			System.Threading.Thread.Sleep(500);
			Environment.Exit(0);
		}

		private static void DoLanding(Connection conn, Vessel vessel, ReferenceFrame surfVelFrame)
		{
			//Land using last second burn
			Radar.RunDeciderInNewThread();
			Radar.OnDecision += DoHoverSlam;

		}



		private static Tuple<double, double, double> HoverslamSteering(Connection conn, Vessel vessel, ReferenceFrame referenceFrame)
		{
			Vector3 prograde = Vector3.Normalize(VectorMath.TupleToVector(vessel.Flight(referenceFrame).Velocity));
			Vector3 steeringDirVec = Vector3.Zero;
			Tuple<double, double, double> steeringDirection = Tuple.Create(0.0, 0.0, 0.0);
			double verticalSpeed = vessel.Flight(referenceFrame).VerticalSpeed;
			double realAltitude = vessel.Flight(referenceFrame).MeanAltitude - vessel.Flight(referenceFrame).Elevation;
			if (verticalSpeed > 0)
			{
				vessel.AutoPilot.ReferenceFrame = vessel.SurfaceReferenceFrame;
				steeringDirVec = Vector3.UnitX;
			}
			else
			{
				vessel.AutoPilot.ReferenceFrame = vessel.SurfaceVelocityReferenceFrame;
				steeringDirVec = -Vector3.UnitY/*-0.1f*prograde*/;
			}

			steeringDirection = VectorMath.VectorToTuple(steeringDirVec);
			return steeringDirection;

		}
		/// <summary>
		/// Finds a postition with an acceptable slope and goes there.
		/// </summary>
		public static void GoToBetterSlopePos()
		{
			//check slope 20 meters further
			//compare it to current slope
			//check slope 20 meters closer
			//compare
			//repeat within acceptable deltaV range
			//pick spot, burn horizontally
		}
		private static void DoHoverSlam()
		{
			PID.SetPoint = -5;
		}
		private static void GetSteeringVector(out string state, double groundDist, Vector3 groundVelVec, Vector3 groundDistVec, out Tuple<double, double, double> steeringDir, double dir)
		{
			if (groundDist > 300)
			{
				if (groundVelVec.Length() * dir < 25)
				{
					steeringDir = VectorMath.VectorToTuple(Vector3.UnitX + (1.3f * Vector3.Normalize(groundDistVec) - Vector3.Normalize(groundVelVec)));
				}
				else
				{
					steeringDir = VectorMath.VectorToTuple(Vector3.UnitX);
				}
				state = "groundDist > 300";
			}
			else if (groundDist > 30 && groundDist < 300)
			{
				state = "30 < groundDist < 300";
				if (groundVelVec.Length() > 5)
				{
					steeringDir = VectorMath.VectorToTuple(Vector3.UnitX - 0.01f * groundVelVec);
				}
				else
				{
					steeringDir = VectorMath.VectorToTuple(Vector3.UnitX + 0.1f * (1.3f * Vector3.Normalize(groundDistVec) - Vector3.Normalize(groundVelVec)));
				}
			}
			else
			{
				if (groundVelVec.Length() > 2)
				{
					steeringDir = VectorMath.VectorToTuple(Vector3.UnitX - 0.03f * groundVelVec);
				}
				else
				{
					steeringDir = VectorMath.VectorToTuple(Vector3.UnitX + 0.03f * (1.3f * Vector3.Normalize(groundDistVec) - Vector3.Normalize(groundVelVec)));
				}
				state = "else";
			}
		}



		private static string Land(string state, double realAltitude)
		{
			if (realAltitude > 550)
			{
				PID.SetPoint = -35;
				state = "Above 550";
			}
			if (realAltitude < 550)
			{
				PID.SetPoint = -25;
				state = "Below 550";

			}
			if (realAltitude < 70)
			{
				PID.SetPoint = -10;
				state = "Below 70";
			}
			if (realAltitude < 35)
			{
				PID.SetPoint = -4;
				state = "Below 25";
			}

			return state;
		}





		private static void IterateVessels(Connection conn)
		{
			int i = 3;
			foreach (var ves in conn.SpaceCenter().Vessels)
			{
				Output.Print(ves.Name, i);
				i++;
			}
		}

		public static void Visualize(List<object> x, List<double> y, List<double> baseline)
		{

			//prepare chart control...
			Chart chart = new Chart();


			chart.Width = 600;
			chart.Height = 400;
			//create serie...
			Series serie1 = new Series();
			serie1.Points.DataBindXY(x, baseline);
			serie1.Name = "Serie1";
			serie1.Color = Color.FromArgb(112, 255, 200);
			serie1.BorderColor = Color.FromArgb(164, 164, 164);
			serie1.ChartType = SeriesChartType.Line;
			serie1.IsVisibleInLegend = false;
			serie1.BorderDashStyle = ChartDashStyle.Solid;
			serie1.BorderWidth = 2;
			serie1.ShadowColor = Color.FromArgb(128, 128, 128);
			serie1.ShadowOffset = 1;
			serie1.IsValueShownAsLabel = false;
			serie1.XValueMember = "Time";
			serie1.YValueMembers = "Speed";
			serie1.Font = new Font("Tahoma", 8.0f);
			serie1.BackSecondaryColor = Color.FromArgb(0, 102, 153);
			serie1.LabelForeColor = Color.FromArgb(100, 100, 100);
			//serie2
			Series serie2 = new Series();
			serie2.Points.DataBindXY(x, y);
			serie2.Name = "Serie2";
			serie2.Color = Color.FromArgb(150, 50, 50);
			serie2.BorderColor = Color.FromArgb(164, 164, 164);
			serie2.ChartType = SeriesChartType.Line;
			serie2.IsVisibleInLegend = false;
			serie2.BorderDashStyle = ChartDashStyle.Solid;
			serie2.BorderWidth = 1;
			serie2.ShadowColor = Color.FromArgb(128, 128, 128);
			serie2.ShadowOffset = 1;
			serie2.IsValueShownAsLabel = false;
			serie2.Font = new Font("Tahoma", 8.0f);
			serie2.BackSecondaryColor = Color.FromArgb(0, 102, 153);
			serie2.LabelForeColor = Color.FromArgb(100, 100, 100);
			chart.Series.Add(serie1);
			chart.Series.Add(serie2);
			//create chartareas...
			ChartArea ca = new ChartArea();
			ca.Name = "ChartArea1";
			ca.BackColor = Color.White;
			ca.BorderColor = Color.FromArgb(26, 59, 105);
			ca.BorderWidth = 0;
			ca.BorderDashStyle = ChartDashStyle.Solid;
			ca.AxisX = new Axis();
			ca.AxisY = new Axis();
			chart.ChartAreas.Add(ca);
			//databind...
			chart.DataBind();
			//save result...
			chart.SaveImage(@"C:\Users\aalex\Desktop\myChart.png", ChartImageFormat.Png);

		}
		private static double Input(ConsoleKey key)
		{
			string keyName = key.ToString();
			if (keyName == "DownArrow")
			{
				return -5;
			}
			else if (keyName == "UpArrow")
			{
				return 5;
			}
			else return 0;
		}

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
					Calculations.Stage(vessel);


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
			Calculations.AddNode(vessel, ut);
			flightMode = 3;
			List<Node> nodes = new List<Node>(vessel.Control.Nodes);
			Node node = nodes[0];

			// Calculate burn time (using rocket equation)
			double burnTime = Calculations.GetBurnTime(vessel, node);

			// Warp to burn
			Console.WriteLine("Waiting until circularization burn");
			double burnUT = ut.Get() + node.TimeTo - burnTime / 2;
			Calculations.WarpToNode(conn, altitude, node, burnTime + 4, burnUT);

			//Orient to node prograde
			vessel.AutoPilot.ReferenceFrame = node.ReferenceFrame;
			vessel.AutoPilot.TargetDirection = Tuple.Create(0.0, 1.0, 0.0);


			//Wait for the perfect time to burn
			while (node.TimeTo > (burnTime + 1) / 2)
			{
				//do nothing
			}
			vessel.Control.RCS = false;
			Calculations.ExeNode(vessel);



			node.Remove();

			vessel.AutoPilot.Disengage();
			vessel.Control.Throttle = 0;
			conn.Dispose();
		}
	}
}
