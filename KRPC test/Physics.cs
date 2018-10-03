using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Numerics;
using System.Threading;
using Tuple3 = System.Tuple<double, double, double>;
namespace KRPC_test
{
	partial class Physics
	{
		private static Connection conn = new Connection("Physics");
		private static Vessel vessel = conn.SpaceCenter().ActiveVessel;
		private static CelestialBody currentBody = vessel.Orbit.Body;
		private static ReferenceFrame bodyFrame = RefFrames.BodyFrame(conn);
		private static Stream<float> mass = conn.AddStream(() => vessel.Mass);
		private static Stream<float> thrust = conn.AddStream(() => vessel.AvailableThrust);
		private static Stream<double> velHStream = conn.AddStream(() => vessel.Flight(bodyFrame).HorizontalSpeed);
		private static Stream<double> altitudeStream = conn.AddStream(() => vessel.Flight(bodyFrame).MeanAltitude);
		private static Stream<Tuple<double, double, double>> posStream = conn.AddStream(() => vessel.Position(bodyFrame));
		private static Stream<Tuple<double, double, double>> dragStream = conn.AddStream(() => vessel.Flight(bodyFrame).Drag);
		public static Tuple<double,double,double> ImpactPosition { get => impactPos ; set => impactPos = value; }
		private static Tuple<double,double,double> impactPos = Tuple.Create(.0,.0,.0);
		const float g = 9.80665f;

		/// <summary>
		/// Gravitational acceleration at given altitude.
		/// </summary>
		/// <returns></returns>
		public static double AltG()
		{

			double altitude = altitudeStream.Get();
			return (currentBody.GravitationalParameter / Math.Pow(currentBody.EquatorialRadius + altitude, 2));
		}
		/// <summary>
		/// Centrifugal acceleration.
		/// </summary>
		/// <returns></returns>
		public static double CentrAcc()
		{
			double velH = velHStream.Get();
			double altitude = altitudeStream.Get();
			return (velH / (currentBody.EquatorialRadius + altitude));
		}
		/// <summary>
		/// Resulting acceleration recieved from gravity and centrifugal force.
		/// </summary>
		/// <returns></returns>
		public static double DownAcc()
		{
			return AltG() - CentrAcc();
		}


		/// <summary>
		/// Maximal acceleration provided by the ship's engines.
		/// </summary>
		/// <returns></returns>
		public static double MaxA()
		{
			return thrust.Get() / mass.Get();
		}

		public static Tuple<double, double, double> DragDeceleration()
		{
			Tuple<double, double, double> dragDeceleration = Vectors.MultiplyByNumber(dragStream.Get(), 1 / mass.Get());
			return dragDeceleration;
		}
		/// <summary>
		/// Predicted velocity based on current velocity, acceleration and drag
		/// </summary>
		private static Tuple<double, double, double> GetNextVelocity(Tuple<double, double, double> position, Tuple<double, double, double> velocity)
		{
			Tuple<double, double, double> vel = velocity;
			double gMag = AltG();
			Vector3 norm = Radar.MSLNormal(position);
			Tuple<double, double, double> dragDeceleration = DragDeceleration();
			Tuple<double, double, double> gDirection = Vectors.ToTuple(-norm);
			Tuple<double, double, double> g = Vectors.MultiplyByNumber(gDirection, gMag);
			vel = Vectors.Add(vel, dragDeceleration);
			return Vectors.Add(vel, g);
		}
		/// <summary>
		/// Position of the vessel in given time.
		/// </summary>
		/// <param name="position">Current position</param>
		/// <param name="velocity">Current velocity</param>
		/// <param name="time">Predition time</param>
		public static Tuple<double, double, double> GetImpactPosition(Tuple<double, double, double> position, Tuple<double, double, double> velocity, double time)
		{
			for (int t = 0; t < (Int32)time; t++)
			{
				position = Vectors.Add(position, velocity);
				velocity = GetNextVelocity(position, velocity);
				var surfacePos = Radar.SurfacePositionAtPosition(position);
				//If next position is below surface, return surface position.
				if (currentBody.AltitudeAtPosition(position, bodyFrame) < currentBody.AltitudeAtPosition(surfacePos, bodyFrame))
				{
					position = surfacePos;
					break;
				}
			}
			impactPos = position;
			return position;
		}
		/// <summary>
		/// Position of the vessel in given time.
		/// </summary>
		public static Tuple<double, double, double> GetImpactPosition(Stream<Tuple3> positionStream, Stream<Tuple3> velocityStream)
		{
			var position = positionStream.Get();
			var velocity = velocityStream.Get();
			for (int t = 0; t < 50; t++)
			{
				position = Vectors.Add(position, velocity);
				velocity = GetNextVelocity(position, velocity);
				var surfacePos = Radar.SurfacePositionAtPosition(position);
				//If next position is below surface, return surface position.
				if (currentBody.AltitudeAtPosition(position, bodyFrame) < currentBody.AltitudeAtPosition(surfacePos, bodyFrame))
				{
					position = surfacePos;
					break;
				}
			}
			impactPos = position;
			return position;
		}
		public static void GetImpPosThread(Stream<Tuple3> positionStream, Stream<Tuple3> velocityStream)
		{
			Thread thread = new Thread(() => GetImpactPosition(positionStream, velocityStream));
			thread.Start();
		}
	}
}
