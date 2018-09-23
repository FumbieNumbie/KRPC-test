using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System.Numerics;
using System.Threading;


namespace KRPC_test
{
	partial class Physics
	{
		private static Connection conn = new Connection("Physics");
		private static Vessel vessel = conn.SpaceCenter().ActiveVessel;
		private static CelestialBody currentBody = vessel.Orbit.Body;
		private static ReferenceFrame bodyFrame = RefFrames.BodyFrame(conn);
		private static Stream<float> mass = conn.AddStream(() => vessel.Mass);
		private static Stream<float>  thrust = conn.AddStream(() => vessel.AvailableThrust);
		private static Stream<double> velHStream = conn.AddStream(() => vessel.Flight(bodyFrame).HorizontalSpeed);
		private static Stream<double> altitudeStream = conn.AddStream(() => vessel.Flight(bodyFrame).MeanAltitude);
		private static Stream<Tuple<double, double, double>> posStream = conn.AddStream(() => vessel.Position(bodyFrame));
		private static Stream<Tuple<double, double, double>> dragStream = conn.AddStream(() => vessel.Flight(bodyFrame).Drag);


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

		public static Tuple<double,double,double> DragDeceleration()
		{
			Tuple<double, double, double> dragDeceleration = VectorMath.MultiplyByNumber(dragStream.Get(), 1 / mass.Get());
			return dragDeceleration;
		}

		private static Tuple<double, double, double> NextVelocity(Tuple<double,double,double> position, Tuple<double, double, double> velocity)
		{
			Tuple<double, double, double> vel = velocity;
			double gMag = AltG();
			Vector3 norm = Radar.MSLNormal(position);
			Tuple<double, double, double> dragDeceleration = DragDeceleration();
			//dragDeceleration = VectorMath.MultiplyByNumber(dragDeceleration, 1 / 2);
			Tuple<double, double, double> gDirection = VectorMath.VectorToTuple(-norm);
			Tuple<double, double, double> g = VectorMath.MultiplyByNumber(gDirection, gMag);
			vel = VectorMath.Add(vel, dragDeceleration);
			return VectorMath.Add(vel,g);
		}
		/// <summary>
		/// Position of the vessel in given time.
		/// </summary>
		/// <param name="position">Current position</param>
		/// <param name="velocity">Current velocity</param>
		/// <param name="time">Predition time</param>
		/// <returns>Future position of the vessel.</returns>
		public static Tuple<double, double, double> FuturePosition(Tuple<double, double, double> position, Tuple<double, double, double> velocity, double time)
		{
			for (int t = 0; t < (Int32)time; t++)
			{
				position = VectorMath.Add(position,velocity);
				velocity = NextVelocity(position,velocity);
				//Check if predicted position if below surface
				var surfacePos = Radar.SurfacePositionAtPosition(position);
				if (currentBody.AltitudeAtPosition(position, bodyFrame) < currentBody.AltitudeAtPosition(surfacePos,bodyFrame))
				{
					position = surfacePos;
					break;//If yes, return surface position instead.
				}
			}
			return position;
		}
	}
}
