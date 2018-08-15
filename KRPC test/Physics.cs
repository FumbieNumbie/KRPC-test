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
	class Physics
	{
		private static Connection conn = new Connection("Physics");
		private static Vessel vessel = conn.SpaceCenter().ActiveVessel;
		private static CelestialBody currentBody = vessel.Orbit.Body;
		private static ReferenceFrame bodyFrame = RefFrames.BodyFrame(conn);
		private static Stream<float> mass = conn.AddStream(() => vessel.Mass);
		private static Stream<float>  thrust = conn.AddStream(() => vessel.AvailableThrust);
		private static Stream<double> velHStream = conn.AddStream(() => vessel.Flight(bodyFrame).HorizontalSpeed);
		private static Stream<double> altitudeStream = conn.AddStream(() => vessel.Flight(bodyFrame).MeanAltitude);

		

		/// <summary>
		/// Gravitational acceleration at given altitude.
		/// </summary>
		/// <returns></returns>
		public static double AltG()
		{
			
			double altitude = altitudeStream.Get();
			return currentBody.GravitationalParameter / Math.Pow(currentBody.EquatorialRadius + altitude, 2);
		}
		/// <summary>
		/// Centrifugal acceleration.
		/// </summary>
		/// <returns></returns>
		public static double CentrAcc()
		{
			double velH = velHStream.Get();
			double altitude = altitudeStream.Get();
			return velH / (currentBody.EquatorialRadius + altitude);
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

	}
}
