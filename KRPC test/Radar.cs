using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Numerics;
using System.Threading;

namespace KRPC_test
{
	class Radar
	{

		private static Connection conn = new Connection("Radar");
		private static Vessel vessel = conn.SpaceCenter().ActiveVessel;
		private static CelestialBody body = vessel.Orbit.Body;
		private static readonly ReferenceFrame bodyFrame = body.ReferenceFrame;
		private static Stream<Tuple<double, double, double>> velStream = conn.AddStream(() => vessel.Flight(bodyFrame).Velocity);
		private static Stream<Tuple<double, double, double>> dragStream = conn.AddStream(() => vessel.Flight(bodyFrame).Drag);
		private static Stream<double> velVStream = conn.AddStream(() => vessel.Flight(bodyFrame).VerticalSpeed);
		private static Stream<double> altStream = conn.AddStream(() => vessel.Flight(bodyFrame).MeanAltitude);
		private static Stream<Tuple<double, double, double>> posStream = conn.AddStream(() => vessel.Position(bodyFrame));


		/// <summary>
		/// Calculate time before the impact.
		/// </summary>
		/// <returns>Time in seconds</returns>
		public static float ImpactTime()
		{
			double velV = velVStream.Get();
			double altitude = altStream.Get(); //remove 66 later
			double downAcc = Physics.DownAcc();
			float time = (float)((velV + Math.Sqrt(velV * velV + 2 * altitude * downAcc)) / Math.Max(downAcc, 0.001));
			return time;
		}

		private static double ImpactLocSurfaceHeight()
		{
			Vector3 vesselPosition = VectorMath.TupleToVector(posStream.Get());
			Vector3 newPosition = ImpactPos();
			double latitude = body.LatitudeAtPosition(VectorMath.VectorToTuple(newPosition), bodyFrame);
			double longitude = body.LongitudeAtPosition(VectorMath.VectorToTuple(newPosition), bodyFrame);
			return body.SurfaceHeight(latitude, longitude);
		}
		/// <summary>
		/// Altitude (terrain)
		/// </summary>
		/// <returns></returns>
		public static double RealAltitude()
		{
			return altStream.Get() - ImpactLocSurfaceHeight();
		}


		public static Tuple<double,double,double> SurfacePositionAtPosition(Tuple<double, double, double> position)
		{
			Tuple<double, double> latLon = LatLonAtPosition(position);
			return body.SurfacePosition(latLon.Item1, latLon.Item2, bodyFrame);
		}

		/// <summary>
		/// Calculate time before the impact with respect of surface elevation.
		/// </summary>
		/// <param name="realAltitude"></param>
		/// <returns>Time in seconds</returns>
		public static double RealImpactTime()
		{
			double velV = velVStream.Get();
			double downAcc = Physics.DownAcc();
			return (velV + Math.Sqrt(velV * velV + 2 * RealAltitude() * downAcc)) / Math.Max(downAcc, 0.001);
		}
		/// <summary>
		/// Calculates a distance needed to slow down
		/// </summary>
		/// <returns></returns>
		public static double BurnDistance()
		{
			Vector3 velocity = VectorMath.TupleToVector(velStream.Get()) + VectorMath.TupleToVector(Physics.DragDeceleration());
			return velocity.Length() * velocity.Length() / Physics.MaxA();
		}
		/// <summary>
		/// Calculates an impact position based on velocity and maximum acceleration. 
		/// </summary>
		/// <returns></returns>
		public static Vector3 ImpactPos()
		{
			Vector3 vesselPosition = VectorMath.TupleToVector(posStream.Get());
			Vector3 velocity = VectorMath.TupleToVector(velStream.Get());
			return vesselPosition + velocity * ImpactTime();
		}
		/// <summary>
		/// Gets a surface position at an impact location.
		/// </summary>
		/// <returns>Returns a vector of the real landing position</returns>
		private static Vector3 RealImpactPos()
		{
			Vector3 newPosition = ImpactPos();
			double latitude = body.LatitudeAtPosition(VectorMath.VectorToTuple(newPosition), bodyFrame);
			double longitude = body.LongitudeAtPosition(VectorMath.VectorToTuple(newPosition), bodyFrame);
			return VectorMath.TupleToVector(body.SurfacePosition(latitude, longitude, bodyFrame));

		}
		public static Tuple<double, double> LatLonAtPosition(Tuple<double, double, double> position)
		{
			double latitude = body.LatitudeAtPosition(position, body.ReferenceFrame);
			double longitude = body.LongitudeAtPosition(position, body.ReferenceFrame);

			return Tuple.Create(latitude, longitude);
		}
		internal static Vector3 MSLNormal(Tuple<double, double, double> pos)
		{
			double latitude = body.LatitudeAtPosition(pos, bodyFrame);
			double longitude = body.LongitudeAtPosition(pos, bodyFrame);
			Tuple<double, double, double> landingSpotMSL = body.MSLPosition(latitude, longitude, body.ReferenceFrame);
			//Spot 1
			latitude += 1 / 60 / 60 * 20;
			longitude += 1 / 60 / 60 * 20;
			Vector3 spotOnePositionMSL = VectorMath.TupleToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
			//Spot 2										  
			latitude -= 1.4 / 60 / 60 * 20;
			longitude += 0.3 / 60 / 60 * 20;
			Vector3 spotTwoPositionMSL = VectorMath.TupleToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
			//Spot 3										
			latitude += 0.4 / 60 / 60 * 20;
			longitude -= 1.4 / 60 / 60 * 20;
			Vector3 spotThreePositionMSL = VectorMath.TupleToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));

			Vector3 v1MSL = spotTwoPositionMSL - spotOnePositionMSL;
			Vector3 v2MSL = spotThreePositionMSL - spotOnePositionMSL;


			Vector3 n2 = Vector3.Normalize(Vector3.Cross(v1MSL, v2MSL)); // normal to the sea surface
			return n2;
		}

		public static Tuple<double, double> LandingSpotLatLong()
		{
			Tuple<double, double, double> distanceToTravel = VectorMath.MultiplyByNumber(velStream.Get(), RealTimeToImpact());
			Tuple<double, double, double> landingPos = VectorMath.Add(distanceToTravel, posStream.Get()); //predicted landing spot with no regard of surface elevation
			double latitude = body.LatitudeAtPosition(landingPos, body.ReferenceFrame);
			double longitude = body.LongitudeAtPosition(landingPos, body.ReferenceFrame);
			return Tuple.Create(latitude, longitude);
		}
		public static Vector3 SurfaceNormal(Tuple<double, double, double> position)
		{
			Tuple<double, double> latLon = LatLonAtPosition(position);
			double latitude = latLon.Item1;
			double longitude = latLon.Item2;
			//predicted landing spot with regard of surface elevation
			//Tuple<double, double, double> landingSpot = body.SurfacePosition(latitude, longitude, body.ReferenceFrame);
			double spread = 3; //How far away the measures will be taken. Affects precision.
												 //Spot 1
			latitude = latLon.Item1;
			longitude = latLon.Item2 + Math.Sqrt(3) / 4 / 60 / 60 * spread;
			Vector3 spotOnePosition = VectorMath.TupleToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
			//Spot 2										  
			latitude = latLon.Item1 - 0.5 / 60 / 60 * spread;
			longitude = latLon.Item2 - Math.Sqrt(3) / 4 / 60 / 60 * spread;
			Vector3 spotTwoPosition = VectorMath.TupleToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
			//Spot 3										
			latitude = latLon.Item1 + 0.5 / 60 / 60 * spread;
			longitude = latLon.Item2 - Math.Sqrt(3) / 4 / 60 / 60 * spread;
			Vector3 spotThreePosition = VectorMath.TupleToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
			//These vectors represent the sides of an equilateral triangle. The first two are for setting a normal. The third vector is only needed for visualization.
			Vector3 v1 = spotTwoPosition - spotOnePosition;
			Vector3 v2 = spotThreePosition - spotOnePosition;
			Vector3 v3 = spotThreePosition - spotTwoPosition;

			VectorMath.Visualize(conn, spotOnePosition, spotOnePosition + v1, bodyFrame);
			VectorMath.Visualize(conn, spotOnePosition, spotOnePosition + v2, bodyFrame);
			VectorMath.Visualize(conn, spotTwoPosition, spotTwoPosition + v3, bodyFrame);

			return Vector3.Cross(v1, v2); //normal to the surface that v1 and v2 lie on.
		}
		public static double Slope(Vector3 surfaceNormal, Tuple<double, double, double> position)
		{
			Vector3 n1 = surfaceNormal;
			Tuple<double, double> latLon = LatLonAtPosition(position);
			double latitude = latLon.Item1;
			double longitude = latLon.Item2;
			//Same as above but for mean Sea level
			Tuple<double, double, double> landingSpotMSL = body.MSLPosition(latitude, longitude, body.ReferenceFrame);
			//Spot 1
			latitude += 1 / 60 / 60 * 20;
			longitude += 1 / 60 / 60 * 20;
			Vector3 spotOnePositionMSL = VectorMath.TupleToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
			//Spot 2										  
			latitude -= 1.4 / 60 / 60 * 20;
			longitude += 0.3 / 60 / 60 * 20;
			Vector3 spotTwoPositionMSL = VectorMath.TupleToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
			//Spot 3										
			latitude += 0.4 / 60 / 60 * 20;
			longitude -= 1.4 / 60 / 60 * 20;
			Vector3 spotThreePositionMSL = VectorMath.TupleToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));

			Vector3 v1MSL = spotTwoPositionMSL - spotOnePositionMSL;
			Vector3 v2MSL = spotThreePositionMSL - spotOnePositionMSL;


			Vector3 n2 = Vector3.Cross(v1MSL, v2MSL); // normal to the sea surface
			return VectorMath.Angle(n1, n2);
		}


		/// <summary>
		/// Gets the height of the impact location.
		/// </summary>
		/// <returns></returns>
		
		private static float RealTimeToImpact()
		{
			double velV = velVStream.Get();
			double altitude = altStream.Get();
			double realAltitude = altitude - ImpactLocSurfaceHeight();
			return (float)(realAltitude / velV);
		}
		private static void DecideWhenToBurn()
		{
			Vector3 distanceToImpact = 10 * Vector3.UnitX;
			double distanceToBurn = 0;
			Vector3 velocity;
			Vector3 vesselPosition;
			while (distanceToBurn < distanceToImpact.Length())
			{
				velocity = VectorMath.TupleToVector(velStream.Get());
				var position = posStream.Get();
				vesselPosition = VectorMath.TupleToVector(position);

				distanceToBurn = BurnDistance();
				//Vector3 dist = vesselPosition - RealImpactPos();
				distanceToImpact = vesselPosition - VectorMath.TupleToVector(Physics.FuturePosition(position,velStream.Get(),ImpactTime()));
				//distanceToImpact = (velocity) * RealTimeToImpact();
				double halfCos = Math.Cos(VectorMath.Angle(distanceToImpact, -velocity) / 2);

				//Output.Print(Math.Round(distanceToImpact.Length()) + " | " + Math.Round(distanceToBurn, 2), 3);
				//Output.Print(distanceToBurn, 2);
				//Output.Print("distance to impact " + dist.Length(), 4);
				//Output.Print("drag " + Math.Round(VectorMath.Length(dragStream.Get())), 5);
			}
			OnDecision?.Invoke();


		}

		public delegate void DecisionHandler();
		public static event DecisionHandler OnDecision;
		/// <summary>
		/// Create a new thread and run DicideWhenToBurn() in it.
		/// </summary>
		public static void RunDeciderInNewThread()
		{
			Thread thread = new Thread(() => DecideWhenToBurn());
			thread.Start();

		}


	}
}
