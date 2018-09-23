using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Numerics;

namespace KRPC_test
{
	static class VectorMath
	{

		public static Tuple<double, double, double> Subtract(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			return new Tuple<double, double, double>(
				t1.Item1 - t2.Item1,
				t1.Item2 - t2.Item2,
				t1.Item3 - t2.Item3
				);
		}
		public static Tuple<double, double, double> Add(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			return new Tuple<double, double, double>(
				t1.Item1 + t2.Item1,
				t1.Item2 + t2.Item2,
				t1.Item3 + t2.Item3
				);
		}
		/// <summary>
		/// A projection of vector1 on vector2 multiplied by the length of vector2.
		/// </summary>
		/// <param name="t1">First vector (tuple). Order doesn't matter.</param>
		/// <param name="t2">Second vector (tuple).</param>
		/// <returns>Dot product (double).</returns>
		public static double Dot(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			return
				t1.Item1 * t2.Item1 +
				t1.Item2 * t2.Item2 +
				t1.Item3 * t2.Item3;
		}
		public static double Cos(Vector3 v1, Vector3 v2)
		{
			double dot = Vector3.Dot(v1, v2);
			double cos = dot / v1.Length() / v2.Length();
			return cos;
		}
		/// <summary>
		/// An angle between two vectors
		/// </summary>
		/// <param name="v1">First vector</param>
		/// <param name="v2">Second vector</param>
		/// <param name="returnDegrees">True if the method should return degrees. Default value is true.</param>
		/// <returns>Angle in degrees or radians</returns>
		public static double Angle(Vector3 v1, Vector3 v2, bool returnDegrees = true)
		{

			double cos = Cos(v1, v2);
			double angle = Math.Acos(cos);
			if (returnDegrees == true)
			{
				return angle * 360.0 / (2 * Math.PI);
			}
			else return angle;
		}

		public static Tuple<double, double, double> MultiplyByNumber(Tuple<double, double, double> tuple, double number)
		{
			return new Tuple<double, double, double>(
				tuple.Item1 * number,
				tuple.Item2 * number,
				tuple.Item3 * number
				);
		}
		/// <summary>
		/// A vector perpendicular to the surface of two initial vectors with a magnitude of their lengths multiplied.
		/// Directions: y(t1), z(t2), x(cross). All positive.
		/// </summary>
		/// <param name="t1">First vector (tuple). Order matters.</param>
		/// <param name="t2">Second vector (tuple).</param>
		/// <returns>Tuple.</returns>
		public static Tuple<double, double, double> Cross(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			return new Tuple<double, double, double>(
				t1.Item2 * t2.Item3 - t1.Item3 * t2.Item2,
				t1.Item3 * t2.Item1 - t1.Item1 * t2.Item3,
				t1.Item1 * t2.Item2 - t1.Item2 * t2.Item1
				);
		}
		public static double Length(Tuple<double, double, double> t1)
		{
			return Math.Sqrt(
				t1.Item1 * t1.Item1 +
				t1.Item2 * t1.Item2 +
				t1.Item3 * t1.Item3
				);
		}
		public static Tuple<double, double, double> Normalize(Tuple<double, double, double> t1)
		{
			double l = Length(t1);
			return new Tuple<double, double, double>(
				t1.Item1 / l,
				t1.Item2 / l,
				t1.Item3 / l
				);
		}
		public static double Distance(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			return Math.Round(Length(Subtract(t2, t1)), 2);
		}

		public static Tuple<double, double, double> Round(Tuple<double, double, double> t1)
		{
			return Tuple.Create(
				Math.Round(t1.Item1, 2),
				Math.Round(t1.Item2, 2),
				Math.Round(t1.Item3, 2)
				);
		}
		public static double GroundDistance(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			var t3 = Tuple.Create(
				0.0,
				Subtract(t2, t1).Item2,
				Subtract(t2, t1).Item3
				);

			return Math.Round(Length(t3), 2);
		}
		/// <summary>
		/// Item2: South if positive. Item3: West if positive. 
		/// </summary>
		/// <param name="t1"></param>
		/// <param name="t2"></param>
		/// <returns>Tuple with x set to 0</returns>
		public static Tuple<double, double, double> GroundDistVec(Tuple<double, double, double> t1, Tuple<double, double, double> t2)
		{
			var t3 = Tuple.Create(
				0.0,
				Subtract(t1, t2).Item2,
				Subtract(t1, t2).Item3
				);
			return t3;
		}
		public static Tuple<double, double, double> GroundAnything(Tuple<double, double, double> t1)
		{
			return Tuple.Create(
				0.0,
				t1.Item2,
				t1.Item3);
		}



		public static Vector3 TupleToVector(Tuple<double, double, double> tuple)
		{
			return new Vector3((float)tuple.Item1, (float)tuple.Item2, (float)tuple.Item3);
		}
		public static Vector3 GetGroundVector(Tuple<double, double, double> tuple)
		{
			return new Vector3(0, (float)tuple.Item2, (float)tuple.Item3);
		}
		public static Vector3 GetGroundVector(Vector3 v)
		{
			return new Vector3(0, (float)v.Y, (float)v.Z);
		}
		public static Tuple<double, double, double> VectorToTuple(Vector3 t1)
		{
			return Tuple.Create((double)t1.X, (double)t1.Y, (double)t1.Z);
		}
		/// <summary>
		/// Projects the first vector on the second.
		/// </summary>
		/// <param name="v1">First vector.</param>
		/// <param name="v2">Second vector.</param>
		/// <returns>Projection on vector 2.</returns>
		public static Vector3 Project(Vector3 v1, Vector3 v2)
		{
			float cos = Vector3.Dot(v1, v2) / v1.Length() / v2.Length();
			return v1.Length() * cos * v2;
		}

		public static Vector3 SideError(Vector3 v1, Vector3 v2)
		{
			Vector3 proj = Project(Vector3.Normalize(v1), Vector3.Normalize(v2));
			float v3mag = (float)Math.Sqrt(v1.Length() - proj.Length());
			Vector3 v3 = Vector3.Normalize(Vector3.Cross(Vector3.UnitX, v2));
			return v3 * v3mag;

		}

		public static double Slope(Connection connection, Vector3 position)
		{
			CelestialBody body = connection.SpaceCenter().ActiveVessel.Orbit.Body;
			var vessel = connection.SpaceCenter().ActiveVessel;
			double latitude = body.LatitudeAtPosition(VectorMath.VectorToTuple(position), body.ReferenceFrame);
			double longtitude = body.LongitudeAtPosition(VectorMath.VectorToTuple(position), body.ReferenceFrame);
			Tuple<double, double, double> landingSpot = body.SurfacePosition(latitude, longtitude, body.ReferenceFrame);
			//Spot 1
			latitude += 1 / 60 / 60 * 5;
			longtitude += 1 / 60 / 60 * 5;
			Vector3 spotOnePosition = TupleToVector(body.SurfacePosition(latitude, longtitude, body.ReferenceFrame));
			//Spot 2										  
			latitude -= 1.4 / 60 / 60 * 5;
			longtitude += 0.3 / 60 / 60 * 5;
			Vector3 spotTwoPosition = TupleToVector(body.SurfacePosition(latitude, longtitude, body.ReferenceFrame));
			//Spot 3										
			latitude += 0.4 / 60 / 60 * 5;
			longtitude -= 1.4 / 60 / 60 * 5;
			Vector3 spotThrePosition = TupleToVector(body.SurfacePosition(latitude, longtitude, body.ReferenceFrame));

			Vector3 v1 = spotTwoPosition - spotOnePosition;
			Vector3 v2 = spotThrePosition - spotOnePosition;

			Vector3 n1 = Vector3.Cross(v1, v2);
			//Mean See level
			Tuple<double, double, double> landingSpotMSL = body.MSLPosition(latitude, longtitude, body.ReferenceFrame);
			//Spot 1
			latitude += 1 / 60 / 60 * 5;
			longtitude += 1 / 60 / 60 * 5;
			Vector3 spotOnePositionMSL = TupleToVector(body.MSLPosition(latitude, longtitude, body.ReferenceFrame));
			//Spot 2										  
			latitude -= 1.4 / 60 / 60 * 5;
			longtitude += 0.3 / 60 / 60 * 5;
			Vector3 spotTwoPositionMSL = TupleToVector(body.MSLPosition(latitude, longtitude, body.ReferenceFrame));
			//Spot 3										
			latitude += 0.4 / 60 / 60 * 5;
			longtitude -= 1.4 / 60 / 60 * 5;
			Vector3 spotThrePositionMSL = TupleToVector(body.MSLPosition(latitude, longtitude, body.ReferenceFrame));

			Vector3 v1MSL = spotTwoPositionMSL - spotOnePositionMSL;
			Vector3 v2MSL = spotThrePositionMSL - spotOnePositionMSL;


			Vector3 n2 = Vector3.Cross(v1MSL, v2MSL);
			return Angle(n1, n2);
		}

		public static void Visualize(Connection conn, Tuple<double, double, double> start, Vector3 vector, ReferenceFrame frame)
		{
			var t1 = VectorToTuple(vector);
			conn.Drawing().AddLine(start, MultiplyByNumber(t1, 1), frame);
			//conn.Drawing().Clear();


		}
		public static void Visualize(Connection conn, Vector3 start, Vector3 vector, ReferenceFrame frame)
		{


			var zero = VectorToTuple(start);
			var t1 = VectorToTuple(vector);
			conn.Drawing().AddLine(zero, t1, frame);
			System.Threading.Thread.Sleep(50);

			//conn.Drawing().Clear();
		}
		public static void Visualize(Connection conn, Vector3 vector, ReferenceFrame frame)
		{


			var zero = Tuple.Create(0.0, 0.0, 0.0);
			var t1 = VectorToTuple(vector);
			conn.Drawing().AddLine(zero, t1, frame);
			System.Threading.Thread.Sleep(50);

			//conn.Drawing().Clear();
		}
		/// <summary>
		/// Check if given tuples are equal withing specified tolerance
		/// </summary>
		/// <param name="t1">First tuple.</param>
		/// <param name="t2">Second tuple.</param>
		/// <param name="tolerance">Tolerance in meters/10.</param>
		/// <returns>bool</returns>
		public static bool TuplesAreEqual(Tuple<double, double, double> t1, Tuple<double, double, double> t2, int tolerance)
		{
			double itemDifference = Math.Abs(t1.Item3 - t2.Item3);
			Output.Print(itemDifference , 6);
			if (Math.Abs(t1.Item1 - t2.Item1) / t1.Item1 < tolerance / 10 && Math.Abs(t1.Item2 - t2.Item2) / t1.Item2 < tolerance / 10 && Math.Abs(t1.Item3 - t2.Item3) / t1.Item3 < tolerance / 10)
			{

				return true;

			}
			else
			{
				return false;
			}
		}

	}
}
