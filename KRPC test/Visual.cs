using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Numerics;

namespace KRPC_test
{
	public class Visual
	{
		public static void Draw(Connection conn, Tuple<double, double, double> start, Vector3 vector, ReferenceFrame frame)
		{
			var t1 = Vectors.ToTuple(vector);
			conn.Drawing().AddLine(start, Vectors.MultiplyByNumber(t1, 1), frame);
		}
		public static void Draw(Connection conn, Vector3 start, Vector3 vector, ReferenceFrame frame)
		{
			var zero = Vectors.ToTuple(start);
			var t1 = Vectors.ToTuple(vector);
			conn.Drawing().AddLine(zero, t1, frame);
			System.Threading.Thread.Sleep(50);
		}
		public static void Draw(Connection conn, Vector3 vector, ReferenceFrame frame)
		{
			var zero = Tuple.Create(0.0, 0.0, 0.0);
			var t1 = Vectors.ToTuple(vector);
			conn.Drawing().AddLine(zero, t1, frame);
			System.Threading.Thread.Sleep(50);
		}
	}
}
