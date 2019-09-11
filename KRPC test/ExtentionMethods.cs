using System;
using System.Numerics;
using Tuple3 = System.Tuple<double, double, double>;

namespace KRPC_autopilot
{

    public static class ExtentionMethods
    {
        /// <summary>
        /// Check if vectors are equal.
        /// </summary>
        /// <param name="t1"></param>
        /// <param name="t2"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        public static bool Equals(this Tuple<double, double, double> t1, Tuple<double, double, double> t2, int tolerance)
        {
            double itemDifference = Math.Abs(t1.Item3 - t2.Item3);
            Output.Print(itemDifference, 6);
            if (Math.Abs(t1.Item1 - t2.Item1) / t1.Item1 < tolerance / 10 && Math.Abs(t1.Item2 - t2.Item2) / t1.Item2 < tolerance / 10 && Math.Abs(t1.Item3 - t2.Item3) / t1.Item3 < tolerance / 10)
            {

                return true;

            }
            else
            {
                return false;
            }
        }
        /// <summary>
        /// Add vectors.
        /// </summary>
        /// <param name="firstTuple"></param>
        /// <param name="secondTuple"></param>
        /// <returns></returns>
        public static Tuple<double, double, double> Plus(this Tuple<double, double, double> firstTuple, Tuple<double, double, double> secondTuple)
        {
            return Tuple.Create(
                firstTuple.Item1 + secondTuple.Item1,
                firstTuple.Item2 + secondTuple.Item2,
                firstTuple.Item3 + secondTuple.Item3);
        }
        /// <summary>
        /// Sumbract one vector from another.
        /// </summary>
        /// <param name="firstTuple"></param>
        /// <param name="secondTuple"></param>
        /// <returns></returns>
        public static Tuple<double, double, double> Minus(this Tuple<double, double, double> firstTuple, Tuple<double, double, double> secondTuple)
        {
            return Tuple.Create(
                firstTuple.Item1 - secondTuple.Item1,
                firstTuple.Item2 - secondTuple.Item2,
                firstTuple.Item3 - secondTuple.Item3);
        }
        /// <summary>
        /// Multiply a vector by a number.
        /// </summary>
        /// <param name="tuple"></param>
        /// <param name="number"></param>
        /// <returns></returns>
        public static Tuple<double, double, double> Mult(this Tuple<double, double, double> tuple, double number)
        {
            return new Tuple<double, double, double>(
                tuple.Item1 * number,
                tuple.Item2 * number,
                tuple.Item3 * number
                );
        }
        /// <summary>
		/// A projection of vector1 on vector2 multiplied by the length of vector2. Order matters.
		/// </summary>
		/// <param name="t1">First vector (tuple). Order doesn't matter.</param>
		/// <param name="t2">Second vector (tuple).</param>
		/// <returns>Dot product (double).</returns>
		public static double Dot(this Tuple<double, double, double> t1, Tuple<double, double, double> t2)
        {
            return
                t1.Item1 * t2.Item1 +
                t1.Item2 * t2.Item2 +
                t1.Item3 * t2.Item3;
        }
        /// <summary>
		/// A vector perpendicular to the surface of two initial vectors with a magnitude of their lengths multiplied.
		/// Directions: y(t1), z(t2), x(cross). All positive.
		/// </summary>
		/// <param name="t1">First vector (tuple). Order matters.</param>
		/// <param name="t2">Second vector (tuple).</param>
		/// <returns>Tuple.</returns>
        public static Tuple<double, double, double> Cross(this Tuple<double, double, double> t1, Tuple<double, double, double> t2)
        {
            return new Tuple<double, double, double>(
                t1.Item2 * t2.Item3 - t1.Item3 * t2.Item2,
                t1.Item3 * t2.Item1 - t1.Item1 * t2.Item3,
                t1.Item1 * t2.Item2 - t1.Item2 * t2.Item1
                );
        }
        public static double Length(this Tuple<double, double, double> t1)
        {
            return Math.Sqrt(
                t1.Item1 * t1.Item1 +
                t1.Item2 * t1.Item2 +
                t1.Item3 * t1.Item3
                );
        }
        public static Vector3 ToVector(this Tuple<double, double, double> tuple)
        {
            return new Vector3((float)tuple.Item1, (float)tuple.Item2, (float)tuple.Item3);
        }
        public static Tuple<double, double, double> ToTuple(this Vector3 t1)
        {
            return Tuple.Create((double)t1.X, (double)t1.Y, (double)t1.Z);
        }
        public static Tuple3 Normalise(this Tuple3 tuple)
        {
            return tuple.Mult(1 / tuple.Length());
        }
        public static Vector3 Normalise(this Vector3 vector)
        {
            return Vector3.Normalize(vector);
        }
        
        public static Tuple3 RotateByNinety(this Tuple3 tuple, string axis)
        {
            double[,] matrix = new double[,] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
            double pi = 90;
            if (axis == "x")
            {
                matrix = new double[,] { { 1, 0, 0 }, { 0, Math.Cos(pi), -Math.Sin(pi) }, { 0, Math.Sin(pi), Math.Cos(pi) } };
            }
            if (axis == "y")
            {
                matrix = new double[,] { { Math.Cos(pi), 0, Math.Sin(pi) }, { 0, 1, 0 }, { -Math.Sin(pi), 0, Math.Sin(pi) } };
            }
            if (axis == "z")
            {
                matrix = new double[,] { { Math.Cos(pi), -Math.Sin(pi), 0 }, { Math.Sin(pi), Math.Cos(pi), 0 }, { 0, 0, 1 } };
            }
            return Tuple.Create(tuple.Item1 * matrix[0, 0] + tuple.Item2 * matrix[0, 1] + tuple.Item3 * matrix[0, 2], tuple.Item1 * matrix[1, 0] + tuple.Item2 * matrix[1, 1] + tuple.Item3 * matrix[1, 2], tuple.Item1 * matrix[2, 0] + tuple.Item2 * matrix[2, 1] + tuple.Item3 * matrix[2, 2]);
        }
    }


}
