using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace KRPC_test
{
	static class PID
	{

		private static double kP = 0.0;
		private static double kI = 0.0;
		private static double kD = 0.0;
		private static double p = 0.001;
		private static double i = 0.00;
		private static double d = 0.00;
		private static double minOut = 0;
		private static double maxOut;
		private static double lastInput = 0;
		private static bool calculateD = false;
		private static DateTime lastTime = DateTime.Now;
		public static double MaxOut { get => maxOut; set => maxOut = value; }
		public static double SetPoint { get; set; }
		public static double ClampI { get; set; } = 1;
		public static bool Enabled { get; set; }
		public static double KP { get => kP; set => kP = value; }
		public static double KI { get => kI; set => kI = value; }
		public static double KD { get => kD; set => kD = value; }

		private static Connection conn = new Connection("PID");
		private static Vessel vessel = conn.SpaceCenter().ActiveVessel;
		private static ReferenceFrame padRef = RefFrames.CreateLaunchPadRef(conn);
		private static Stream<double> velVStream = conn.AddStream(() => vessel.Flight(padRef).VerticalSpeed);
		private static Stream<float> mass = conn.AddStream(() => vessel.Mass);
		private static Stream<float> thrust = conn.AddStream(() => vessel.AvailableThrust);
		private static double maxA;
		private static double maxTWR;
		private static double velV = 0;
		private static Thread thread;

		public static void PIDThread()
		{

			thread = new Thread(Update);
			thread.Start();


		}

		//public PID(double _p = 0.0, double _i = 0.0, double _d = 0.00)
		//{
		//	kP = _p;
		//	kI = _i;
		//	kD = _d;
		//}


		public static Tuple<double, double, double> GetCoefficients()
		{
			return new Tuple<double, double, double>(KP, KI, KD);
		}

		public static void Update()
		{

			while (true)
			{

				maxA = thrust.Get() / mass.Get();
				maxTWR = maxA / 9.81;
				maxOut = maxTWR;
				velV = velVStream.Get();
				DateTime now = DateTime.Now;
				TimeSpan timePassed = now - lastTime;
				if (timePassed == null || timePassed.TotalSeconds == 0)
				{
					timePassed = new TimeSpan(0, 0, 1);
				}
				double error = SetPoint - velV;
				p = error;
				i += error * timePassed.TotalSeconds;
				i = Clamp(i);
				if (calculateD & KD != 0)
				{
					d = (velV - lastInput) / timePassed.TotalSeconds;
				}
				lastInput = velV;
				lastTime = now;
				calculateD = true;

				double output = KP * p + KI * i - KD * d;
				if (output > maxOut)
				{
					output = maxOut;
				}
				else if (output < minOut)
				{
					output = minOut;
				}
				if (Enabled == true)
				{
					vessel.Control.Throttle = (float)(output / maxTWR);

				}
				else
				{
					vessel.Control.Throttle = 0;

				}
				//Output.Line(velV, 1);
				//Output.Line(output, 2);
			}
		}
		public static double Update(double input, double maxAcc)
		{
			DateTime now = DateTime.Now;
			TimeSpan timePassed = now - lastTime;
			if (timePassed == null || timePassed.TotalSeconds == 0)
			{
				timePassed = new TimeSpan(0, 0, 1);
			}

			double twrFactor = 4.87 / maxAcc;
			double error = (SetPoint - input);
			p = error;
			i += error * timePassed.TotalSeconds;
			i = Clamp(i);
			if (calculateD & KD != 0)
			{
				d = (input - lastInput) / timePassed.TotalSeconds;
			}
			lastInput = input;
			lastTime = now;
			calculateD = true;

			double output = twrFactor * (KP * p + KI * i - KD * d);
			if (output > maxOut)
			{
				output = maxOut;
			}
			else if (output < minOut)
			{
				output = minOut;
			}
			return output;
		}

		private static double Clamp(double i)
		{
			if (i > ClampI)
			{
				return ClampI;
			}
			else if (i < -ClampI)
			{
				return -ClampI;
			}
			else
			{
				return i;
			}

		}
	}
}
