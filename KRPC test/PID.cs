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
		private static double p = 0.001;
		private static double i = 0.00;
		private static double d = 0.00;
		private static double minOut = 0;
		private static double lastInput = 0;
		private static bool calculateD = false;
		private static DateTime lastTime = DateTime.Now;
		public static double MaxOut { get; set; }
		public static double SetPoint { get; set; }
		public static double ClampI { get; set; } = 1;
		public static bool Enabled { get; set; }
		public static double KP { get; set; } = 0.0;
		public static double KI { get; set; } = 0.0;
		public static double KD { get; set; } = 0.0;

		////private static ReferenceFrame padRef = RefFrames.CreateLaunchPadRef(conn);
		//private static Connection conn = new Connection("PID");
		//private static Vessel vessel = conn.SpaceCenter().ActiveVessel;
		//private static Stream<double> velVStream = conn.AddStream(() => vessel.Flight(padRef).VerticalSpeed);
		//private static Stream<float> mass = conn.AddStream(() => vessel.Mass);
		//private static Stream<float> thrust = conn.AddStream(() => vessel.AvailableThrust);
		private static double maxA;
		private static double maxTWR;
		private static double velV = 0;

		public static void RunInNewThread(Stream<float> thrust, Stream<float> mass, Stream<double> velocity, Vessel vessel)
		{

			Thread thread = new Thread(() => Update(thrust, mass, velocity, vessel));
			thread.Start();


		}

		


		public static Tuple<double, double, double> GetCoefficients()
		{
			return new Tuple<double, double, double>(KP, KI, KD);
		}

		public static void Update(Stream<float> thrust, Stream<float> mass,Stream<double> velocity, Vessel vessel)
		{

			while (true)
			{
				if (Enabled == true)
				{
					maxA = thrust.Get() / mass.Get();
					maxTWR = maxA / 9.81;
					MaxOut = maxTWR;
					velV = velocity.Get();
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
					if (output > MaxOut)
					{
						output = MaxOut;
					}
					else if (output < minOut)
					{
						output = minOut;
					}

					vessel.Control.Throttle = (float)(output / maxTWR);
				}
				Thread.Sleep(50);
				//Output.Print(velV, 7, 3);
				//Output.Print(output);
			}
		}
		//public static double Update(double input, double maxAcc)
		//{
		//	DateTime now = DateTime.Now;
		//	TimeSpan timePassed = now - lastTime;
		//	if (timePassed == null || timePassed.TotalSeconds == 0)
		//	{
		//		timePassed = new TimeSpan(0, 0, 1);
		//	}

		//	double twrFactor = 4.87 / maxAcc;
		//	double error = (SetPoint - input);
		//	p = error;
		//	i += error * timePassed.TotalSeconds;
		//	i = Clamp(i);
		//	if (calculateD & KD != 0)
		//	{
		//		d = (input - lastInput) / timePassed.TotalSeconds;
		//	}
		//	lastInput = input;
		//	lastTime = now;
		//	calculateD = true;

		//	double output = twrFactor * (KP * p + KI * i - KD * d);
		//	if (output > maxOut)
		//	{
		//		output = maxOut;
		//	}
		//	else if (output < minOut)
		//	{
		//		output = minOut;
		//	}
		//	return output;
		//}

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
