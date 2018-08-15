using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KRPC_test
{
	class kosPid
	{
		private double kP = 0.0;
		private double kI = 0.0;
		private double kD = 0.0;
		private double p = 0.001;
		private double i = 0.00;
		private double d = 0.00;
		private double minOut = 0;
		private double maxOut = 1;
		private double pTerm;
		private double iTerm;
		private double dTerm;
		private DateTime lastTime = DateTime.Now;
		private double lastMeasure;

		public double SetPoint { get; set; } = 0;
		public double ClampI { get; set; } = 1;

		public kosPid(double _p = 0.0, double _i = 0.0, double _d = 0.00)
		{
			kP = _p;
			kI = _i;
			kD = _d;
		}
		public Tuple<double, double, double> GetCoefficients()
		{
			return new Tuple<double, double, double>(kP, kI, kD);
		}

		public double Update(double input)
		{
			
			double error = SetPoint - input;
			pTerm = error * kP;
			iTerm = 0;
			dTerm = 0;
			DateTime now = DateTime.Now;
			TimeSpan timePassed = now - lastTime;
			if (timePassed.Seconds > 0)
			{
				lastTime = now;
				if (kI != 0)
				{
					i += error*timePassed.TotalSeconds;
					iTerm = i * kI;
				}
			}
		}
	}
}