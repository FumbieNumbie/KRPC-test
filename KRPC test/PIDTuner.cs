using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KRPC_test
{
	class PIDTuner
	{
		private int flips = 0;
		private int sign = 1;
		public int Sign { get => sign;}

		public int GetFlips(double pidOutput)
		{
			if (sign * pidOutput < 0)
			{
				flips++;
				sign = -sign;
			}
			return flips;
		}

	}
}
