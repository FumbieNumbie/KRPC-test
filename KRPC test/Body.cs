using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KRPC_autopilot
{
    class Body
    {

        private DateTime lastT = DateTime.Now;
        private DateTime lastT1 = DateTime.Now;

        public double Drop()
        {
            TimeSpan timePassed = DateTime.Now - lastT;
            lastT = DateTime.Now;
            return -9.806 * timePassed.TotalSeconds;
        }
        public double ThrustUp()
        {
            TimeSpan timePassed = DateTime.Now - lastT1;
            lastT1 = DateTime.Now;
            return 15 * timePassed.TotalSeconds;
        }
    }
}
