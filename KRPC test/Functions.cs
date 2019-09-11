using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using KRPC.Client.Services;
using Tuple3 = System.Tuple<double, double, double>;

namespace KRPC_autopilot
{
	public static partial class Functions
    {
        public static void Test()
        {
            Connection conn = CommonData.conn;
            Vessel vessel = CommonData.vessel;
            ReferenceFrame hybridFrame = CommonData.hybridFrame;
            ReferenceFrame bodyFrame = CommonData.bodyFrame;
            ReferenceFrame surfFrame = vessel.SurfaceReferenceFrame;
            ReferenceFrame surfVelFrame = vessel.SurfaceVelocityReferenceFrame;
            //var launchPad = RefFrames.LaunchPad;
            ReferenceFrame currentFrame = bodyFrame;
            Stream<double> velVStream = CommonData.velVStream;
            Stream<Tuple<double, double, double>> posStream = CommonData.posStream;
            Stream<Tuple<double, double, double>> velStream = CommonData.velStream;
            Stream<float> mass = CommonData.mass;
            Stream<float> thrust = CommonData.thrust;
            CelestialBody body = vessel.Orbit.Body;
            Stream<double> velHStream = CommonData.velHStream; ;
            Stream<double> altitudeStream = CommonData.altitudeStream;
            Stream<Tuple<double, double, double>> dragStream = CommonData.dragStream;

            PID.SetPoint = 0;
            PID.Enabled = false;
            PID.RunInNewThread(thrust, mass, velVStream, vessel);
            CommonData data = new CommonData();

            //Control section
            vessel.Control.SAS = false;
            vessel.Control.RCS = true;
            vessel.AutoPilot.Engage();
            vessel.AutoPilot.ReferenceFrame = bodyFrame;
            Tuple<double, double, double> steeringDir = Tuple.Create(.0, .0, .0);
            PID.Enabled = true;
            Physics.GetImpPosThread(1);
            Tuple3 up = Radar.MSLNormal().ToTuple();
            DateTime lastTime = DateTime.Now;
            vessel.AutoPilot.TargetDirection = up;
            while (true)
            {
                RefFrames.UpdateImpactFrame();
                var surfaceNormal = Radar.SurfaceNormal();
                if ((DateTime.Now-lastTime).TotalMilliseconds > 1000)
                {
                    conn.Drawing().Clear();
                    //Visual.Draw(conn, 10 * surfaceNormal, RefFrames.ImpactFrame);
                    //Visual.DrawLandingMarker(surfaceNormal.ToTuple());
                    Visual.Draw(10*surfaceNormal, RefFrames.ImpactFrame);

                }
                Physics.GetImpPosThread(1);

                Output.Print(Physics.ImpactPosition, 19);
                System.Threading.Thread.Sleep(25);

            }

        }
        public static void TestVelocityVector()
        {
            Connection conn = CommonData.conn;
            Vessel vessel = CommonData.vessel;
            ReferenceFrame hybridFrame = CommonData.hybridFrame;
            ReferenceFrame bodyFrame = CommonData.bodyFrame;
            ReferenceFrame surfFrame = vessel.SurfaceReferenceFrame;
            ReferenceFrame surfVelFrame = vessel.SurfaceVelocityReferenceFrame;
            while (true)
            {
                Tuple3 position = CommonData.posStream.Get();
                Tuple3 velocity = CommonData.velStream.Get();
                Tuple3 retrograde = velocity.Normalise().Mult(-1);
                Tuple3 up = Vector3.Normalize(Radar.MSLNormal()).ToTuple();

                Tuple3 deviation = up.Minus(retrograde);
                float alt = (float)Radar.RealAltitude();
                float k = Math.Max(0.01f, Math.Min(1, alt / 100));

                Visual.Draw(position.ToVector(), position.ToVector() + deviation.ToVector(), bodyFrame);
                Output.Print(deviation.Length(), 13);
                Output.Print(position.Length(), 14);
                Output.Print(velocity.Length(), 15);
                conn.Drawing().Clear();
            }


        }

    }
}
