using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
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
        public static class Landing
        {
            /// <summary>
            /// Land at a random place along the trajectory as efficiently as possible.
            /// </summary>
            public static void Land()
            {
                Connection conn = CommonData.conn;
                Vessel vessel = CommonData.vessel;
                ReferenceFrame bodyFrame = CommonData.bodyFrame;
                ReferenceFrame surfFrame = CommonData.surfFrame;
                ReferenceFrame surfVelFrame = CommonData.surfVelFrame;
                //var launchPad = RefFrames.LaunchPad;
                ReferenceFrame currentFrame = bodyFrame;
                Stream<double> velVStream = CommonData.velVStream;
                Stream<Tuple<double, double, double>> posStream = CommonData.posStream;
                Stream<Tuple<double, double, double>> velStream = CommonData.velStream;
                Stream<float> mass = CommonData.mass;
                Stream<float> thrust = CommonData.thrust;
                CelestialBody body = vessel.Orbit.Body;


                CommonData data = new CommonData();

                //PID section
                PID.SetPoint = -1000;
                PID.Enabled = false;
                PID.RunInNewThread(thrust, mass, velVStream, vessel);

                //Control section
                vessel.Control.SAS = false;
                vessel.Control.RCS = true;
                vessel.AutoPilot.Engage();
                vessel.AutoPilot.ReferenceFrame = bodyFrame;
                vessel.AutoPilot.TargetDirection = Vectors.ToTuple(Vector3.UnitX);
                Tuple<double, double, double> steeringDir = Tuple.Create(.0, .0, .0);

            //Actual execution
            Start:
                PID.Enabled = true;
                ReferenceFrame impactFrame = RefFrames.ImpactFrame;
                DateTime lastTime = DateTime.Now;
                DoLanding(vessel); // Set vertical speed to -5
                while (vessel.Situation.ToString() != "Landed")
                {
                    //Manage steering
                    steeringDir = HoverslamSteering();
                    vessel.AutoPilot.TargetDirection = steeringDir;
                    //Define position variables
                    var position = posStream.Get();
                    var velocity = velStream.Get();
                    Vector3 posVector = Vectors.ToVector(position);
                    Physics.GetImpPosThread(1);
                    //Tuple3 landingPos = Physics.GetImpactPosition(position,velocity,Radar.ImpactTime());
                    Tuple3 landingPos = Physics.ImpactPosition;
                    Vector3 landingSpot = Vectors.ToVector(landingPos);
                    //Vector3 landingSpot = ((posVector - Vectors.ToVector(landingPos) ) * 2 + posVector);
                    //Slope
                    var surfaceNormal = Radar.SurfaceNormal();
                    var slope = Radar.Slope(surfaceNormal, position);
                    //Visualize landing spot
                    Vector3 norm = Radar.MSLNormal();

                    //Vector3 landingSpot = VectorMath.TupleToVector(body.SurfacePosition(predictedLatLon.Item1, predictedLatLon.Item2, bodyFrame));
                    //Vector3 landingSpot = Vectors.ToVector(landingPos);
                    Tuple<double, double> currentLatLon = Tuple.Create(body.LatitudeAtPosition(position, bodyFrame), body.LongitudeAtPosition(position, bodyFrame));
                    RefFrames.UpdateImpactFrame();
                    if ((DateTime.Now - lastTime).TotalMilliseconds > 1000)
                    {
                        conn.Drawing().Clear();
                        Visual.Draw(10 * surfaceNormal, RefFrames.ImpactFrame);
                    }
                    //Output
                    Output.Print("Slope: " + slope + "  ", 1);
                    Output.Print("Time till impact: " + Physics.TimeToImpact + "  ", 2);
                    Output.Print("My method: " + landingPos + "  ", 3);
                    Output.Print("Trajectories: " + landingPos + "  ", 4);
                    Output.Print("Altitude: " + Radar.RealAltitude() + "  ", 5);
                    Output.Print("Velocity: " + velocity.Length() + "  ", 6);
                    //Save fuel and run hoverslam again if the first time was an overshot.
                    if (Radar.RealAltitude() > 55 && (velocity.Length() < 6))
                    {
                        PID.SetPoint = -35;
                        PID.Enabled = false;
                        goto Start;

                    }
                    //Console.Clear();
                    //System.Threading.Thread.Sleep(50);
                    //conn.Drawing().Clear();

                }
                Console.Clear();
                vessel.Control.SAS = true;
                PID.SetPoint = -20;
                System.Threading.Thread.Sleep(100);
                PID.Enabled = false;
                vessel.Control.Throttle = 0;
                vessel.AutoPilot.Disengage();
                Output.Print("Done", 1);

            }

            /// <summary>
            /// Steering during landing.
            /// </summary>
            private static Tuple<double, double, double> HoverslamSteering()
            {

                Connection conn = CommonData.conn;
                ReferenceFrame bodyF = CommonData.bodyFrame;
                //ReferenceFrame sFrame = vessel.SurfaceReferenceFrame;
                Tuple3 position = CommonData.posStream.Get();
                Tuple3 velocity = CommonData.velStream.Get();
                double velH = CommonData.velHStream.Get();
                Tuple3 retrograde = velocity.Normalise().Mult(-1);
                Tuple3 steeringDirVec;
                Tuple3 up = Radar.MSLNormal().ToTuple();

                float alt = (float)Radar.RealAltitude();
                double k = Math.Max(0.01f, Math.Min(1, velH / 5));
                //Visual.DrawDirection(conn, retrograde.ToVector(), CommonData.bodyFrame);

                //To do: Make sure it follows velocity vector
                steeringDirVec = up.Plus(retrograde.Mult(k));

                Output.Print(k, 13);
                Output.Print(position, 14);
                Output.Print(velocity, 15);
                return steeringDirVec;

            }
            /// <summary>
            /// Subscribe to burn decider event.
            /// </summary>
            private static void DoLanding(Vessel vessel)
            {
                Radar.RunDeciderInNewThread();
                Radar.OnDecision += () => { PID.SetPoint = -5; vessel.Control.Gear = true; };
            }
        }

    }
}
