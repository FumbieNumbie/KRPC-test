using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using Tuple3 = System.Tuple<double, double, double>;

namespace KRPC_autopilot
{

    public class CommonData
    {
        public static Connection conn = new Connection("Main");
        public static Vessel vessel = conn.SpaceCenter().ActiveVessel;
        public static ReferenceFrame hybridFrame = RefFrames.CreateHybrid(conn);
        public static ReferenceFrame bodyFrame = RefFrames.BodyFrame(conn);
        public static ReferenceFrame surfFrame = vessel.SurfaceReferenceFrame;
        public static ReferenceFrame surfVelFrame = vessel.SurfaceVelocityReferenceFrame;
        private static ReferenceFrame currentFrame = bodyFrame;
        //var launchPad = RefFrames.LaunchPad;
        public static Stream<double> velVStream = conn.AddStream(() => vessel.Flight(currentFrame).VerticalSpeed);
        public static Stream<Tuple<double, double, double>> posStream = conn.AddStream(() => vessel.Position(bodyFrame));
        public static Stream<Tuple<double, double, double>> posStreamSF = conn.AddStream(() => vessel.Position(bodyFrame));
        public static Stream<Tuple<double, double, double>> velStream = conn.AddStream(() => vessel.Flight(bodyFrame).Velocity);
        public static Stream<Tuple3> velStreamSF = conn.AddStream(() => vessel.Flight(surfFrame).Velocity);
        public static Stream<float> mass = conn.AddStream(() => vessel.Mass);
        public static Stream<float> thrust = conn.AddStream(() => vessel.AvailableThrust);
        public static CelestialBody body = vessel.Orbit.Body;
        public static Stream<double> velHStream = conn.AddStream(() => vessel.Flight(bodyFrame).HorizontalSpeed);
        public static Stream<double> altitudeStream = conn.AddStream(() => vessel.Flight(bodyFrame).MeanAltitude);
        public static Stream<Tuple<double, double, double>> dragStream = conn.AddStream(() => vessel.Flight(bodyFrame).Drag);
        

        

        public CommonData()
        {
            double velV = velVStream.Get();
            Tuple3 position = posStream.Get();
            Tuple3 velocity = velStream.Get();
            Tuple3 drag = dragStream.Get();
        }
    }


}
