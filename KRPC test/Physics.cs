using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Numerics;
using System.Threading;
using Tuple3 = System.Tuple<double, double, double>;
namespace KRPC_autopilot
{
    static class Physics
    {
        private static Connection conn = CommonData.conn;
        private static Vessel vessel = CommonData.vessel;
        private static CelestialBody currentBody = vessel.Orbit.Body;
        private static ReferenceFrame bodyFrame = CommonData.bodyFrame;
        private static Stream<float> mass = CommonData.mass;
        private static Stream<float> thrust = CommonData.thrust;
        private static Stream<double> velHStream = CommonData.velHStream;
        private static Stream<double> altitudeStream = CommonData.altitudeStream;
        private static Stream<Tuple<double, double, double>> posStream = CommonData.posStream;
        private static Stream<Tuple<double, double, double>> dragStream = CommonData.dragStream;
        private static Stream<Tuple<double, double, double>> velStream = CommonData.velStream;
        public static Tuple<double, double, double> ImpactPosition { get => impactPos; set => impactPos = value; }
        public static double TimeToImpact { get => timeToImpact; set => timeToImpact = value; }
        public static bool HasImpactPosition { get => hasImpactPosition; set => hasImpactPosition = value; }

        private static Tuple<double, double, double> impactPos = Tuple.Create(.0, .0, .0);
        private static double timeToImpact;
        const float g = 9.80665f;
        private static bool hasImpactPosition = false;



        /// <summary>
        /// Gravitational acceleration at given altitude.
        /// </summary>
        /// <returns></returns>
        public static double AltG()
        {

            double altitude = altitudeStream.Get();
            return (currentBody.GravitationalParameter / Math.Pow(currentBody.EquatorialRadius + altitude, 2));
        }
        /// <summary>
        /// Centrifugal acceleration.
        /// </summary>
        /// <returns></returns>
        public static double CentrAcc()
        {
            double velH = velHStream.Get();
            double altitude = altitudeStream.Get();
            return (velH / (currentBody.EquatorialRadius + altitude));
        }
        /// <summary>
        /// Resulting acceleration recieved from gravity and centrifugal force.
        /// </summary>
        /// <returns></returns>
        public static double DownAcc()
        {
            return AltG() - CentrAcc();
        }


        /// <summary>
        /// Maximal acceleration provided by the ship's engines.
        /// </summary>
        /// <returns></returns>
        public static double MaxA()
        {
            return thrust.Get() / mass.Get();
        }

        public static Tuple<double, double, double> DragDeceleration()
        {
            Tuple<double, double, double> dragDeceleration = dragStream.Get().Mult(1 / mass.Get());
            return dragDeceleration;
        }
        /// <summary>
        /// Predicted velocity based on current velocity, acceleration and drag
        /// </summary>
        private static Tuple<double, double, double> GetNextVelocity(double deltaT)
        {
            Tuple3 velocity = CommonData.velStream.Get();
            Tuple<double, double, double> vel = velocity;
            double gMag = AltG();
            Vector3 norm = Radar.MSLNormal();
            Tuple<double, double, double> dragDeceleration = DragDeceleration();
            Tuple<double, double, double> gDirection = Vectors.ToTuple(-norm);
            Tuple<double, double, double> g = gDirection.Mult(gMag);
            return vel.Plus(dragDeceleration.Plus(g).Mult(deltaT));
        }
        /// <summary>
        /// Position of the vessel while lithobraking.
        /// </summary>
        /// <param name="position">Current position</param>
        /// <param name="velocity">Current velocity</param>
        /// <param name="time">Predition time</param>
        public static void GetImpactPosition(double deltaT)
        {
            Tuple3 position = CommonData.posStream.Get();
            Tuple3 velocity = CommonData.velStream.Get();
            for (double t = 0; t < 50; t += deltaT)
            {
                position = position.Plus(velocity);
                velocity = GetNextVelocity(deltaT);
                var surfacePos = Radar.SurfacePositionAtPosition(position);
                //If next position is below surface, return surface position.
                if (currentBody.AltitudeAtPosition(position, bodyFrame) < currentBody.AltitudeAtPosition(surfacePos, bodyFrame))
                {
                    position = surfacePos;
                    timeToImpact = t;
                    impactPos = position;
                    hasImpactPosition = true;
                    break;
                }
            }


        }

        public static void GetImpPosThread(double deltaT)
        {
            Thread thread = new Thread(() => GetImpactPosition(deltaT));
            thread.Start();
        }
    }
}
