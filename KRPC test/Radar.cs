using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using KRPC.SpaceCenter;
using System.Numerics;
using System.Threading;
using Tuple3 = System.Tuple<double, double, double>;
using KRPC.Client.Services.Drawing;
using System.Collections.Generic;

namespace KRPC_autopilot
{
    static class Radar
    {

        private static Connection conn = CommonData.conn;
        private static Vessel vessel = CommonData.vessel;
        private static CelestialBody body = vessel.Orbit.Body;
        private static readonly ReferenceFrame bodyFrame = body.ReferenceFrame;
        private static Stream<Tuple<double, double, double>> velStream = CommonData.velStream;
        private static Stream<double> velVStream = CommonData.velVStream;
        private static Stream<double> altStream = CommonData.altitudeStream;
        private static Stream<Tuple<double, double, double>> posStream = CommonData.posStream;



        /// <summary>
        /// Calculate time before the impact.
        /// </summary>
        /// <returns>Time in seconds</returns>
        public static float ImpactTime()
        {
            double velV = velVStream.Get();
            double altitude = altStream.Get();
            double downAcc = Physics.DownAcc();
            float time = (float)((velV + Math.Sqrt(velV * velV + 2 * altitude * downAcc)) / Math.Max(downAcc, 0.001));
            return time;
        }



        /// <summary>
        /// Altitude (terrain)
        /// </summary>
        /// <returns></returns>
        public static double RealAltitude()
        {
            return altStream.Get() - body.AltitudeAtPosition(Physics.ImpactPosition, bodyFrame);
        }


        public static Tuple<double, double, double> SurfacePositionAtPosition(Tuple<double, double, double> position)
        {
            Tuple<double, double> latLon = LatLonAtPosition(position);
            return body.SurfacePosition(latLon.Item1, latLon.Item2, bodyFrame);
        }

        /// <summary>
        /// Calculate time before the impact with respect of surface elevation.
        /// </summary>
        /// <param name="realAltitude"></param>
        /// <returns>Time in seconds</returns>
        public static double RealImpactTime()
        {
            double velV = velVStream.Get();
            double downAcc = Physics.DownAcc();
            return (velV + Math.Sqrt(velV * velV + 2 * RealAltitude() * downAcc)) / Math.Max(downAcc, 0.001);
        }
        /// <summary>
        /// Calculates a distance needed to slow down
        /// </summary>
        /// <returns></returns>
        public static double BurnDistance()
        {
            Vector3 velocity = Vectors.ToVector(velStream.Get()) + Vectors.ToVector(Physics.DragDeceleration());
            return velocity.Length() * velocity.Length() / Physics.MaxA();
        }
        /// <summary>
        /// Calculates an impact position based on velocity and maximum acceleration. 
        /// </summary>
        /// <returns></returns>
        public static Vector3 ImpactPos()
        {
            Vector3 vesselPosition = posStream.Get().ToVector();
            Vector3 velocity = velStream.Get().ToVector();
            return vesselPosition + velocity * ImpactTime();
        }
        /// <summary>
        /// Gets a surface position at an impact location.
        /// </summary>
        /// <returns>Returns a vector of the real landing position</returns>
        private static Vector3 RealImpactPos()
        {
            Vector3 newPosition = ImpactPos();
            double latitude = body.LatitudeAtPosition(Vectors.ToTuple(newPosition), bodyFrame);
            double longitude = body.LongitudeAtPosition(Vectors.ToTuple(newPosition), bodyFrame);
            return Vectors.ToVector(body.SurfacePosition(latitude, longitude, bodyFrame));

        }
        public static Tuple<double, double> LatLonAtPosition(Tuple<double, double, double> position)
        {
            double latitude = body.LatitudeAtPosition(position, body.ReferenceFrame);
            double longitude = body.LongitudeAtPosition(position, body.ReferenceFrame);

            return Tuple.Create(latitude, longitude);
        }
        internal static Vector3 MSLNormal()
        {
            Tuple3 pos = CommonData.posStream.Get();
            double latitude = body.LatitudeAtPosition(pos, bodyFrame);
            double longitude = body.LongitudeAtPosition(pos, bodyFrame);
            Tuple<double, double, double> landingSpotMSL = body.MSLPosition(latitude, longitude, body.ReferenceFrame);
            //Spot 1
            latitude += 1 / 60 / 60 * 20;
            longitude += 1 / 60 / 60 * 20;
            Vector3 spotOnePositionMSL = Vectors.ToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
            //Spot 2										  
            latitude -= 1.4 / 60 / 60 * 20;
            longitude += 0.3 / 60 / 60 * 20;
            Vector3 spotTwoPositionMSL = Vectors.ToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
            //Spot 3										
            latitude += 0.4 / 60 / 60 * 20;
            longitude -= 1.4 / 60 / 60 * 20;
            Vector3 spotThreePositionMSL = Vectors.ToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));

            Vector3 v1MSL = spotTwoPositionMSL - spotOnePositionMSL;
            Vector3 v2MSL = spotThreePositionMSL - spotOnePositionMSL;


            Vector3 n2 = Vector3.Cross(v1MSL, v2MSL).Normalise(); // normal to the sea surface
            return n2;
        }


        public static Vector3 SurfaceNormal1(Tuple<double, double, double> position)
        {
            Tuple<double, double> latLon = LatLonAtPosition(position);
            double latitude = latLon.Item1;
            double longitude = latLon.Item2;
            //predicted landing spot with regard of surface elevation
            //Tuple<double, double, double> landingSpot = body.SurfacePosition(latitude, longitude, body.ReferenceFrame);
            double spread = 3; //How far away the measures will be taken. Affects precision.
            //Spot 1
            latitude = latLon.Item1;
            longitude = latLon.Item2 + Math.Sqrt(3) / 4 / 60 / 60 * spread;
            Vector3 spotOnePosition = Vectors.ToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
            //Spot 2										  
            latitude = latLon.Item1 - 0.5 / 60 / 60 * spread;
            longitude = latLon.Item2 - Math.Sqrt(3) / 4 / 60 / 60 * spread;
            Vector3 spotTwoPosition = Vectors.ToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
            //Spot 3										
            latitude = latLon.Item1 + 0.5 / 60 / 60 * spread;
            longitude = latLon.Item2 - Math.Sqrt(3) / 4 / 60 / 60 * spread;
            Vector3 spotThreePosition = Vectors.ToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
            //These vectors represent the sides of an equilateral triangle. The first two are for setting a normal. The third vector is only needed for visualization.
            Vector3 v1 = spotTwoPosition - spotOnePosition;
            Vector3 v2 = spotThreePosition - spotOnePosition;
            Vector3 v3 = spotThreePosition - spotTwoPosition;

            //Visual.Draw(conn, spotOnePosition, spotOnePosition + v1, bodyFrame);
            //Visual.Draw(conn, spotOnePosition, spotOnePosition + v2, bodyFrame);
            //Visual.Draw(conn, spotTwoPosition, spotTwoPosition + v3, bodyFrame);

            return Vector3.Cross(v1, v2).Normalise(); //normal to the surface that v1 and v2 lie on.
        }

        private static double ImpactLocSurfaceHeight()
        {
            Vector3 newPosition = ImpactPos();
            double latitude = body.LatitudeAtPosition(Vectors.ToTuple(newPosition), bodyFrame);
            double longitude = body.LongitudeAtPosition(Vectors.ToTuple(newPosition), bodyFrame);
            return body.SurfaceHeight(latitude, longitude);
        }

        /// <summary>
        /// A normal to the surface directly under the vessel.
        /// </summary>
        /// <returns></returns>
        public static Vector3 SurfaceNormal()
        {
            Tuple3 position = Physics.ImpactPosition;
            Tuple<double, double> latLon = LatLonAtPosition(position);
            Tuple<double, double, double> surfacePos = body.SurfacePosition(latLon.Item1, latLon.Item2, bodyFrame);
            double latitude;
            double longitude;
            //predicted landing spot with regard of surface elevation
            //Tuple<double, double, double> landingSpot = body.SurfacePosition(latitude, longitude, body.ReferenceFrame);
            double spread = 3; //How far away the measures will be taken. Affects precision.
            //Spot 1
            latitude = latLon.Item1;
            longitude = latLon.Item2 + Math.Sqrt(3) / 4 / 60 / 60 * spread;
            Vector3 spotOnePosition = Vectors.ToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
            //Spot 2										  
            latitude = latLon.Item1 - 0.5 / 60 / 60 * spread;
            longitude = latLon.Item2 - Math.Sqrt(3) / 4 / 60 / 60 * spread;
            Vector3 spotTwoPosition = Vectors.ToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));
            //Spot 3										
            latitude = latLon.Item1 + 0.5 / 60 / 60 * spread;
            longitude = latLon.Item2 - Math.Sqrt(3) / 4 / 60 / 60 * spread;
            Vector3 spotThreePosition = Vectors.ToVector(body.SurfacePosition(latitude, longitude, body.ReferenceFrame));

            //These vectors represent the sides of an equilateral triangle. The first two are for setting a normal. The third vector is only needed for visualization.
            Vector3 v1 = spotTwoPosition - spotOnePosition;
            Vector3 v2 = spotThreePosition - spotOnePosition;
            //Vector3 v3 = spotThreePosition - spotTwoPosition;

            
            //Visual.Draw(spotOnePosition, spotOnePosition + v1, bodyFrame);
            //Visual.Draw(spotOnePosition, spotOnePosition + v2, bodyFrame);
            //Visual.Draw(spotTwoPosition, spotTwoPosition + v3, bodyFrame);

            return Vector3.Cross(v1, v2).Normalise(); //normal to the surface that v1 and v2 lie on.
        }
        public static double Slope(Vector3 surfaceNormal, Tuple<double, double, double> position)
        {
            Vector3 n1 = surfaceNormal;
            Tuple<double, double> latLon = LatLonAtPosition(position);
            double latitude = latLon.Item1;
            double longitude = latLon.Item2;
            //Same as above but for mean Sea level
            Tuple<double, double, double> landingSpotMSL = body.MSLPosition(latitude, longitude, body.ReferenceFrame);
            //Spot 1
            latitude += 1 / 60 / 60 * 20;
            longitude += 1 / 60 / 60 * 20;
            Vector3 spotOnePositionMSL = Vectors.ToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
            //Spot 2										  
            latitude -= 1.4 / 60 / 60 * 20;
            longitude += 0.3 / 60 / 60 * 20;
            Vector3 spotTwoPositionMSL = Vectors.ToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));
            //Spot 3										
            latitude += 0.4 / 60 / 60 * 20;
            longitude -= 1.4 / 60 / 60 * 20;
            Vector3 spotThreePositionMSL = Vectors.ToVector(body.MSLPosition(latitude, longitude, body.ReferenceFrame));

            Vector3 v1MSL = spotTwoPositionMSL - spotOnePositionMSL;
            Vector3 v2MSL = spotThreePositionMSL - spotOnePositionMSL;


            Vector3 n2 = Vector3.Cross(v1MSL, v2MSL); // normal to the sea surface
            return Vectors.Angle(n1, n2);
        }



        private static void DecideWhenToBurn()
        {
            Vector3 distanceToImpact = 10 * Vector3.UnitX;
            double distanceToBurn = 0;
            Vector3 velocity;
            Vector3 vesselPosition;

            while (distanceToBurn < distanceToImpact.Length())
            {
                Tuple3 velTuple = velStream.Get();
                velocity = Vectors.ToVector(velTuple);
                var position = posStream.Get();
                vesselPosition = Vectors.ToVector(position);
                distanceToBurn = BurnDistance();
                var impactPosition = Physics.ImpactPosition.ToVector();
                distanceToImpact = vesselPosition - impactPosition;
            }
            OnDecision?.Invoke();


        }

        public delegate void DecisionHandler();
        public static event DecisionHandler OnDecision;
        /// <summary>
        /// Create a new thread and run DicideWhenToBurn() in it.
        /// </summary>
        public static void RunDeciderInNewThread()
        {
            Thread thread = new Thread(() => DecideWhenToBurn());
            thread.Start();

        }


    }
}
