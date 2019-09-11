using KRPC.Client.Services.Drawing;
using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Numerics;
using Tuple3 = System.Tuple<double, double, double>;

namespace KRPC_autopilot
{

    static class RefFrames
    {
        private static ReferenceFrame impactFrame;
        public static ReferenceFrame ImpactFrame { get => impactFrame; set => impactFrame = value; }

        public static Tuple<double, double, double> LaunchPad
        {
            get { return Tuple.Create(159780.589017241, -1018.52068352856, -578410.076172454); }
            //get { return Tuple.Create(0.0, -0.45, -0.29); }

            //get { return Tuple.Create(600074.26, 0.0, 0.0); }

        }


        public static ReferenceFrame CreateHybrid(Connection conn)
        {
            var vessel = conn.SpaceCenter().ActiveVessel;
            return ReferenceFrame.CreateHybrid(
             conn,
             vessel.Orbit.Body.ReferenceFrame,
             vessel.OrbitalReferenceFrame);
        }
        public static ReferenceFrame BodyFrame(Connection conn)
        {
            return conn.SpaceCenter().ActiveVessel.Orbit.Body.ReferenceFrame;
        }

        public static void UpdateImpactFrame()
        {
            Connection conn = CommonData.conn;
            ReferenceFrame bodyFrame = CommonData.bodyFrame;
            Tuple3 position;

            position = Physics.ImpactPosition;
            Output.Print(position, 21);
            Output.Print(DateTime.Now, 22);
            impactFrame = ReferenceFrame.CreateRelative(conn, bodyFrame, position, angularVelocity: position);

        }


        public static ReferenceFrame CreateLaunchPadRef(Connection conn)
        {
            var vessel = conn.SpaceCenter().ActiveVessel;
            var body = vessel.Orbit.Body;

            // Launchpad coordinates
            double landingLatitude = -0.0972069432271304;
            double landingLongitude = -74.557630926517;
            double landingAltitude = 8;

            // Determine landing site reference frame
            // (orientation: x=zenith, y=north, z=east)
            var landingPosition = body.SurfacePosition(
                 landingLatitude, landingLongitude, body.ReferenceFrame);
            var qLong = Tuple.Create(
              0.0,
              Math.Sin(-landingLongitude * 0.5 * Math.PI / 180.0),
              0.0,
              Math.Cos(-landingLongitude * 0.5 * Math.PI / 180.0)
            );
            var qLat = Tuple.Create(
              0.0,
              0.0,
              Math.Sin(landingLatitude * 0.5 * Math.PI / 180.0),
              Math.Cos(landingLatitude * 0.5 * Math.PI / 180.0)
            );
            var landingReferenceFrame =
              ReferenceFrame.CreateRelative(
                 conn,
                 ReferenceFrame.CreateRelative(
                    conn,
                    ReferenceFrame.CreateRelative(
                      conn,
                      body.ReferenceFrame,
                      landingPosition,
                      qLong),
                    Tuple.Create(0.0, 0.0, 0.0),
                    qLat),
                 Tuple.Create(landingAltitude, 0.0, 0.0));

            // Draw axes
            //var zero = Tuple.Create(0.0, 0.0, 0.0);
            //conn.Drawing().AddLine(
            //    zero, Tuple.Create(1.0, 0.0, 0.0), landingReferenceFrame);
            //conn.Drawing().AddLine(
            //	 zero, Tuple.Create(0.0, 1.0, 0.0), landingReferenceFrame);
            //conn.Drawing().AddLine(
            //		 zero, Tuple.Create(0.0, 0.0, 1.0), landingReferenceFrame);

            return landingReferenceFrame;
        }
        public static ReferenceFrame SurfaceBodyHybrid(Connection conn)
        {
            Vessel vessel = conn.SpaceCenter().ActiveVessel;
            ReferenceFrame bodyFrame = vessel.Orbit.Body.ReferenceFrame;
            return ReferenceFrame.CreateHybrid(conn, vessel.SurfaceReferenceFrame, bodyFrame, vessel.SurfaceReferenceFrame);
        }

    }


}