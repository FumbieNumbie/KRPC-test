using KRPC.Client;
using KRPC.Client.Services.Drawing;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.Numerics;
using Tuple3 = System.Tuple<double, double, double>;

namespace KRPC_autopilot
{
    public partial class Visual
    {

        public static void Draw(Tuple<double, double, double> start, Vector3 vector, ReferenceFrame frame)
        {
            Connection conn = CommonData.conn;
            var t1 = Vectors.ToTuple(vector);
            conn.Drawing().AddLine(start, Vectors.MultiplyByNumber(t1, 1), frame);
        }
        public static void Draw(Vector3 start, Vector3 vector, ReferenceFrame frame)
        {
            Connection conn = CommonData.conn;

            var zero = start.ToTuple();
            var tuple = vector.ToTuple();
            conn.Drawing().AddLine(zero, tuple, frame);
        }
        public static void Draw(Vector3 vector, ReferenceFrame frame)
        {
            Connection conn = CommonData.conn;
            
            var zero = Tuple.Create(0.0, 0.0, 0.0);
            var t1 = Vectors.ToTuple(vector);
            Line line = conn.Drawing().AddLine(zero, t1, frame);
            line.Color = Tuple.Create(0.7, .0, 0.40);
            line.Thickness = 2;
        }
        public static void DrawDirection(Vector3 vector, ReferenceFrame frame)
        {
            Connection conn = CommonData.conn;
            var t1 = Vectors.ToTuple(vector);
            conn.Drawing().AddDirection(t1, frame);
        }
        public static void DrawDirection(Tuple3 tuple, ReferenceFrame frame)
        {
            Connection conn = CommonData.conn;
            var t1 = tuple;
            conn.Drawing().AddDirection(t1, frame);
            //System.Threading.Thread.Sleep(50);
        }
        public static void DrawLandingMarker(Tuple3 tuple)
        {
            Connection conn = CommonData.conn;
            Tuple3 oa = tuple.RotateByNinety("x");
            Tuple3 ob = tuple.RotateByNinety("y");
            Tuple3 oc = tuple.RotateByNinety("z");
            Tuple3 ab = oa.Minus(ob);
            Tuple3 bc = ob.Minus(oc);
            Tuple3 ca = oc.Minus(oa);
            List<Tuple3> vertices = new List<Tuple3>() { ab, bc, ca };
            conn.Drawing().AddPolygon(vertices, RefFrames.ImpactFrame);
        }


    }
}
