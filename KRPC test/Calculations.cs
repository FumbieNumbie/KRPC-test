using System;
using System.Collections.Generic;
using KRPC.Client;
using KRPC.Client.Services.SpaceCenter;


namespace KRPC_test
{
	public class Readings
	{
		const float g = 9.80665f;
		private static readonly Connection conn = new Connection("Readings");
		private static readonly Vessel vessel = conn.SpaceCenter().ActiveVessel;
		private static readonly Resources resources = vessel.Resources;
		public static readonly	Stream<int> currentStage = conn.AddStream(() => vessel.Control.CurrentStage);


		/// <summary>
		/// Gets fuel
		/// </summary>
		public static float GetFuelMass()
		{

			Resources resources = vessel.ResourcesInDecoupleStage(currentStage.Get(), cumulative: false);
			float sum = 0;
			List<Resource> reses = new List<Resource>(resources.All);
			foreach (var item in reses)
			{
				sum += item.Amount * item.Density;
			}
			return sum;
		}

		/// <summary>
		/// Stages on flameout
		/// </summary>
		public static void Stage(Vessel vessel)
		{

			foreach (var engine in vessel.Parts.Engines)
			{
				if (engine.HasFuel == false)
				{
					vessel.Control.Throttle = 0;
					vessel.Control.ActivateNextStage();
					vessel.Control.Throttle = 1;

					break;
				}
			}

		}
		/// <summary>
		/// Calculates time that maneuver execution will take
		/// </summary>
		public static double GetBurnTime(Vessel vessel, Node node)
		{
			double F = vessel.AvailableThrust;
			double Isp = vessel.SpecificImpulse;
			double m0 = vessel.Mass;
			double m1 = m0 / Math.Exp(node.DeltaV / Isp); //mass after burn
			double flowRate = F / Isp / g;
			double burnTime = (m0 - m1) / flowRate;
			return burnTime;
		}

	
		/// <summary>
		/// Calculates DV in stage
		/// </summary>
		public static float StageDV(float mass, float fuelmass, float isp)
		{
			if (mass - fuelmass > 0) // why would it ever be false?
			{
				return g * isp * (float)Math.Log(mass / (mass - fuelmass));
			}
			else
			{
				return 0;
			}
		}

		/// <summary>
		/// Adds node
		/// </summary>
		/// <param name="vessel">Your vessel</param>
		/// <param name="ut">A stream of Universal Time</param>
		public static void AddNode(Vessel vessel, Stream<double> ut)
		{
			// Plan circularization burn (using vis-viva equation)
			double mu = vessel.Orbit.Body.GravitationalParameter;
			double r = vessel.Orbit.Apoapsis;
			double a1 = vessel.Orbit.SemiMajorAxis;
			double a2 = r;
			double v1 = Math.Sqrt(mu * ((2.0 / r) - (1.0 / a1)));
			double v2 = Math.Sqrt(mu * ((2.0 / r) - (1.0 / a2)));
			double deltaV = v2 - v1;
			var node = vessel.Control.AddNode(
					ut.Get() + vessel.Orbit.TimeToApoapsis, prograde: (float)deltaV);



		}
		/// <summary>
		/// This method warps to maneuver node that has to be passed as a parameter
		/// </summary>
		/// <param name="conn">Program's connection object</param>
		/// <param name="altitude">This is required to decide what kind of warp should be used</param>
		/// <param name="node">Maneuver node you want to warp to</param>
		/// <param name="burnTime">An amount of time it will take to execute the node</param>
		/// <param name="burnUT">Universal time at the point timewarp should end</param>
		public static void WarpToNode(Connection conn, Stream<double> altitude, Node node, double burnTime, double burnUT)
		{
			while (node.TimeTo > (burnTime / 2 + 12))
			{
				Console.Write(Convert.ToString(node.TimeTo) + "\r");

				if (altitude.Get() > 70000)
				{
					conn.SpaceCenter().WarpTo(burnUT - 12);

				}
				else
				{
					conn.SpaceCenter().PhysicsWarpFactor = 3;
				}
			}
			conn.SpaceCenter().PhysicsWarpFactor = 0;
		}

		/// <summary>
		/// Executes the next node if present
		/// </summary>
		/// <param name="vessel">your space poker</param>
		public static void ExeNode(Vessel vessel)
		{
			float maxAcc = vessel.MaxThrust / vessel.Mass;
			List<Node> nodes = new List<Node>(vessel.Control.Nodes);
			if (nodes.Count > 0)
			{
				var node = nodes[0];

				while (node.RemainingDeltaV > 1)
				{
					//if (node.RemainingDeltaV > 10)
					//{
					vessel.Control.Throttle = Math.Min((float)node.RemainingDeltaV / maxAcc, 1);
					//}

					//else
					//{
					//    vessel.AutoPilot.Wait();
					//    vessel.Control.Throttle = 0.05f;
					//}

				}
				//vessel.Control.Throttle = 0;
			}
		}
	}
}
