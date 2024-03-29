﻿using System;

namespace KRPC_test
{
	class Output
	{

		public static void Print(object output, int y)
		{
			string stringValue = output.ToString();
			Console.CursorVisible = false;
			Console.SetCursorPosition(1, y);
			if (output.GetType().ToString() == "System.Double" || output.GetType().ToString() == "System.Integer" || output.GetType().ToString() == "System.Float")
			{
				double value = Double.Parse(stringValue);
				if (value < 0)
				{
					Console.Write(" " + Math.Round(value,2) + "  ");
				}
				else
				{
					Console.Write(" " + Math.Round(value,2) + "  ");
				}
			}
			else
			{
				Console.Write(" "+stringValue + "  ");
			}
		}
		public static void Print(object output, int x, int y)
		{
			string stringValue = output.ToString();
			Console.CursorVisible = false;
			Console.SetCursorPosition(10 * x, y);
			if (output.GetType().ToString() == "System.Double" || output.GetType().ToString() == "System.Integer" || output.GetType().ToString() == "System.Float")
			{
				double value = Double.Parse(stringValue);
				if (value < 0)
				{
					Console.Write(" " + Math.Round(value, 2) + "  ");
				}
				else
				{
					Console.Write(" " + Math.Round(value, 2)+ "  ");
				}
			}
			else
			{
				Console.Write(" " + stringValue + "  ");
			}
		}


	}
}
