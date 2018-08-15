using System;

namespace KRPC_test
{
	class Output
	{
		public static void Line(object output, int y, int x = 1)
		{
			Console.CursorVisible = false;
			Console.SetCursorPosition(x, y);
			Console.Write(output+"   ");
		}
	}	
}
