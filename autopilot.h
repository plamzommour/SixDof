// Header file for the Autopilot Routine 

namespace AP
{ 
	class autopilot
	{
	public:
	static void vertical_channel(double[], double[], double[], double, double&); 
	static void lateral_channel(double[], double[], double[], double, double&, double&); 
	static void throttle_channel(double[], double[], double[], double, double&); 
	}; 
}
