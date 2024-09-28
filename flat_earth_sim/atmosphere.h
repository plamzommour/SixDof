// Atmosphere Definition

namespace AT
{ 
	class atmos
	{
	public:
	static void run_1976_atmos(double, double&, double&, double&);
	static void interp(double, double, double, double, double, double&); 
	static void q_bar(double[], double, double&);
	}; 
}
