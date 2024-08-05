// Header file for integration routine - Forward Euler

namespace FE
{ 
	class integrator
	{
	public:
	static void euler_integrate_6(double[], double[], double); 
	static void euler_integrate_3(double[], double[], double); 
	static void euler_integrate_4(double[], double[], double);
	}; 
}
