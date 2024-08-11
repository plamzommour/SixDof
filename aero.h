// Aero.h header file 

namespace ARO
{
	class aero
	{
	public:
	static void rotate_B2S(double[], double[], double[]); 
	static void rotate_S2B(double[], double[], double[]); 
	static void rotate_B2W(double[], double[], double[]); 
	static void rotate_W2B(double[], double[], double[]); 
	static void drag_force_wind_x(double, double[], double&);
	static void lift_force_wind_z(double, double[], double[], double&);
	static void side_force_wind_y(double, double[], double[], double&);
	static void moments_body(double[], double, double[], double, double, double, double[]); 
	static void test_vehicle_brick(double[]); 
	static void test_vehicle_cessna(double[]); 
	static void alpha_dot(double, double, double, double&);
	static void aero_driver(double[], double[], double, double, double, double[], double[], double[], double[], double[], double&, double&);
	};
	
}
