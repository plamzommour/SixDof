// This is a file that sets up all the functions for quaternion operations

namespace Q
{ 
	class quat
	{
	public:
	static void init_quaternion(double[], double[]); 
        static void quaternion_form_dcm(double[], double[3][3]); 
	static void rotate_I2B(double[], double[3][3], double[]); 
	static void rotate_B2I(double[], double[3][3], double[]); 
	static void propogate_quaternion(double[], double[], double[]); 
	static void find_eulers(double[], double[]); 
	static void omega_2_euler_rates(double[], double[], double[]);
	static void quat_mult(double[], double[], double[]);
	static void normalize_quaternion(double []); 
	}; 
}
