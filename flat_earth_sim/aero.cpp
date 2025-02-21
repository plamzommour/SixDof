/* This is the big one: Aero

This file takes care of all the aero-related tasks. 

Transformations to/from stability frame and wind axes are performed using DCMs for ease

Aero forces are in WIND axes as follows - [Drag; Sideforce; Lift] == [X_wind force, Y_wind force, Z_wind force], 
and are rotated back to BODY frame for usage in EoM. 

Included for completeness is stability axis, which will come in handy for moments

Propulsion will also be included here. 

*/ 

#include "aero.h" 
#include "matrix_operations.h" 
#include "atmosphere.h"
#include "autopilot.h"
#include "./guidance.h"
#include <bits/stdc++.h> 

using namespace ARO; 
using namespace N; 
using namespace AT; 
using namespace AP; 
using namespace GD; 

// Function 2 - Rotate from Body Frame to Stability Frame via Alpha 
void aero::rotate_B2S(double vector_in[], double alpha_beta_airspeed[], double vector_out[])
{

	double dcm[3][3]; 
	double alpha; 
	
	alpha = alpha_beta_airspeed[0]; 
	
	// Form DCM for Stability Axes - Rotation of y body axes about alpha 
	dcm[0][0] = cos(alpha); 
	dcm[0][1] = 0;
	dcm[0][2] = sin(alpha); 
	dcm[1][0] = 0; 
	dcm[1][1] = 1;  
	dcm[1][2] = 0;
	dcm[2][0] = -sin(alpha); 
	dcm[2][1] = 0;
	dcm[2][2] = cos(alpha); 
	
	// Rotate using matrix formula 
	vector_math::matrix_2_vect(dcm, vector_in, vector_out);

}

// Function 3 - Rotate from Stability Frame to Body Frame via Alpha 
void aero::rotate_S2B(double vector_in[], double alpha_beta_airspeed[], double vector_out[])
{
	
	double dcm[3][3]; 
	
	double alpha; 
	
	alpha = alpha_beta_airspeed[0]; 
	
	// Form DCM for Stability Axes - Rotation of y body axes about alpha - transpose 
	dcm[0][0] = cos(alpha); 
	dcm[1][0] = 0;
	dcm[2][0] = sin(alpha); 
	dcm[0][1] = 0; 
	dcm[1][1] = 1;  
	dcm[2][1] = 0;
	dcm[0][2] = -sin(alpha); 
	dcm[1][2] = 0;
	dcm[2][2] = cos(alpha); 
	
	// Rotate using matrix formula 
	vector_math::matrix_2_vect(dcm, vector_in, vector_out);
	
}

// Function 4 - Rotate from Body Frame to Wind Frame via Alpha and Beta
void aero::rotate_B2W(double vector_in[], double alpha_beta_airspeed[], double vector_out[])
{

	double dcm[3][3]; 
	double alpha, beta; 
	
	alpha = alpha_beta_airspeed[0]; 
	beta = alpha_beta_airspeed[1]; 
	
	// Form DCM for Wind Axes - Rotation of y body axes about alpha and stability z about beta 
	dcm[0][0] = cos(alpha)*cos(beta); 
	dcm[0][1] = sin(beta);
	dcm[0][2] = sin(alpha)*cos(beta); 
	dcm[1][0] = -cos(alpha)*sin(beta); 
	dcm[1][1] = cos(beta);  
	dcm[1][2] = sin(alpha)*sin(beta);
	dcm[2][0] = -sin(alpha); 
	dcm[2][1] = 0;
	dcm[2][2] = cos(alpha); 
	
	// Rotate using matrix formula 
	vector_math::matrix_2_vect(dcm, vector_in, vector_out);

}

// Function 5 - Rotate from Wind Frame to Body Frame via Alpha and Beta
void aero::rotate_W2B(double vector_in[], double alpha_beta_airspeed[], double vector_out[])
{

	double dcm[3][3]; 
	double alpha, beta; 
	
	alpha = alpha_beta_airspeed[0]; 
	beta = alpha_beta_airspeed[1]; 
	
	// Form DCM for Wind Axes - Rotation of y body axes about alpha and stability z about beta - Transpose
	dcm[0][0] = cos(alpha)*cos(beta); 
	dcm[1][0] = sin(beta);
	dcm[2][0] = sin(alpha)*cos(beta); 
	dcm[0][1] = -cos(alpha)*sin(beta); 
	dcm[1][1] = cos(beta);  
	dcm[2][1] = sin(alpha)*sin(beta);
	dcm[0][2] = -sin(alpha); 
	dcm[1][2] = 0;
	dcm[2][2] = cos(alpha); 
	
	// Rotate using matrix formula 
	vector_math::matrix_2_vect(dcm, vector_in, vector_out);

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
//				    WIND FRAME FORCES                                           //
// 				                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////

// Function 6 - Drag Force Stackup - q * s * drag force coefficients --
// DATCOM output for now, assume it accounts for Parasitic Drag and NOT induced drag
void aero::drag_force_wind_x(double q, double alpha_beta_airspeed[], double& drag_out) 
{
	
	double params[28];
	/*
	params[0] = mass
	params[1] = C_bar/Chord
	params[2] = Wingspan/b 
	params[3] = Reference Area
	params[4] = cl_p 
	params[5] = cl_r
	params[6] = cm_q
	params[7] = cn_p 
	params[8] = cn_r
	params[9] = cl_beta
	params[10] = cm 
	params[11] = cm_alpha 
	params[12] = cn_beta
	params[13] = cl_daileron 
	params[14] = cl_drudder
	params[15] = cm_delevator 
	params[16] = cn_daileron 
	params[17] = cn_drudder
	params[18] = cd
	params[19] = cl 
	params[20] = cl_alpha 
	params[21] = cl_q 
	params[22] = cy_p
	params[23] = cy_beta
	params[24] = cy_drudder
	params[25] = aspect ratio
	params[26] = oswald efficiency
	params[27] = cd_alpha
	*/
	
	// Call Test Vehicle Definition
	aero::test_vehicle_cessna(params);
	
	// Drag Force = -Cd_0 + cd_alpha*alpha + Cd_i * q * a_ref -- Acts negative along wind x axis 
	drag_out =(params[18] + (params[27] * alpha_beta_airspeed[0]) + ( (params[19]*params[19]) / (M_PI*params[26]*params[25]) ) ) * q * params[3]; 
		
}

// Function 7 - Lift Force Stackup - q * s * lift force coefficients 
void aero::lift_force_wind_z(double q_bar, double alpha_beta_airspeed[], double rot_body_radsec[], double& lift_out) 
{
	
	double params[28];
	double alpha, airspeed; 
	
	/*
	params[0] = mass
	params[1] = C_bar/Chord
	params[2] = Wingspan/b 
	params[3] = Reference Area
	params[4] = cl_p 
	params[5] = cl_r
	params[6] = cm_q
	params[7] = cn_p 
	params[8] = cn_r
	params[9] = cl_beta
	params[10] = cm 
	params[11] = cm_alpha 
	params[12] = cn_beta
	params[13] = cl_daileron 
	params[14] = cl_drudder
	params[15] = cm_delevator 
	params[16] = cn_daileron 
	params[17] = cn_drudder
	params[18] = cd
	params[19] = cl 
	params[20] = cl_alpha 
	params[21] = cl_q 
	params[22] = cy_p
	params[23] = cy_beta
	params[24] = cy_drudder
	params[25] = aspect ratio
	params[26] = oswald efficiency
	params[27] = cd_alpha
	*/
	
	alpha = alpha_beta_airspeed[0]; 
	airspeed = alpha_beta_airspeed[2]; 
	
	// Call Test Vehicle Definition
	aero::test_vehicle_cessna(params);
	
	// Lift Force = ( Cl_0 + Clq + Cl_alpha ) * q * s
	lift_out = (params[19] + ( params[20] * alpha ) + ( (params[21] * rot_body_radsec[1])/( 2 * airspeed ) ) ) * q_bar * params[3]; 
		
} 

// Function 8 - Side Force Stackup - q * s * side force coefficients 
void aero::side_force_wind_y(double q_bar, double alpha_beta_airspeed[], double rot_body_radsec[], double& side_force_out) 
{
	
	double params[28];
	double beta, airspeed; 
	
	/*
	params[0] = mass
	params[1] = C_bar/Chord
	params[2] = Wingspan/b 
	params[3] = Reference Area
	params[4] = cl_p 
	params[5] = cl_r
	params[6] = cm_q
	params[7] = cn_p 
	params[8] = cn_r
	params[9] = cl_beta
	params[10] = cm 
	params[11] = cm_alpha 
	params[12] = cn_beta
	params[13] = cl_daileron 
	params[14] = cl_drudder
	params[15] = cm_delevator 
	params[16] = cn_daileron 
	params[17] = cn_drudder
	params[18] = cd
	params[19] = cl 
	params[20] = cl_alpha 
	params[21] = cl_q 
	params[22] = cy_p
	params[23] = cy_beta
	params[24] = cy_drudder
	params[25] = aspect ratio
	params[26] = oswald efficiency
	params[27] = cd_alpha
	*/
	
	beta = alpha_beta_airspeed[1]; 
	airspeed = alpha_beta_airspeed[2]; 
	
	// Call Test Vehicle Definition
	aero::test_vehicle_cessna(params);
	
	// // Cy_beta  + Cy_p + cy_dr * q * s
	side_force_out = (( params[23] * beta)+ ( (params[22] * rot_body_radsec[0])/( 2 * airspeed ) ) ) * q_bar * params[3]; 
		
} 

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
//				    BODY FRAME MOMENTS                                          //
// 				                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////

// Function 9 - Moment Stackup - q * s * l(control force lever arm) * moment coefficients
void aero::moments_body(double alpha_beta_airspeed[], double q_bar, double rot_body_radsec[], double delevator, double drudder, double daileron, double moments_out_LMN_body[])
{

	// THIS VOID CONTAINS ALL THE CONTROL SURFACE DEFLECTION MOMENT CONTRIBUTIONS TOO
	
	double params[28];
	double airspeed; 
	double alpha; 
	double beta; 
	double p, q, r;
	double cl_p, cl_r, cm_q, cn_p, cn_r;  
	double cl_beta, cm, cm_alpha, cn_beta; 
	double cl_daileron, cl_drudder, cm_delevator, cn_daileron, cn_drudder;
	
	/*
	params[0] = mass
	params[1] = C_bar/Chord
	params[2] = Wingspan/b 
	params[3] = Reference Area
	params[4] = cl_p 
	params[5] = cl_r
	params[6] = cm_q
	params[7] = cn_p 
	params[8] = cn_r
	params[9] = cl_beta
	params[10] = cm 
	params[11] = cm_alpha 
	params[12] = cn_beta
	params[13] = cl_daileron 
	params[14] = cl_drudder
	params[15] = cm_delevator 
	params[16] = cn_daileron 
	params[17] = cn_drudder
	params[18] = cd
	params[19] = cl 
	params[20] = cl_alpha 
	params[21] = cl_q 
	params[22] = cy_p
	params[23] = cy_beta
	params[24] = cy_drudder
	params[25] = aspect ratio
	params[26] = oswald efficiency
	params[27] = cd_alpha
	*/
	
	// Call Vehicle Definitions 
	aero::test_vehicle_cessna(params); 
	
	// Assign values to airseed, alpha and beta for readability
	alpha = alpha_beta_airspeed[0]; 
	beta  = alpha_beta_airspeed[1]; 
	airspeed = alpha_beta_airspeed[2]; 
	
	// Assign values to p, q, r for readability 
	p = rot_body_radsec[0]; 
	q = rot_body_radsec[1]; 
	r = rot_body_radsec[2]; 
	
	// Assign values to dynamic derivatives for readibility 
	cl_p = params[4]; 
	cl_r = params[5]; 
	cm_q = params[6]; 
	cn_p = params[7]; 
	cn_r = params[8];
	cl_beta = params[9]; 
	cm = params[10]; 
	cm_alpha = params[11]; 
	cn_beta = params[12];
	cl_daileron = params[13]; 
	cl_drudder = params[14]; 
	cm_delevator = params[15]; 
	cn_daileron = params[16]; 
	cn_drudder = params[17]; 
	
	// This is where you'd call the result of a autopilot if you had one -- to deflect the control surfaces 

	// Rolling Moment (L) = CL_beta * beta + (Cl_p * p * b/ 2*v_total) + (Cl_r * r * b/ 2*v_total) + Cl_d_aileron * d_aileron + 
	// Cl_d_rudder * d_rudder 
	moments_out_LMN_body[0] = ( (cl_beta * beta) + ( (cl_p * p * params[2]) / (2*airspeed) ) + ( (cl_r * r * params[2]) / 
	(2*airspeed) ) + ( cl_daileron * daileron ) + ( cl_drudder * drudder) ) * q_bar * params[3] * params[2]; 
	
	// Pitching Moment (M) = Cm + Cm_alpha * alpha + (Cm_q * q * c / 2 * v_total) + Cm_d_elevator * d_elevator
	moments_out_LMN_body[1] = (cm + (cm_alpha * alpha) + (cm_q * q * params[1]) / (2*airspeed) + (cm_delevator * delevator) ) * 
	q_bar * params[3] * params[1]; 
	
	// Yawing Moment (N) = CN_beta * beta + (Cn_p * p * b/ 2*v_total) + (Cn_r * r * b/ 2*v_total) + Cn_d_aileron * d_aileron + 
	// Cn_d_rudder * d_rudder 
	moments_out_LMN_body[2] = ( (cn_beta * beta) + ( (cn_p * p * params[2]) / (2*airspeed) ) + ( (cn_r * r * params[2]) / 
	(2*airspeed) ) + ( cn_daileron * daileron ) + ( cn_drudder * drudder ) ) * q_bar * params[3] * params[2];

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//// Test Vehicle Definition - Cessna 172 at Sea Level and 1000 Feet 
void aero::test_vehicle_cessna(double params[])
{
/*

0 Feet: 

 -----------------------  FLIGHT CONDITIONS  ------------------------           --------------  REFERENCE DIMENSIONS  ------------
  MACH    ALTITUDE   VELOCITY    PRESSURE    TEMPERATURE     REYNOLDS             REF.      REFERENCE LENGTH   MOMENT REF. CENTER
 NUMBER                                                       NUMBER              AREA       LONG.     LAT.     HORIZ      VERT
             FT       FT/SEC     LB/FT**2       DEG R         1/FT               FT**2        FT        FT        FT        FT
0 0.160       0.00     178.61   2.1162E+03     518.670     1.1309E+06           176.310      4.955    36.000     7.290     3.250
0                                                               -------------------DERIVATIVE (PER RADIAN)-------------------
0 ALPHA     CD       CL       CM       CN       CA       XCP        CLA          CMA          CYB          CNB          CLB
0
   -2.5    0.029   -0.083    0.0313  -0.084    0.025   -0.374    5.163E+00   -3.210E-01   -2.697E-01   -7.621E-03   -1.935E-01
    0.0    0.030    0.148    0.0069   0.148    0.030    0.047    5.408E+00   -6.416E-01                             -2.013E-01
    1.0    0.031    0.243   -0.0048   0.244    0.027   -0.020    5.540E+00   -7.192E-01                             -2.045E-01
    2.4    0.035    0.384   -0.0244   0.385    0.019   -0.063    5.745E+00   -8.533E-01                             -2.093E-01
    4.9    0.047    0.639   -0.0664   0.640   -0.008   -0.104    6.024E+00   -1.120E+00                             -2.179E-01
    7.5    0.066    0.918   -0.1242   0.919   -0.054   -0.135    6.250E+00   -1.440E+00                             -2.275E-01
   10.0    0.092    1.195   -0.1940   1.192   -0.117   -0.163    6.158E+00   -1.770E+00                             -2.368E-01
   12.5    0.122    1.455   -0.2787   1.447   -0.196   -0.193    5.795E+00   -2.110E+00                             -2.452E-01

0           -------PITCHING-------    -----ACCELERATION------    --------------ROLLING--------------    --------YAWING--------
0   ALPHA       CLQ          CMQ           CLAD         CMAD         CLP          CYP          CNP          CNR          CLR
0
    -2.50    7.026E+00   -5.498E+00     2.077E+00   -5.666E+00   -4.429E-01   -4.170E-02    2.969E-03   -2.661E-02   -1.086E-02
     0.00                               2.078E+00   -5.668E+00   -4.747E-01   -4.627E-02   -1.080E-02   -2.741E-02    2.993E-02
     1.00                               2.074E+00   -5.659E+00   -4.870E-01   -4.818E-02   -1.626E-02   -2.801E-02    4.707E-02
     2.43                               2.061E+00   -5.623E+00   -5.024E-01   -5.098E-02   -2.408E-02   -2.920E-02    7.230E-02
     4.90                               2.022E+00   -5.517E+00   -5.241E-01   -5.606E-02   -3.778E-02   -3.221E-02    1.176E-01
     7.50                               1.949E+00   -5.316E+00   -5.398E-01   -6.166E-02   -5.260E-02   -3.682E-02    1.670E-01
    10.00                               1.808E+00   -4.932E+00   -5.227E-01   -6.747E-02   -6.921E-02   -4.268E-02    2.155E-01
    12.50                               1.713E+00   -4.672E+00   -4.770E-01   -7.265E-02   -8.582E-02   -4.920E-02    2.598E-01
    
1000 Feet: 

0 ALPHA     CD       CL       CM       CN       CA       XCP        CLA          CMA          CYB          CNB          CLB
0
   -2.5    0.029   -0.083    0.0313  -0.084    0.025   -0.374    5.163E+00   -3.210E-01   -2.697E-01   -7.498E-03   -1.935E-01
    0.0    0.030    0.148    0.0070   0.148    0.030    0.047    5.408E+00   -6.416E-01                             -2.013E-01
    1.0    0.031    0.243   -0.0048   0.244    0.027   -0.020    5.539E+00   -7.192E-01                             -2.045E-01
    2.4    0.035    0.384   -0.0244   0.385    0.019   -0.063    5.745E+00   -8.533E-01                             -2.093E-01
    4.9    0.047    0.639   -0.0664   0.640   -0.008   -0.104    6.024E+00   -1.121E+00                             -2.179E-01
    7.5    0.067    0.918   -0.1242   0.919   -0.054   -0.135    6.249E+00   -1.440E+00                             -2.275E-01
   10.0    0.092    1.195   -0.1940   1.193   -0.117   -0.163    6.158E+00   -1.770E+00                             -2.368E-01
   12.5    0.122    1.455   -0.2787   1.447   -0.196   -0.193    5.795E+00   -2.110E+00                             -2.452E-01
   
 -----------------------  FLIGHT CONDITIONS  ------------------------           --------------  REFERENCE DIMENSIONS  ------------
  MACH    ALTITUDE   VELOCITY    PRESSURE    TEMPERATURE     REYNOLDS             REF.      REFERENCE LENGTH   MOMENT REF. CENTER
 NUMBER                                                       NUMBER              AREA       LONG.     LAT.     HORIZ      VERT
             FT       FT/SEC     LB/FT**2       DEG R         1/FT               FT**2        FT        FT        FT        FT
0 0.160    1000.00     178.00   2.0409E+03     515.104     1.1003E+06           176.310      4.955    36.000     7.290     3.250
                                                    DYNAMIC DERIVATIVES (PER RADIAN)
0           -------PITCHING-------    -----ACCELERATION------    --------------ROLLING--------------    --------YAWING--------
0   ALPHA       CLQ          CMQ           CLAD         CMAD         CLP          CYP          CNP          CNR          CLR
0
    -2.50    7.026E+00   -5.498E+00     2.077E+00   -5.666E+00   -4.429E-01   -4.170E-02    2.969E-03   -2.662E-02   -1.086E-02
     0.00                               2.078E+00   -5.668E+00   -4.747E-01   -4.627E-02   -1.080E-02   -2.742E-02    2.993E-02
     1.00                               2.074E+00   -5.659E+00   -4.870E-01   -4.818E-02   -1.626E-02   -2.802E-02    4.707E-02
     2.43                               2.061E+00   -5.623E+00   -5.024E-01   -5.098E-02   -2.408E-02   -2.921E-02    7.230E-02
     4.90                               2.022E+00   -5.517E+00   -5.241E-01   -5.606E-02   -3.778E-02   -3.222E-02    1.176E-01
     7.50                               1.949E+00   -5.316E+00   -5.398E-01   -6.166E-02   -5.260E-02   -3.683E-02    1.670E-01
    10.00                               1.808E+00   -4.932E+00   -5.227E-01   -6.747E-02   -6.921E-02   -4.269E-02    2.155E-01
    12.50                               1.713E+00   -4.672E+00   -4.770E-01   -7.265E-02   -8.582E-02   -4.921E-02    2.598E-01
    
Additional Control Power and Control Coupling Derivatives (found online) 

cy_d_rudder = 0.187
cl_d_aileron = -0.178 
cl_d_rudder = 0.0147 // Roll Coupling from Rudder
cm_d_elevator = -1.28 
cn_d_aileron = -0.053 // Adverse Yaw Contribution
cn_d_rudder = -0.0657

Aspect Ratio - 7.32 

e_efficiency - 0.8

Mass Properties: 

771.107 KG at CG 

Inertia Tensor: 
1285.315    0        0 
    0    1824.93     0 
    0       0    2666.893
    
Geometry 
    
Xcg = 1.0414 m 

Zcg = 0.9271 m 

Wing Span, b = 10.91 m 

Reference Area, s = 16.3797 m2 

C_bar, mean chord = 1.493 m

*/

	params[0] = 771.107; //mass
	params[1] = 1.493;   // C_bar/Chord
	params[2] = 10.91; // Wingspan/b 
	params[3] = 16.3797; // Reference Area
	params[4] = -4.747E-01; // cl_p 
	params[5] =  0.096; //  cl_r
	params[6] = -12.4; // cm_q
	params[7] = -0.03; //  cn_p 
	params[8] = -0.099; // cn_r
	params[9] = -0.089; // cl_beta
	params[10] = -0.015; // cm 
	params[11] = -0.89; // cm_alpha 
	params[12] = 0.065; // cn_beta
	params[13] = -0.178; // cl_daileron 
	params[14] =  0.0147; // cl_drudder
	params[15] =  -1.28; // cm_delevator 
	params[16] = -0.053; // cn_daileron 
	params[17] = -0.0657; // cn_drudder
	params[18] =  0.030; // cd
	params[19] =  0.31; // cl 
	params[20] =  5.143; // cl_alpha 
	params[21] =  3.9E+00; // cl_q 
	params[22] = -4.627E-02; // cy_p
	params[23] = -2.697E-01; // cy_beta
	params[24] =  0.187; // cy_drudder
	params[25] =  7.32; // aspect ratio
	params[26] =  0.8; // oswald efficiency
	params[27] =  0.13; // cd_alpha

}

//// Test Vehicle Definition - Brick

void aero::test_vehicle_brick(double params[])
{

/*
Mass = 2.26795 kg 

c_bar = 0.2032 meters 

b = 0.101599999 meters

s = 0.0206449 meters

Inertia Tensor 
 0.0025682175      0         0 
      0      0.0084210111    0 
      0            0    0.009754656

*/

	params[0] = 2.226795; //mass
	params[1] = 0.2032;   // C_bar/Chord
	params[2] = 0.1015999; // Wingspan/b 
	params[3] = 0.0206449; // Reference Area
	params[4] = -1.0; // cl_p 
	params[5] = 0.0; //  cl_r
	params[6] = -1.0; // cm_q
	params[7] = 0.0; //  cn_p 
	params[8] = -1.0; // cn_r
	params[9] = 0; // cl_beta
	params[10] = 0; // cm 
	params[11] = 0; // cm_alpha 
	params[12] = 0; // cn_beta
	params[13] = 0; // cl_daileron 
	params[14] = 0; // cl_drudder
	params[15] = 0; // cm_delevator 
	params[16] = 0; // cn_daileron 
	params[17] = 0; // cn_drudder
	params[18] = 0; // cd
	params[19] = 0; // cl 
	params[20] = 0; // cl_alpha 
	params[21] = 0; // cl_q 
	params[22] = 0; // cy
	params[23] = 0; // cy_beta
	params[24] = 0; // cy_drudder
	params[25] = 0; // aspect ratio
	params[26] = 0; // oswald efficiency
	params[27] = 0; // cd_alpha
	
	

}

/*
//// Test Vehicle Definition - Sphere

Mass = 14.5939 kg 

Inertia Tensor 
 4.8809      0         0 
      0    4.8809      0 
      0      0      4.8809
 
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
*/

// Alpha Dot - For Future Integration
void aero::alpha_dot(double alpha_prev, double alpha_current, double dt, double& alpha_dot)
{ 

	alpha_dot = alpha_current-alpha_prev/dt;

} 

// Final Function - Aero Driver 
void aero::aero_driver(double position[], double eulers[], double nz_NED, double dt, double time, double velocity_body[], double rot_body_radsec[], double alpha_beta_airspeed[], double aero_force_body[], double aero_moments_body[], double &bank_required, double &heading_err) 
{ 
	// Internal Variables - 
	double static_pressure; 
	double temperature; 
	double rho; 
	double q; 
	double forces_in_wind[3];
	double drag_out, lift_out, side_force_out; 
	double altitude;
	double guidance_in[3]; 
	double dynamics_in[3]; 
	double lat_guidance_in[3]; 
	double lat_dynamics_in[3];
	double int_path[3]; 
	double int_path_lat[2]; 
	double del_elevator, del_ail, del_rud; 
	double v_x, v_y, v_z;  
	double throttle; 
	//double bank_required; 
	
	altitude = position[2]; 

////////////  To Do - Move this Alpha and Beta Calculation to the outside of this
	
	// Step 1: Determine Alpha and Beta 
	v_x = velocity_body[0]; 
	v_y = velocity_body[1]; 
	v_z = velocity_body[2]; 
	
	// Calculate airspeed
	alpha_beta_airspeed[2] = sqrt(v_x*v_x + v_y*v_y + v_z*v_z); 
	
	// Angle of Attack - Alpha -- Startup Transient Exists Here
	alpha_beta_airspeed[0] = atan(v_z/v_x); 
	
	// Sideslip - Beta 
	alpha_beta_airspeed[1] = atan(v_y/alpha_beta_airspeed[2]); 
	
////////////////////////////////////////////////////////////////////////////////
	
	// Step 2: Establish Atmosphere - Only Valid Up to 11km (GA Planes Wouldn't go that high anyways)
	atmos::run_1976_atmos(altitude, static_pressure, temperature, rho);
	
	// Step 3: Establish Dynamic Pressure - q
	atmos::q_bar(alpha_beta_airspeed, rho, q);
	
	//////////////////////////////////////////////////
	//                 WIND FRAME                   // 
	//////////////////////////////////////////////////
	
	// Step 4: Lift Force - NEGATIVE Along WIND Z Axis  
	aero::lift_force_wind_z(q, alpha_beta_airspeed, rot_body_radsec, lift_out);
	
	// Step 5: calculate drag component - NEGATIVE along WIND x axis
	aero::drag_force_wind_x(q, alpha_beta_airspeed, drag_out); 
	
	// Step 6: Side Force - NEGATIVE Along WIND y Axis
	aero::side_force_wind_y(q, alpha_beta_airspeed, rot_body_radsec, side_force_out);
	
	// Step 7: Bring it all together
	forces_in_wind[0] = -drag_out; // Drag Force - Acts Negatively in X + propulsion
	forces_in_wind[1] = -side_force_out; // Side Force - Acts Negatively in Y
	forces_in_wind[2] = -lift_out; // Lift Force - Acts Negatively in Z
	
	// Step 8: Rotate it back to body frame for usage in EoM
	aero::rotate_W2B(forces_in_wind, alpha_beta_airspeed, aero_force_body);
	
	//////////////////////////////////////////////////
	//                 BODY FRAME                   // 
	//////////////////////////////////////////////////
	
	
	//////////////////////////////////////////////////
	//               GNC ALGORITHMS                // 
	//////////////////////////////////////////////////
	
	// Package Inputs for Vertical -- To Come - Guidance 
	guidance_in[0] = 0.0; //(M_PI/16) * sin((M_PI/100) * time); // Theta Command - sine wave of 11.25 deg at a 200 second period - Simple Guidance Algorithm
	guidance_in[1] = 0; // Pitch Rate Command - Not Integrated in AP
	guidance_in[2] = -2000; // Altitude
	dynamics_in[0] = eulers[1]; // Pitch from EoM
	dynamics_in[1] = rot_body_radsec[1]; // q rate from EoM
	dynamics_in[2] = position[2]; // vertical rate;
	
	// Call Autopilots - Vertical,  Throttle
	autopilot::vertical_channel(guidance_in, dynamics_in, int_path, dt, del_elevator);
	autopilot::throttle_channel(guidance_in, dynamics_in, int_path, dt, throttle);
	aero_force_body[0] += 2000 * throttle; // Simulate Cruise Throttle Setting + In Body X Direction
	
	// Call Lateral Guidance Algorithm
	guidance::waypoint_guidance(position, eulers, alpha_beta_airspeed, bank_required, heading_err); 
	lat_guidance_in[0] = bank_required; //(M_PI/16) * sin((M_PI/100) * time); 
	lat_guidance_in[1] = 0; // Q Command - Unused for now 
	lat_dynamics_in[0] = eulers[0]; // Phi
	lat_dynamics_in[1] = rot_body_radsec[0]; // P about X
	
	// Call Lateral Autopilot 
	autopilot::lateral_channel(lat_guidance_in, lat_dynamics_in, int_path_lat, dt, del_rud, del_ail); 
	
	del_rud = 0; 
	
	// Next Up, add propulsion and moments too
	aero::moments_body(alpha_beta_airspeed, q, rot_body_radsec, del_elevator, del_rud, del_ail, aero_moments_body); 
	
}
