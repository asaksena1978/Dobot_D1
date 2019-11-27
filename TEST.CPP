//Code for D1 robot for CR test
//Q1 - Scanned pages
//Q2 - Object manipulator - FK
//Q3 - Object manipulator - IK
//Q4, Q5, Q6 Object trajectory
//#include<conio>
#include<iostream>
#include<math.h>

using namespace std;

const double PI = 3.1415926535;

//calculates tan inverse
//num - numerator, den - denominator - input
//an - angle in randians - output
void calculate_angle(double num, double den, double *an)
{
	double al;

	if(den == 0)
	{
		if(num == 0)
			al = 0;
		else
		{
			if(num > 0)
				al = PI/2;
			else
				al = -PI/2;
		}
	}
	else
	{
		if(num == 0)
		{
			if(den > 0)
				al = 0;
			else
				al = PI;
		}
		else
			al = atan2(num, den);
	}
	*an = al;
}

class transform_matrix
{
	double m[4][4];
	public:
		void input_transformation_matrix(double *);
		void input_DH_matrix(double, double, double, double);
		void multiply_transform_matrix(transform_matrix, transform_matrix);
		void output_position(double *);
		void output_rotation_matrix(double *);
		void output_transpose_matrix(transform_matrix);
		void output_orientation(double *);
		double output_element(int , int);
		//void output_pose(double *);
};

void transform_matrix::input_transformation_matrix(double *pose)
{
	double al = *(pose + 3);
	double bt = *(pose + 4);
	double ga = *(pose + 5);

	double c_al = cos(al);
	double s_al = sin(al);
	double c_bt = cos(bt);
	double s_bt = sin(bt);
	double c_ga = cos(ga);
	double s_ga = sin(ga);


	m[0][0] = (c_ga * c_al) - (s_ga * c_bt * s_al);
	m[0][1] = (- s_ga * c_al) -(c_ga * c_bt * s_al);
	m[0][2] = s_bt * s_al;
	m[0][3] = *pose;

	m[1][0] = (c_ga * s_al) + (s_ga * c_bt * c_al);
	m[1][1] = (-s_ga * s_al) + (c_ga * c_bt * c_al);
	m[1][2] = -s_bt * c_al;
	m[1][3] = *(pose + 1);

	m[2][0] = s_ga * s_bt;
	m[2][1] = c_ga * s_bt;
	m[2][2] = c_bt;
	m[2][3] = *(pose + 2);

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
}

double transform_matrix::output_element(int r, int c)
{
	return(m[r][c]);
}


void transform_matrix::output_orientation(double *o)
{
	double num, den;

	num = m[0][2];
	den = m[1][2];
	calculate_angle(num, den, o);

	num = (-m[1][2] * cos(*o)) + (m[0][2] * sin(*o));
	den = m[2][2];
	calculate_angle(num, den, (o + 1));

	num = m[2][0];
	den = m[2][1];
	calculate_angle(num, den, (o + 2));
}

void transform_matrix::output_transpose_matrix(transform_matrix t)
{
	int i, j;
	for(i = 0; i < 4; i++)
	{
		for(j = 0; j < 4; j++)
		{
			m[i][j] = t.m[j][i];
		}
	}
}

void transform_matrix::output_position(double *pos)
{
	for(int i = 0; i < 3; i++)
	{
		*(pos + i) = m[i][3];
	}
}

void transform_matrix::output_rotation_matrix(double *r)
{
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			*(r + (i * 3) + j) = m[i][j];
		}
	}
}

void transform_matrix::multiply_transform_matrix(transform_matrix m1, transform_matrix m2)
{
    int i, j, k;
    
    for(i = 0; i < 4; i++)
    {
        for(j = 0; j < 4; j++)
        {
            double temp = 0;
            for(k = 0; k < 4; k++)
            {
                temp = temp + (m1.m[i][k] * m2.m[k][j]);
            }
            m[i][j] = temp;
        }
    }
}

void transform_matrix::input_DH_matrix(double a, double al, double d, double th)
{
	m[0][0] = cos(th);
	m[0][1] = - sin(th) * cos(al);
	m[0][2] = sin(th) * sin(al);
	m[0][3] = a * cos(th);
	
	m[1][0] = sin(th);
	m[1][1] = cos(th) * cos(al);
	m[1][2] = - cos(th) * sin(al);
	m[1][3] = a * sin(th);
	
	m[2][0] = 0;
	m[2][1] = sin(al);
	m[2][2] = cos(al);
	m[2][3] = d;
	
	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
}

class manipulator
{
	//const double D1 = 0;		//manipulator parameters as per drawing in attached sheet
    const double D2 = 1.215;
    const double D3 = 0.2;
    const double D4 = 0.2;
	const double L1 = 0.3;
	const double L2 = 0;
	const double L3 = 0;
	const double L4 = 0;
	//const double dq_dt_max = 0.2;	//rad/ sec
	
	double pose[6];		//end-effector pose
	double r[9];		//end-effector rotational matrix
	double j[4];		//joint distance, angles
	
	//trajectory traj;

    public:
        void FK(double *);			//Forward kinematics
		void IK(double *);			//Inverse kinematics
		void input_jnt(double *);	//input joint
		void input_pose(double *);	//input pose
		void output_jnt(double *);
		void output_pose(double *);
};


void manipulator::output_pose(double *p)
{
	for(int i = 0; i < 6; i++)
		*(p + i) = pose[i];
}

void manipulator::output_jnt(double *jnt)
{
	for(int i = 0; i < 4; i++)
		*(jnt + i) = j[i];
}

void manipulator::input_pose(double *p)
{
	for(int i = 0; i < 6; i++)
		pose[i] = *(p + i);
}

void manipulator::input_jnt(double *jnt_val)
{
	for(int i = 0; i < 4; i++)
		j[i] = *(jnt_val + i);
}


void manipulator::IK(double *pose)
{	
	//https://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation
	j[0] = *(pose + 2) - L1 - L2 - L3 - L4;

	double al = atan2((*(pose + 1)), (*(pose)));

	double num = (D3 * D3) + (D4 * D4) - ((*(pose)) * (*(pose))) - ((*(pose + 1)) * (*(pose + 1)));
	double den = (2 * D3 * D4);
	double ga = acos(num/den);

	double bt = (PI - ga)/ 2.0;

	j[1] = bt + al;
	j[2] = PI - ga;

	transform_matrix t01, t12, t23, t34, t02, t03, t04;
	transform_matrix inv_t03;

	t01.input_DH_matrix(0, 0, (L1 + j[0]), 0);
	t12.input_DH_matrix(D2, 0, L2, j[1]);
	t23.input_DH_matrix(D3, 0, L3, j[2]);

	t02.multiply_transform_matrix(t01, t12);
	t03.multiply_transform_matrix(t02, t23); 

	t04.input_transformation_matrix(pose);
	inv_t03.output_transpose_matrix(t03);
	t34.multiply_transform_matrix(inv_t03, t04);

	num = t34.output_element(1, 0);
	den = t34.output_element(0, 0);
	j[3] = atan2(num, den);
}


void manipulator::FK(double *jnt)
{
	transform_matrix t01, t12, t23, t34, t02, t03, t04;
	double j1, j2, j3, j4;

	j1 = *(jnt);
	j2 = *(jnt + 1);
	j3 = *(jnt + 2);
	j4 = *(jnt + 3);

	t01.input_DH_matrix(0, 0, (L1 + j1), 0);
	t12.input_DH_matrix(D2, 0, L2, j2);
	t23.input_DH_matrix(D3, 0, L3, j3);
	t34.input_DH_matrix(D4, 0, L4, j4);

	t02.multiply_transform_matrix(t01, t12);
	t03.multiply_transform_matrix(t02, t23);
	t04.multiply_transform_matrix(t03, t34);

	t04.output_position(&pose[0]);
	t04.output_orientation(&pose[3]);
	t04.output_rotation_matrix(&r[0]);
}

//object trajectory contains coeffiecient for 6 order polynomial trajectory
class trajectory
{
	const double dq_dt_M[4] = {1.0, PI, PI, PI};		//Maximum Velocity J1 - 1 m/ sec, j2, J3, J4 - PI rad/ sec
	double c[4][3][7];									//trajectoy coefficents for 6th order polynomial
	double T_r[4], T_c[4];								//rise time and fall time - T_r, constant speed time - T_c
	double dq_dt_T[4];									//target joint velocity 
	double d2q_dt2_T[4];								//target joint acc
	double tm;											//current trajectory time
	double q[4];										//current trajectory joint value to be fed to manipulator control
	double dq_dt_prev[4];								//previous joint velocity
	double d2q_dt2_prev[4];								//previous joint acceletation
	double *log_q[4], *log_dq_dt[4], *log_d2q_dt2[4], *log_jerk[4];	//log values
	int indx;											//index for logging
	public:
		void calculate_trajectory_coefficients(double *, double *, double *, double *, const double *);	//calculates coefficeint
		void generate_trajectory();																	//generate joint values for time 0.01 sec
		void PTP(double *, double *, double);															//generate trajectory for point to point movement
};


//generate point to point motion
//input pi - intial pose, pt - target pose, t - time
//output trajectory points in log variables of trajectory object
void trajectory::PTP(double *pi, double *pt, double t)
{
    double jnt_i[4], jnt_t[4];
    double t_dash[4], tr_max[4];
    double temp;
    int i;
    int tcount;
    manipulator d1;
    
    d1.IK(pi);
    d1.output_jnt(jnt_i);
    d1.IK(pt);
    d1.output_jnt(jnt_t);
    
    for(i = 0; i < 4; i++)
    {
        t_dash[i] = (jnt_t[i] - jnt_i[i])/ dq_dt_M[i];      //time with max velocity
        if(t_dash[i] > t)
        {
            cout<<"Joint Overspeed";
            break;
        }
        else
        {
            tr_max[i] = t - t_dash[i];
            temp = 2 * tr_max[i];
            if(temp > t)
            {
                T_r[i] = tr_max[i];
                T_c[i] = t - temp;
                dq_dt_T[i] = dq_dt_M[i];
            }
            else
            {
                T_c[i] = 0;
                T_r[i] = t/2;
                dq_dt_T[i] = (2 * (jnt_t[i] - jnt_i[i]))/ t;
            }
            d2q_dt2_T[i] = (15 * dq_dt_T[i])/(8 * T_r[i]);
        }
    }
    calculate_trajectory_coefficients(jnt_i, jnt_t, dq_dt_T, d2q_dt2_T, dq_dt_M);
    
    tcount = int(t * 100);
    tm = 0;
    indx = 0;
    
    for(i = 0; i < tcount; i++)
        generate_trajectory();
}


//genetrates q_trg joint values to be fed to manipulator for every 0.01 sec
void trajectory::generate_trajectory()
{
	double t, t2, t3, t4, t5, t6;
	double dt = 0.01;		//sec
	double q_trg[4];
	double dq_dt_temp, d2q_dt2_temp, jerk_temp;
	int i, j;
	
	t = tm + dt;
	tm = t;
	for(i = 0; i < 4; i++)
	{
		if(t < T_r[i])
		{
		    j = 0;
		}
		else
		{
			if(t < (T_r[i] + T_c[i]))
			{
				j = 1;
				t = t - T_r[i];
			}
			else
			{
				j = 2;
				t = t - T_r[i] - T_c[i];
			}
		}
		t2 = t * t;
		t3 = t2 * t;
		t4 = t3 * t;
		t5 = t4 * t;
		t6 = t5 * t;

		q_trg[i] = c[i][j][0] + (c[i][j][1] * t) + (c[i][j][2] * t2) + (c[i][j][3] * t3) + (c[i][j][4] * t4) + (c[i][j][5] * t5) + (c[i][j][6] * t6);
		dq_dt_temp = (q_trg[i] - q[i])/ dt;
		d2q_dt2_temp = (dq_dt_temp - dq_dt_prev[i])/ dt;
		jerk_temp = (d2q_dt2_temp - d2q_dt2_prev[i])/ dt;

		q[i] = q_trg[i];
		dq_dt_prev[i] = dq_dt_temp;
		d2q_dt2_prev[i] = d2q_dt2_prev[i];

		*(log_q[i] + indx) = q[i];
		*(log_dq_dt[i] + indx) = dq_dt_temp;
		*(log_d2q_dt2[i] + indx) = d2q_dt2_temp;
		*(log_jerk[i] + indx) = jerk_temp;
	}
	indx = indx + 1;
}

//calculates joint trajectory 6th order polynomial coefficients
//input - intial joint position (or get it through FK, qT - target joint pose, dq_dtT target joint velocity, dq_dtM maximum velocity, d2q_dt2T target joint acceleration
//output - trajectory coeffceints of object trajectory
void trajectory::calculate_trajectory_coefficients(double *q0, double *qT, double *dq_dtT, double *dq_dtM, const double *d2q_dt2T)
{
	double Tr[4], Tr3[4], Tr4[4], Tr5[4], Tc[4]; 
	double T_dash[4];
	double q1[4], q2[4];
	int i;

	for(i = 0; i < 4; i++)
	{
		//check target speed should not cross the maximum limit
		if(*(dq_dtT + i) > *(dq_dtM + i)) *(dq_dtT + i) = *(dq_dtM + i);
		if(*(qT + i) == *(q0 + i))	*(dq_dtT + i) = 0;
		else
		{
			T_dash[i] = (*(qT + i) - *(q0 + i))/(*(dq_dtT +i));
			Tr[i] = (15 * (*(dq_dtT + i)))/((*(d2q_dt2T + i)) * 8);
			if(T_dash[i] < Tr[i])
			{
				Tc[i] = T_dash[i] - Tr[i];
			}
			else
			{
				dq_dtT[i] = sqrt((8 * ((*(qT + i)) - (*(q0 + i))) * (*(d2q_dt2T + i)))/15);
				Tr[i] = (15 * (*(dq_dtT + i)))/((*(d2q_dt2T + i)) * 8);	//constant velocity time period is zero
				Tc[i] = 0;
			}
		}
		T_r[i] = Tr[i];
		T_c[i] = Tc[i];
		dq_dt_T[i] = *(dq_dtT + i);
		d2q_dt2_T[i] = *(d2q_dt2T + i);

		Tr3[i] = Tr[i] * Tr[i] * Tr[i];
		Tr4[i] = Tr3[i] * Tr[i];
		Tr5[i] = Tr4[i] * Tr[i];
		//calculate coefficients - 6 degree polynomial
		c[i][0][0] = *(q0 + i);
		c[i][0][1] = 0;
		c[i][0][2] = 0;
		c[i][0][3] = 0;
		c[i][0][4] = (5 * (*(dq_dtT + i)))/(2 * Tr3[i]);
		c[i][0][5] = (-3 * (*(dq_dtT + i)))/ Tr4[i];
		c[i][0][6] = *(dq_dtT + i)/Tr5[i];

		q1[i] = (*(q0 + i)) + (((*(dq_dtT + i)) * Tr[i])/ 2);
		c[i][1][0] = q1[i];
		c[i][1][1] = *(dq_dtT + i);
		c[i][1][2] = 0;
		c[i][1][3] = 0;
		c[i][1][4] = 0;
		c[i][1][5] = 0;
		c[i][1][6] = 0;

		q2[i] = q1[i] + ((*(dq_dtT + i)) * Tc[i]);
		c[i][0][0] = q2[i];
		c[i][0][1] = dq_dtT[i];
		c[i][0][2] = 0;
		c[i][0][3] = 0;
		c[i][0][4] = (-5 * (*(dq_dtT + i)))/(2 * Tr3[i]);
		c[i][0][5] = (3 * (*(dq_dtT + i)))/ Tr4[i];
		c[i][0][6] = *(dq_dtT + i)/(-Tr5[i]);

		dq_dt_prev[i] = 0;
		d2q_dt2_prev[i] = 0;
	}
	tm = 0;
	indx = 0;
}



double deg_to_rad(double th)
{
    double rad = th * PI/ 180.0;
    
    return(rad);
}

double rad_to_deg(double rad)
{
    double deg = rad * 180.0/ PI;
    
    return(deg);
}

int main()
{
	//transform_matrix t01;
	//getch();
    
        //Test FK
        double jnt[4] = {0, 0, 0, 0};
        double pose[6];
        manipulator d1;
        
        d1.FK(jnt);
        d1.output_pose(pose);
        
        //test IK
        d1.IK(pose);

	return(0);
}