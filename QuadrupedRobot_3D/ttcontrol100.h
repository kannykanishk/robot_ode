double FilterTime = S_TIME;		// Sampling time used for the filter

// Add 2PI from the previous and next relationship
double add_2PI(int id, double ang){									
	static double flag[20]	= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double b_ang[20];
	static int n[20]		= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};;

	if(flag[id] == 0){
		b_ang[id] = ang;
		flag[id]  = 1;
	}

	if(ang - b_ang[id] >  PI) n[id]--;
	if(ang - b_ang[id] < -PI) n[id]++;
	b_ang[id] = ang;

	return ang + n[id]*2.0*PI;
}

// First-order low-pass filter
double cal_lp_1st_filter(int id, double x, double omega){
	static double x_p[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double y_p[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double y;
	double w = 2.0*omega*PI;

	y = (FilterTime*w*(x+x_p[id]) + (2.0-FilterTime*w)*y_p[id]) / (2.0+FilterTime*w);
	x_p[id] = x;
	y_p[id] = y;

	return y;
}

// First-order high-pass filter
double cal_hp_1st_filter(int id, double x, double omega){
	static double x_p[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double y_p[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double y;
	double w = 2.0*omega*M_PI;
	
	y = (2.0*(x-x_p[id]) + (2.0-S_TIME*w)*y_p[id]) / (2.0+S_TIME*w);
	x_p[id] = x;
	y_p[id] = y;

	return y;
}

// First-order low-pass filter + derivative filter
double cal_d_1st_filter(int id, double x, double omega){
	static double x_p[21]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double y_p[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double y;
	double w = 2.0*omega*PI;
	
	y = (2.0*w*(x-x_p[id]) + (2.0-FilterTime*w)*y_p[id]) / (2.0+FilterTime*w);
	x_p[id] = x;
	y_p[id] = y;

	return y;
}

// Second-order low-pass filter + derivative filter
double cal_d_2nd_filter(int id, double x, double omega, double zeta){
	static double x_p1[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double x_p2[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double y_p1[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static double y_p2[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	double y;
	double w = 2.0*omega*PI;

	double A;
	double B;
	double C;
	
	A =     w*w*FilterTime*FilterTime - 4.0*zeta*w*FilterTime + 4.0;
	B = 2.0*w*w*FilterTime*FilterTime - 8.0;
	C =     w*w*FilterTime*FilterTime + 4.0*zeta*w*FilterTime + 4.0;

	y = (w*w*FilterTime*(x -x_p2[id]) -B*y_p1[id] -A*y_p2[id])/C;
	x_p2[id] = x_p1[id];
	x_p1[id] = x;
	y_p2[id] = y_p1[id];
	y_p1[id] = y;

	return y;
}
