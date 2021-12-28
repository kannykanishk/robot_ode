#define SKY				(StateFilter	 <  StateTh)
#define GROUND			(StateFilter	 >= StateTh)
#define SKY_PAST		(StateFilterPast <  StateTh)
#define GROUND_PAST		(StateFilterPast >= StateTh)

double	Time			= 0;				// Time elapsed since start of simulation
int		State			= 0;				// Indicates state of sky leg = 0 or grounded leg = 1
double	StateFilter		= 0;				// Start at sky leg = 0, lp filter on State to prevent chattering
double	StateFilterPast	= 0;				// Past filter through State
double	StateTh			= 0.2;				// If StateFilter exceeds this value, it is considered grounded leg

//------- State Variables -------
// Body (can be measured by sensors)
dReal AngRoll		= 0;					// Roll angle
dReal AngPitch		= 0;					// Pitch angle
dReal AngYaw		= 0;					// Yaw angle

dReal AngVelX		= 0;					// x-axis angular velocity
dReal AngVelY		= 0;					// y-axis angular velocity
dReal AngVelZ		= 0;					// z-axis angular velocity

dReal AccX			= 0;					// x-axis acceleration
dReal AccY			= 0;					// y-axis acceleration
dReal AccZ			= 0;					// z-axis acceleration

// Leg
dReal AngLeg		= 0;					// Angle of leg	
dReal AngVelLeg		= 0;					// Angular velocity of leg
dReal TorqueLeg		= 0;					// Torque applied on leg

// Slider
dReal PosSlider		= 0;					// Position of slider
dReal VelSlider		= 0;					// Velocity of slider
dReal ForceSlider	= 0;					// Force applied to slider

// Position of the main unit (cannot be measured)
dReal PosX			= 0;					// x-axis position of the body
dReal PosY			= 0;					// y-axis position of the body
dReal PosZ			= 0;					// z-axis position of the body

dReal VelX			= 0;					// x-axis velocity of the body
dReal VelY			= 0;					// y-axis velocity of the body
dReal VelZ			= 0;					// z-axis velocity of the body

// Target values
dReal DesVel		= 0;					// Target velocity
dReal DesPos		= 0;					// Target position
dReal DesLegAng		= 0;					// Target leg angle
dReal DesContactPos = 0;					// Target ground position of leg

// Ground time
dReal GroundTime	= 0.16;					// Support leg time

void cal_state_value(void) {
	static double time_tmp = 0;
	
	// Assuming to get it from IMU
	const dReal *R	 = dBodyGetRotation(Body.body);			// Rotation matrix of the body
	const dReal *pos = dBodyGetPosition(Body.body);			// Position of the body

	AngRoll	 = atan2( R[9], R[10]);							// Roll angle
	AngPitch = atan2(-R[8], sqrt(R[9]*R[9]+R[10]*R[10]));	// Pitch angle
	AngYaw   = atan2( R[4], R[0] );							// Yaw angle

	AngVelX  = cal_d_1st_filter(0, AngRoll , OMEGA);		// x-axis angular velocity
	AngVelY  = cal_d_1st_filter(1, AngPitch, OMEGA);		// y-axis angular velocity
	AngVelZ  = cal_d_1st_filter(2, AngYaw  , OMEGA);		// z-axis angular velocity
	
	PosX = pos[0];											// x-axis position (not really measurable)
	PosY = pos[1];											// y-axis position (not really measurable)
	PosZ = pos[2];											// z-axis position (not really measurable)

	VelX = cal_d_1st_filter(5, PosX, OMEGA);				// x-axis velocity (not really measurable)
	VelY = cal_d_1st_filter(6, PosY, OMEGA);				// y-axis velocity (not really measurable)
	VelZ = cal_d_1st_filter(7, PosZ, OMEGA);				// z-axis velocity (not really measurable)
	
	AccX = cal_d_1st_filter(8 , VelX , OMEGA);				// x-axis acceleration
	AccY = cal_d_1st_filter(9 , VelY , OMEGA);				// y-axis acceleration
	AccZ = cal_d_1st_filter(10, VelZ , OMEGA);				// z-axis acceleration

	// Leg
	AngLeg    = dJointGetHingeAngle(Joint_BL);				// Angle of leg
	AngVelLeg = cal_d_1st_filter(3, AngLeg, OMEGA);			// Angular velocity of leg

	// Slider
	PosSlider = dJointGetSliderPosition(Joint_LS);			// Position of slider
	VelSlider = cal_d_1st_filter(4, PosSlider, OMEGA);		// Velocity of slider

	// Determining leg in sky and leg on ground
	StateFilterPast = StateFilter;
	StateFilter     = cal_lp_1st_filter(0, (double)(State), 50.0);
	
	if(SKY && GROUND_PAST) GroundTime = Time - time_tmp;	// Leg is in sky
	if(GROUND && SKY_PAST) time_tmp   = Time;				// Leg is on ground
}