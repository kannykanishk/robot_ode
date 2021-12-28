#define SKY1			(StateFilter1	  <  StateTh)
#define SKY2			(StateFilter2	  <  StateTh)
#define GROUND1			(StateFilter1	  >= StateTh)
#define GROUND2			(StateFilter2	  >= StateTh)
#define SKY_PAST1		(StateFilterPast1 <  StateTh)
#define SKY_PAST2		(StateFilterPast2 <  StateTh)
#define GROUND_PAST1	(StateFilterPast1 >= StateTh)
#define GROUND_PAST2	(StateFilterPast2 >= StateTh)

double	Time			 = 0;				// Time elapsed since start of simulation
int		State1			 = 0;				// Indicates state of sky leg1 = 0 or grounded leg1 = 1
int		State2			 = 0;				// Indicates state of sky leg2 = 0 or grounded leg2 = 1
double	StateFilter1	 = 0;				// Start at sky leg1 = 0, lp filter on State to prevent chattering
double  StateFilter2	 = 0;				// Start at sky leg1 = 0, lp filter on State to prevent chattering
double	StateFilterPast1 = 0;				// Past filter through State1
double	StateFilterPast2 = 0;				// Past filter through State2
double	StateTh			 = 0.2;				// If StateFilter exceeds this value, it is considered grounded leg

int StepLeg1 = 0;							// If Leg1 touches ground, this value is 1
int StepLeg2 = 1;							// If Leg2 touches ground, this value is 1

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

// Legs
dReal AngLeg1		= 0;					// Angle of leg1	
dReal AngVelLeg1	= 0;					// Angular velocity of leg1
dReal TorqueLeg1	= 0;					// Torque applied to leg1

dReal AngLeg2		= 0;					// Angle of leg2	
dReal AngVelLeg2	= 0;					// Angular velocity of leg2
dReal TorqueLeg2	= 0;					// Torque applied to leg2

// Sliders
dReal PosSlider1	= 0;					// Position of slider 1	
dReal VelSlider1	= 0;					// Velocity of slider 1
dReal ForceSlider1	= 0;					// Force applied to slider 1

dReal PosSlider2	= 0;					// Position of slider 2	
dReal VelSlider2	= 0;					// Velocity of slider 2
dReal ForceSlider2	= 0;					// Force applied to slider 2

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
dReal DesLeg1Ang	= 0;					// Target leg1 angle
dReal DesLeg2Ang	= 0;					// Target leg2 angle
dReal DesContactPos = 0;					// Target ground position of active leg

// Ground time
dReal GroundTime1	= 0.096;				// Support leg time 1
dReal GroundTime2	= 0.096;				// Support leg time 2

void cal_state_value(void) {
	static double time_tmp1 = 0;
	static double time_tmp2 = 0;
	
	// Assuming to get it from IMU
	const dReal *R	= dBodyGetRotation(Body.body);			// Rotation matrix of the body
	const dReal *pos= dBodyGetPosition(Body.body);			// Position of the body

	AngRoll	   = atan2( R[9], R[10]);						// Roll angle
	AngPitch   = atan2(-R[8], sqrt(R[9]*R[9]+R[10]*R[10]));	// Pitch angle
	AngYaw     = atan2( R[4], R[0] );						// Yaw angle

	AngVelX    = cal_d_1st_filter(0, AngRoll , OMEGA);		// x-axis angular velocity
	AngVelY    = cal_d_1st_filter(1, AngPitch, OMEGA);		// y-axis angular velocity
	AngVelZ    = cal_d_1st_filter(2, AngYaw  , OMEGA);		// z-axis angular velocity
	
	PosX = pos[0];											// x-axis position (not really measurable)
	PosY = pos[1];											// y-axis position (not really measurable)
	PosZ = pos[2];											// z-axis position (not really measurable)

	VelX = cal_d_1st_filter(3, PosX, OMEGA);				// x-axis velocity (not really measurable)
	VelY = cal_d_1st_filter(4, PosY, OMEGA);				// y-axis velocity (not really measurable)
	VelZ = cal_d_1st_filter(5, PosZ, OMEGA);				// z-axis velocity (not really measurable)
	
	AccX = cal_d_1st_filter(6, VelX , OMEGA);				// x-axis acceleration
	AccY = cal_d_1st_filter(7, VelY , OMEGA);				// y-axis acceleration
	AccZ = cal_d_1st_filter(8, VelZ , OMEGA);				// z-axis acceleration

	// Legs
	AngLeg1    = dJointGetHingeAngle(Joint_BL1);			// Angle of leg1
	AngVelLeg1 = cal_d_1st_filter(9, AngLeg1, OMEGA);		// Angular velocity of leg1

	AngLeg2    = dJointGetHingeAngle(Joint_BL2);			// Angle of leg2
	AngVelLeg2 = cal_d_1st_filter(10, AngLeg2, OMEGA);		// Angular velocity of leg2

	// Sliders
	PosSlider1 = dJointGetSliderPosition(Joint_LS1);		// Position of slider 1
	VelSlider1 = cal_d_1st_filter(11, PosSlider1, OMEGA);	// Velocity of slider 1

	PosSlider2 = dJointGetSliderPosition(Joint_LS2);		// Position of slider 2
	VelSlider2 = cal_d_1st_filter(12, PosSlider2, OMEGA);	// Velocity of slider 2
	
	// Determining leg in sky and leg on ground
	StateFilterPast1 = StateFilter1;
	StateFilterPast2 = StateFilter2;
	StateFilter1     = cal_lp_1st_filter(0, (double)(State1), 200.0);
	StateFilter2     = cal_lp_1st_filter(1, (double)(State2), 200.0);
	
	if(SKY1 && GROUND_PAST1) GroundTime1 = Time - time_tmp1;	// Leg 1 is in sky
	if(GROUND1 && SKY_PAST1) time_tmp1   = Time;				// Leg 1 is on ground
	if(SKY2 && GROUND_PAST2) GroundTime2 = Time - time_tmp2;	// Leg 2 is in sky
	if(GROUND2 && SKY_PAST2) time_tmp2   = Time;				// Leg 2 is on ground
}