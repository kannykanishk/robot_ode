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
dReal AngLegY1		= 0;					// y-axis angle of leg1	
dReal AngVelLegY1	= 0;					// y-axis angular velocity of leg1
dReal TorqueLegY1	= 0;					// y-axis torque applied to leg1
dReal AngLegX1		= 0;					// x-axis angle of leg1	
dReal AngVelLegX1	= 0;					// x-axis angular velocity of leg1
dReal TorqueLegX1	= 0;					// x-axis torque applied to leg1

dReal AngLegY2		= 0;					// y-axis angle of leg2
dReal AngVelLegY2	= 0;					// y-axis angular velocity of leg2
dReal TorqueLegY2	= 0;					// y-axis torque applied to leg2
dReal AngLegX2		= 0;					// x-axis angle of leg2	
dReal AngVelLegX2	= 0;					// x-axis angular velocity of leg2
dReal TorqueLegX2	= 0;					// x-axis torque applied to leg2

dReal AngLegY3		= 0;					// y-axis angle of leg3
dReal AngVelLegY3	= 0;					// y-axis angular velocity of leg3
dReal TorqueLegY3	= 0;					// y-axis torque applied to leg3
dReal AngLegX3		= 0;					// x-axis angle of leg3	
dReal AngVelLegX3	= 0;					// x-axis angular velocity of leg3
dReal TorqueLegX3	= 0;					// x-axis torque applied to leg3

dReal AngLegY4		= 0;					// y-axis angle of leg4	
dReal AngVelLegY4	= 0;					// y-axis angular velocity of leg4
dReal TorqueLegY4	= 0;					// y-axis torque applied to leg4
dReal AngLegX4		= 0;					// x-axis angle of leg4	
dReal AngVelLegX4	= 0;					// x-axis angular velocity of leg4
dReal TorqueLegX4	= 0;					// x-axis torque applied to leg4

// Sliders
dReal PosSlider1	= 0;					// Position of slider 1	
dReal VelSlider1	= 0;					// Velocity of slider 1
dReal ForceSlider1	= 0;					// Force applied to slider 1

dReal PosSlider2	= 0;					// Position of slider 2	
dReal VelSlider2	= 0;					// Velocity of slider 2
dReal ForceSlider2	= 0;					// Force applied to slider 2

dReal PosSlider3	= 0;					// Position of slider 3
dReal VelSlider3	= 0;					// Velocity of slider 3
dReal ForceSlider3	= 0;					// Force applied to slider 3

dReal PosSlider4	= 0;					// Position of slider 4
dReal VelSlider4	= 0;					// Velocity of slider 4
dReal ForceSlider4	= 0;					// Force applied to slider 4

// Position of the main unit (cannot be measured)
dReal PosX			= 0;					// x-axis position of the body
dReal PosY			= 0;					// y-axis position of the body
dReal PosZ			= 0;					// z-axis position of the body

dReal VelX			= 0;					// x-axis velocity of the body
dReal VelY			= 0;					// y-axis velocity of the body
dReal VelZ			= 0;					// z-axis velocity of the body

// Target values
dReal DesVelX		= 0;					// Target velocity along x-axis
dReal DesVelY		= 0;					// Target velocity along y-axis
dReal DesPosX		= 0;					// Target position along x-axis
dReal DesPosY		= 0;					// Target position along y-axis
dReal DesLegY1Ang	= 0;					// Target y-axis leg1 angle
dReal DesLegY2Ang	= 0;					// Target y-axis leg2 angle
dReal DesLegY3Ang	= 0;					// Target y-axis leg3 angle
dReal DesLegY4Ang	= 0;					// Target y-axis leg4 angle
dReal DesLegX1Ang	= 0;					// Target x-axis leg1 angle
dReal DesLegX2Ang	= 0;					// Target x-axis leg2 angle
dReal DesLegX3Ang	= 0;					// Target x-axis leg3 angle
dReal DesLegX4Ang	= 0;					// Target x-axis leg4 angle
dReal DesContactPosY = 0;					// Target y-axis ground position of active leg
dReal DesContactPosX = 0;					// Target x-axis ground position of active leg

// Ground time
dReal GroundTime1	= 0.096;				// Support leg time 1
dReal GroundTime2	= 0.096;				// Support leg time 2

void cal_state_value(void) {
	static double time_tmp1 = 0;
	static double time_tmp2 = 0;

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

	VelX = cal_d_1st_filter(3, PosX, OMEGA);				// x-axis velocity (not really measurable)
	VelY = cal_d_1st_filter(4, PosY, OMEGA);				// y-axis velocity (not really measurable)
	VelZ = cal_d_1st_filter(5, PosZ, OMEGA);				// z-axis velocity (not really measurable)
	
	AccX = cal_d_1st_filter(6 , VelX , OMEGA);				// x-axis acceleration
	AccY = cal_d_1st_filter(7 , VelY , OMEGA);				// y-axis acceleration
	AccZ = cal_d_1st_filter(8 , VelZ , OMEGA);				// z-axis acceleration

	// Legs
	AngLegY1    = dJointGetUniversalAngle1(Joint_BL1);		// y-axis angle of leg1
	AngLegX1    = dJointGetUniversalAngle2(Joint_BL1);		// x-axis angle of leg1
	AngVelLegY1 = cal_d_1st_filter(9, AngLegY1, OMEGA);		// y-axis angular velocity of leg1
	AngVelLegX1 = cal_d_1st_filter(10, AngLegX1, OMEGA);	// x-axis angular velocity of leg1
	
	AngLegY2    = dJointGetUniversalAngle1(Joint_BL2);		// y-axis angle of leg2
	AngLegX2    = dJointGetUniversalAngle2(Joint_BL2);		// x-axis angle of leg2
	AngVelLegY2 = cal_d_1st_filter(11, AngLegY2, OMEGA);	// y-axis angular velocity of leg2
	AngVelLegX2 = cal_d_1st_filter(12, AngLegX2, OMEGA);	// x-axis angular velocity of leg2

	AngLegY3    = dJointGetUniversalAngle1(Joint_BL3);		// y-axis angle of leg3
	AngLegX3    = dJointGetUniversalAngle2(Joint_BL3);		// x-axis angle of leg3
	AngVelLegY3 = cal_d_1st_filter(13, AngLegY3, OMEGA);	// y-axis angular velocity of leg3
	AngVelLegX3 = cal_d_1st_filter(14, AngLegX3, OMEGA);	// x-axis angular velocity of leg3

	AngLegY4    = dJointGetUniversalAngle1(Joint_BL4);		// y-axis angle of leg4
	AngLegX4    = dJointGetUniversalAngle2(Joint_BL4);		// x-axis angle of leg4
	AngVelLegY4 = cal_d_1st_filter(15, AngLegY4, OMEGA);	// y-axis angular velocity of leg4
	AngVelLegX4 = cal_d_1st_filter(16, AngLegX4, OMEGA);	// x-axis angular velocity of leg4

	// Sliders
	PosSlider1 = dJointGetSliderPosition(Joint_LS1);		// Position of slider 1
	VelSlider1 = cal_d_1st_filter(17, PosSlider1, OMEGA);	// Velocity of slider 1

	PosSlider2 = dJointGetSliderPosition(Joint_LS2);		// Position of slider 2
	VelSlider2 = cal_d_1st_filter(18, PosSlider2, OMEGA);	// Velocity of slider 2

	PosSlider3 = dJointGetSliderPosition(Joint_LS3);		// Position of slider 3
	VelSlider3 = cal_d_1st_filter(19, PosSlider3, OMEGA);	// Velocity of slider 3

	PosSlider4 = dJointGetSliderPosition(Joint_LS4);		// Position of slider 4
	VelSlider4 = cal_d_1st_filter(20, PosSlider4, OMEGA);	// Velocity of slider 4
	
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