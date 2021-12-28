#define S_TIME			0.004				// Time for each simulation step [s]

#define SPRING			2000.0				// Spring constant ks
#define MAX_VEL			1.2				    // Maximum linear velocity of the robot
#define MAX_LEG_ANG		(30.0*D2R)			// Maximum angle of leg joint [rad]
#define MAX_SLIDER_POS	0.175				// Maximum slider position
#define MIN_SLIDER_POS	0.1				    // Minimum slider position
#define ADD_FORCE		400.0               // Positive constant slider force (for maximum position)
#define SLIDER_FORCE	-100.0              // Negative constant slider force (for retraction)

#define OMEGA			30.0				// Omega for filters
#define ZETA			0.71				// Zeta for filters

// Parameters for control algorithm (both x-axis and y-axis)
#define k1Y              80.0
#define k2Y              6.0
#define k3Y              800.0
#define k4Y              300.0
#define k5Y              80.0

#define k1X              80.0
#define k2X              6.0
#define k3X              800.0
#define k4X              300.0
#define k5X              80.0