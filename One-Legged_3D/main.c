#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "texturepath.h"
#include "ttconstant100.h"
#include "ttsystem.h"
#include "ttcontrol100.h"
#include "ttstatevalue.h"

#ifdef dDOUBLE								// For both single and double precision
	#define dsDrawBox	dsDrawBoxD
	#define dsDrawSphere	dsDrawSphereD
	#define dsDrawCylinder	dsDrawCylinderD
	#define dsDrawCapsule	dsDrawCapsuleD
#endif

dWorldID		world;						// World for kinematics calculations
dSpaceID		space;						// Space for collision detection
dGeomID			ground;						// Ground surface
dJointID		Joint_BL;					// Rotational joint connecting the body and leg
dJointID		Joint_LS;					// Linear joint between leg and slider
dJointID		Joint_BS;					// Fixed joint between body and sensor
dJointGroupID	contactgroup;				// Contact group
dsFunctions fn;

typedef struct {							// MyCapsuleObject structure
	dBodyID body;							// ID number of the body (rigid body) (for dynamics calculation)
	dGeomID geom;							// ID number of the geometry (for collision detection calculation)
	dReal l, x, y, z, r, m;					// Length l, x, y, z [m], radius r [m], mass m [kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg, Slider;

static float XYZ[3] = { 0.60, -3.26, 0.80 };	// Position of the viewpoint
static float HPR[3] = { 101 , -6.50, 0    };	// Direction of gaze

static void nearCallback(void *data, dGeomID o1, dGeomID o2);
static void simLoop(int pause);
static void makeMonoBot();
void start();
void destroyMonoBot();
static void restart();
void command (int cmd);
void setDrawStuff();
void show_robot(void);

int main(int argc, char **argv) {
	setDrawStuff();
	dInitODE();									// Initializing the ODE
	world = dWorldCreate();
	dWorldSetGravity(world,0,0,-9.8);			// Set the gravity
	space        = dHashSpaceCreate(0);			// Create space for collision
	contactgroup = dJointGroupCreate(0);		// Create joint group
	ground = dCreatePlane(space, 0, 0, 1, 0);	// Create plane geometry
	makeMonoBot();								// Create the MonoBot
	dsSimulationLoop(argc,argv,1500,1000,&fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();								// Close the ODE
	
	return 0;
}


static void simLoop(int pause) {				// Simulation loop
	dReal tmp_ang = 0;

	DesPosX	= 42195.0;							// Target position (x-axis)
	DesPosY	= 42195.0;							// Target position (y-axis)

	cal_state_value();							// Calculating state quantities

	// Deriving the target speed (x-axis)
	DesVelX = DesPosX - PosX;
	if(DesVelX >  MAX_VEL) DesVelX =  MAX_VEL;
	if(DesVelX < -MAX_VEL) DesVelX = -MAX_VEL;

	DesVelY = DesPosY - PosY;
	if(DesVelY >  MAX_VEL) DesVelY =  MAX_VEL;
	if(DesVelY < -MAX_VEL) DesVelY = -MAX_VEL;

	// Control algorithms
	if(SKY) {
		// Leg prepare for landing
		ForceSlider = SPRING*PosSlider;

		// x-axis motion (y-axis rotation)
		DesContactPosX = (GroundTime * VelX)/2.0 + 0.07*(VelX - DesVelX);
		if     ( DesContactPosX/(Leg.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG+AngPitch;
		else if( DesContactPosX/(Leg.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG+AngPitch;
		else tmp_ang = asin(DesContactPosX/(Leg.l + MAX_SLIDER_POS));
		DesLegAngY   =  AngPitch - tmp_ang;
		TorqueLegY   = -100.0*(AngLegY  - DesLegAngY) - 10.0*AngVelLegY;

		// y-axis motion (x-axis rotation)
		DesContactPosY = (GroundTime * (-VelY))/2.0 + 0.07*((-VelY) - DesVelY);
		if     ( DesContactPosY/(Leg.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngRoll) ) tmp_ang =  MAX_LEG_ANG+AngRoll;
		else if( DesContactPosY/(Leg.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngRoll) ) tmp_ang = -MAX_LEG_ANG+AngRoll;
		else tmp_ang = asin(DesContactPosY/(Leg.l + MAX_SLIDER_POS));
		DesLegAngX   = AngRoll - tmp_ang;
		TorqueLegX   = -100.0*(AngLegX  - DesLegAngX) - 10.0*AngVelLegX;
	}

	// Leg is in contact with the ground
	else {
		if(VelSlider < 0)	ForceSlider = SPRING*PosSlider;
		else				ForceSlider = SPRING*(PosSlider + 0.05);

		// Torque on leg
		TorqueLegY = 1000.0*(AngPitch) + 300.0*AngVelY + 280.0*VelX;
		TorqueLegX = 1000.0*(AngRoll)  + 300.0*AngVelX + 280.0*(-VelY);
	}

	dJointAddSliderForce(Joint_LS, ForceSlider);					// Enter force into slider joint
	dJointAddUniversalTorques(Joint_BL, TorqueLegY, TorqueLegX);	// Enter torques into universal joint

	// Updating display view
	XYZ[0] = 0.60+PosX;								// Move the fulcrum to match the robot's position
	XYZ[1] = -2.60+PosY;							// Move the fulcrum to match the robot's position
	dsSetViewpoint(XYZ, HPR);						// Camera settings updated

	// Routine procedure
	dSpaceCollide(space,0,&nearCallback);			// Collision detection function
	dWorldStep(world, S_TIME);
	dJointGroupEmpty(contactgroup);					// Empty the joint group
	show_robot();									// Show the robot

	// Save the simulation results to a file (edit variables accordingly)
//	fprintf(fp_data,"%lf,", Time);
//	fprintf(fp_data,"%d,%lf,%lf,", state, f_state, GroundTime);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosX, PosY, PosZ);
//	fprintf(fp_data,"%lf,%lf,%lf,", VelX, VelY, VelZ);
//	fprintf(fp_data,"%lf,%lf,%lf,", AngRoll*180/M_PI, AngPitch*180/M_PI, AngYaw*180/M_PI);
//	fprintf(fp_data,"%lf,%lf,%lf,", AngVelX*180/M_PI, AngVelY*180/M_PI,  AngVelZ*180/M_PI);
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLeg, DesLegAng, AngVelLeg, TorqueLeg);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// Process at the end of one loop
//	printf("%3.1lf\t%3.1lf\n",Time, PosX);
	Time += S_TIME;
}

void show_robot(void) {								// Display robot
	dReal sides[3];

	// Body display
	dsSetColor(0.0,1.0,0.0); 
	sides[0]=Body.x;
	sides[1]=Body.y;
	sides[2]=Body.z;
	dsDrawBox(dBodyGetPosition(Body.body),  dBodyGetRotation(Body.body),   sides);

	// Leg display
	dsSetColor(0.0,1.0,1.0);
	dsDrawCapsule(dBodyGetPosition(Leg.body),    dBodyGetRotation(Leg.body),    Leg.l,    Leg.r   );

	// Slider display
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider.body), dBodyGetRotation(Slider.body), Slider.l, Slider.r);
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {		// Callback function
	static const int N = 10;						// Maximum number of contact points
	dContact contact[N];							// Contact points
	dJointID c;
	int isGround;
	int n;
	int i;

	// If one of the objects in contact is the ground, set isGround to non-zero
	isGround = ((ground == o1) || (ground == o2)); 

	// Determine if the slider is in contact with the ground
	if((isGround == 1) && ((Slider.geom == o1) || (Slider.geom == o2))) State = 1;	// If there's contact of slider with the ground
	if((isGround == 0) && ((Slider.geom == o1) || (Slider.geom == o2))) State = 0;	// If there's no contact of slider with the ground

	// Generate collision information (n is the number of collision points)
	n= dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if(isGround == 1) {
		for(i = 0; i < n; i++) {
			contact[i].surface.mode			= dContactBounce;	// Set the repulsive property of the contact surface
			contact[i].surface.bounce		= 0.0;				// Repulsion coefficient (0.0 to 1.0)
			contact[i].surface.bounce_vel	= 0.0;				// Minimum velocity required for repulsion
			contact[i].surface.mu			= dInfinity;		// Set surface's Coulumb friction coefficient (infinity = never slips) 
			
			// Create a contact joint
			c = dJointCreateContact(world, contactgroup, &contact[i]);
			
			// Constrain the two rigid bodies in contact with each other with a contact joint
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}
}

static void makeMonoBot() {											// Create robot bodies and joints
	dReal x0 = 0.0, y0 = 0.0, z0 = 1.0;
	dMass mass;

	// Generate the body in the box
	Body.x = 0.80;
	Body.y = 0.80;
	Body.z = 0.10;
	Body.m = 10.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);			// Generate geometry in the box
	dGeomSetBody(Body.geom, Body.body);								// Body and geometry association

	// Generate leg with capsule
	Leg.l = 0.3;
	Leg.r = 0.05;
	Leg.m = 3.0;

	Leg.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg.m, 1, Leg.r, Leg.l);
	dBodySetMass(Leg.body,&mass);
	dBodySetPosition(Leg.body, x0, y0, z0-Leg.l/2);
	Leg.geom= dCreateCapsule(space, Leg.r, Leg.l);					// Generate the geometry of the capsule
	dGeomSetBody(Leg.geom, Leg.body);								// Body and geometry association

	// Generate slider with capsule
	Slider.l = 0.4;
	Slider.r = 0.03;
	Slider.m = 0.7;

	Slider.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider.m, 1, Slider.r, Slider.l);
	dBodySetMass(Slider.body,&mass);
	dBodySetPosition(Slider.body, x0, y0, z0-(Leg.l-Slider.l/2));
	Slider.geom = dCreateCapsule(space, Slider.r, Slider.l);		// Generate the geometry of the capsule
	dGeomSetBody(Slider.geom, Slider.body);							// Body and geometry association

	// Generate joints
	Joint_BL = dJointCreateUniversal(world, 0);						// Generate universal joint
	dJointAttach            (Joint_BL, Leg.body, Body.body);
	dJointSetUniversalAnchor(Joint_BL, x0, y0, z0);
	dJointSetUniversalAxis1 (Joint_BL,  0,  1,  0);
	dJointSetUniversalAxis2 (Joint_BL,  1,  0,  0);
	dJointSetUniversalParam (Joint_BL, dParamLoStop, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL, dParamHiStop,  MAX_LEG_ANG); 

	Joint_LS = dJointCreateSlider(world, 0);
	dJointAttach        (Joint_LS, Leg.body, Slider.body);
	dJointSetSliderAxis (Joint_LS, 0, 0, 1);
	dJointSetSliderParam(Joint_LS, dParamLoStop, MIN_SLIDER_POS);
	dJointSetSliderParam(Joint_LS, dParamHiStop, MAX_SLIDER_POS);
}

void start() {
  dsSetViewpoint(XYZ, HPR);							// Camera settings
}

void destroyMonoBot() {								// Destroy joints and bodies
	dJointDestroy(Joint_BL);
	dJointDestroy(Joint_LS);
	dBodyDestroy(Body.body);
	dBodyDestroy(Leg.body);
	dBodyDestroy(Slider.body);
}

static void restart() {								// To restart the simulation
	destroyMonoBot();
	dJointGroupDestroy(contactgroup);
	contactgroup = dJointGroupCreate(0);
	makeMonoBot();
}

void command (int cmd) {							// Keyboard commands
	float xyz[3], hpr[3];

	switch (cmd) {
		case 'r': restart(); break;
		case 's':
			dsGetViewpoint(xyz, hpr);
			printf("xyz=%4.2f, %4.2f, %4.2f \n", xyz[0], xyz[1], xyz[2]);
			printf("hpr=%6.2f, %6.2f, %6.2f \n", hpr[0], hpr[1], hpr[2]);
			break;
		default: printf("Input a or 1\n"); break;
	}
}

// Setting the drawing function
void setDrawStuff() {
  fn.version = DS_VERSION;							// DrawStuff version
  fn.start   = &start;								// Pointer to the preprocessing start function
  fn.step    = &simLoop;							// Pointer to simLoop function
  fn.command = &command;							// Address of function to be called by key input
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;		// Path to textures folder (see texturepath.h)
}