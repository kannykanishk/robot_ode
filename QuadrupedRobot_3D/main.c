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
dJointID		Joint_BL1;					// Rotational joint connecting the body and leg 1
dJointID		Joint_LS1;					// Linear joint between leg and slider 1
dJointID		Joint_BS;					// Fixed joint between body and sensor
dJointID		Joint_BL2;					// Rotational joint connecting the body and leg 2
dJointID		Joint_LS2;					// Linear joint between leg and slider 2
dJointID		Joint_BL3;					// Rotational joint connecting the body and leg 3
dJointID		Joint_LS3;					// Linear joint between leg and slider 3
dJointID		Joint_BL4;					// Rotational joint connecting the body and leg 4
dJointID		Joint_LS4;					// Linear joint between leg and slider 4
dJointGroupID	contactgroup;				// Contact group
dsFunctions fn;

typedef struct {							// MyCapsuleObject structure
	dBodyID body;							// ID number of the body (rigid body) (for dynamics calculation)
	dGeomID geom;							// ID number of the geometry (for collision detection calculation)
	dReal l, x, y, z, r, m;					// Length l, x, y, z [m], radius r [m], mass m[ kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg1, Slider1, Leg2, Slider2, Leg3, Slider3, Leg4, Slider4;

static float XYZ[3] = { 0.60, -3.26, 0.80 };	// Position of the viewpoint
static float HPR[3] = { 101 , -6.50, 0    };	// Direction of gaze
static int legState = 1;						// To determine state of the legs (ground or sky)

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
	static double contact_time;

	DesPosX	= 42195.0;							// Target position (x-axis)
	DesPosY = 0.0;								// Target position (y-axis)

	cal_state_value();							// Calculating state quantities

	// Deriving the target speed (x-axis and y-axis)
	DesVelX = DesPosX - PosX;
	DesVelY = DesPosY - PosY;
	if(DesVelX >  MAX_VEL) DesVelX =  MAX_VEL;
	if(DesVelX < -MAX_VEL) DesVelX = -MAX_VEL;
	if(DesVelY >  MAX_VEL) DesVelY =  MAX_VEL;
	if(DesVelY < -MAX_VEL) DesVelY = -MAX_VEL;

	// Checking state of both leg pairs (see switch-case)
	// Leg pairs: (1,4) and (2,3) - trot gait
	if(SKY1    && SKY2    && legState == 4) legState = 1;
	if(GROUND1 && SKY2    && legState == 1) { legState = 2; contact_time = 0; }
	if(SKY1    && SKY2    && legState == 2) legState = 3;
	if(SKY1    && GROUND2 && legState == 3) { legState = 4; contact_time = 0; }

	// Control algorithms
	switch(legState) {

		case 1:				// Both legs are in sky, previously Legs 2,3 were in contact with the ground
			// Legs 1,4 prepare for landing
			dJointSetSliderParam(Joint_LS1, dParamLoStop, MIN_SLIDER_POS);
			dJointSetSliderParam(Joint_LS4, dParamLoStop, MIN_SLIDER_POS);
			if(VelZ < 0) { 
				ForceSlider1 = SPRING*(PosSlider1 + MAX_SLIDER_POS);
				ForceSlider4 = SPRING*(PosSlider4 + MAX_SLIDER_POS);
			}
			else         {
				ForceSlider1 = SLIDER_FORCE;
				ForceSlider4 = SLIDER_FORCE;
			}

			// x-axis motion (y-axis rotation)
			DesContactPosX = ((GroundTime1+GroundTime2) * VelX)/2.0 + 0.07*(VelX - DesVelX);
			if     ( DesContactPosX/(Leg1.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG+AngPitch;
			else if( DesContactPosX/(Leg1.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG+AngPitch;
			else tmp_ang   =  asin(DesContactPosX/(Leg1.l + MAX_SLIDER_POS));
			DesLegY1Ang    =  AngPitch - tmp_ang;
			DesLegY4Ang    =  DesLegY1Ang;
			TorqueLegY1    = -k1Y*(AngLegY1 - DesLegY1Ang) - k2Y*AngVelLegY1;
			TorqueLegY4    = -k1Y*(AngLegY4 - DesLegY4Ang) - k2Y*AngVelLegY4;

			// y-axis motion (x-axis rotation)
			DesContactPosY = ((GroundTime1+GroundTime2) * (-VelY))/2.0 + 0.07*((-VelY) - DesVelY);
			if     ( DesContactPosY/(Leg1.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngRoll) ) tmp_ang =  MAX_LEG_ANG+AngRoll;
			else if( DesContactPosY/(Leg1.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngRoll) ) tmp_ang = -MAX_LEG_ANG+AngRoll;
			else tmp_ang   =  asin(DesContactPosY/(Leg1.l + MAX_SLIDER_POS));
			DesLegX1Ang    =  AngPitch - tmp_ang;
			DesLegX4Ang    =  DesLegX1Ang;
			TorqueLegX1    = -k1X*(AngLegX1 - DesLegX1Ang) - k2X*AngVelLegX1;
			TorqueLegX4    = -k1X*(AngLegX4 - DesLegX4Ang) - k2X*AngVelLegX4;

			// Legs 2,3 retract and at opposite angles
			ForceSlider2   = SLIDER_FORCE;
			ForceSlider3   = SLIDER_FORCE;

			DesLegY2Ang    = -DesLegY1Ang;
			DesLegY3Ang    =  DesLegY2Ang;
			DesLegX2Ang    =  0;
			DesLegX3Ang    =  0;
			TorqueLegY2    = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;
			TorqueLegY3    = -k1Y*(AngLegY3 - DesLegY3Ang) - k2Y*AngVelLegY3;
			TorqueLegX2	   = -k1X*(AngLegX2 - DesLegX2Ang) - k2X*AngVelLegX2;
			TorqueLegX3    = -k1X*(AngLegX3 - DesLegX3Ang) - k2X*AngVelLegX3;
		
			break;

		case 2:				// Legs 1,4 are in contact with the ground
			// Legs 1,4 landed
			dJointSetSliderParam(Joint_LS1, dParamLoStop,  MIN_SLIDER_POS + 0.05);
			dJointSetSliderParam(Joint_LS4, dParamLoStop,  MIN_SLIDER_POS + 0.05);
			if(VelSlider1 <= 0.0) ForceSlider1 = SPRING*(PosSlider1+MAX_SLIDER_POS);
			else				  ForceSlider1 = SPRING*(PosSlider1+MAX_SLIDER_POS) + ADD_FORCE;
			if(VelSlider4 <= 0.0) ForceSlider4 = SPRING*(PosSlider4+MAX_SLIDER_POS);
			else				  ForceSlider4 = SPRING*(PosSlider4+MAX_SLIDER_POS) + ADD_FORCE;
			TorqueLegY1   = k3Y*(AngPitch) + k4Y*AngVelY + k5Y*VelX;
			TorqueLegX1   = k3X*(AngRoll)  + k4X*AngVelX + k5X*(-VelY);
			TorqueLegY4   = TorqueLegY4;
			TorqueLegX4	  = TorqueLegX4;

			// Legs 2,3 retract and at opposite angles
			dJointSetSliderParam(Joint_LS2, dParamLoStop, MIN_SLIDER_POS);
			dJointSetSliderParam(Joint_LS3, dParamLoStop, MIN_SLIDER_POS);
			ForceSlider2  = SLIDER_FORCE;
			ForceSlider3  = SLIDER_FORCE;

			DesLegY2Ang   = -DesLegY1Ang;
			DesLegY3Ang   =  DesLegY2Ang;
			DesLegX2Ang   =  0;
			DesLegX3Ang   =  0;
			TorqueLegY2	  = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;
			TorqueLegY3   = -k1Y*(AngLegY3 - DesLegY3Ang) - k2Y*AngVelLegY3;
			TorqueLegX2	  = -k1X*(AngLegX2 - DesLegX2Ang) - k2X*AngVelLegX2;
			TorqueLegX3   = -k1X*(AngLegX3 - DesLegX3Ang) - k2X*AngVelLegX3;
		
			break;

		case 3:				// Both legs are in sky, previously Legs 1,4 were in contact with the ground
			// Legs 2,3 prepare for landing
			dJointSetSliderParam(Joint_LS2, dParamLoStop, MIN_SLIDER_POS);
			dJointSetSliderParam(Joint_LS3, dParamLoStop, MIN_SLIDER_POS);
			if(VelZ < 0) { 
				ForceSlider2 = SPRING*(PosSlider1 + MAX_SLIDER_POS);
				ForceSlider3 = SPRING*(PosSlider4 + MAX_SLIDER_POS);
			}
			else         {
				ForceSlider2 = SLIDER_FORCE;
				ForceSlider3 = SLIDER_FORCE;
			}

			// x-axis motion (y-axis rotation)
			DesContactPosX =  ((GroundTime1+GroundTime2) * VelX)/2.0 + 0.07*(VelX - DesVelX);
			if     ( DesContactPosX/(Leg4.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG + AngPitch;
			else if( DesContactPosX/(Leg4.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG + AngPitch;
			else tmp_ang  =  asin(DesContactPosX/(Leg4.l + MAX_SLIDER_POS));
			DesLegY2Ang   =  AngPitch - tmp_ang;
			DesLegY3Ang   =  DesLegY2Ang;
			TorqueLegY2   = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;
			TorqueLegY3   = -k1Y*(AngLegY3 - DesLegY3Ang) - k2Y*AngVelLegY3;

			// y-axis motion (x-axis rotation)
			DesContactPosY = ((GroundTime1+GroundTime2) * (-VelY))/2.0 + 0.07*((-VelY) - DesVelY);
			if     ( DesContactPosY/(Leg4.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngRoll) ) tmp_ang =  MAX_LEG_ANG + AngRoll;
			else if( DesContactPosY/(Leg4.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngRoll) ) tmp_ang = -MAX_LEG_ANG + AngRoll;
			else tmp_ang  =  asin(DesContactPosY/(Leg4.l + MAX_SLIDER_POS));
			DesLegX2Ang   =  AngPitch - tmp_ang;
			DesLegX3Ang   =  DesLegX2Ang;
			TorqueLegX2   = -k1X*(AngLegX2 - DesLegX2Ang) - k2X*AngVelLegX2;
			TorqueLegX3   = -k1X*(AngLegX3 - DesLegX3Ang) - k2X*AngVelLegX3;

			// Legs 1,4 retract and at opposite angles
			ForceSlider1  = SLIDER_FORCE;
			ForceSlider4  = SLIDER_FORCE;

			DesLegY1Ang   = -DesLegY2Ang;
			DesLegY4Ang   =  DesLegY1Ang;
			DesLegX1Ang   =  0;
			DesLegX4Ang   =  0;
			TorqueLegY1	  = -k1Y*(AngLegY1 - DesLegY1Ang) - k2Y*AngVelLegY1;
			TorqueLegY4   = -k1Y*(AngLegY4 - DesLegY4Ang) - k2Y*AngVelLegY4;
			TorqueLegX1	  = -k1X*(AngLegX1 - DesLegX1Ang) - k2X*AngVelLegX1;
			TorqueLegX4   = -k1X*(AngLegX4 - DesLegX4Ang) - k2X*AngVelLegX4;

			break;

		case 4:				// Legs 2,3 are in contact with the ground
			// Legs 2,3 landed
			dJointSetSliderParam(Joint_LS2, dParamLoStop,  MIN_SLIDER_POS + 0.05);
			dJointSetSliderParam(Joint_LS3, dParamLoStop,  MIN_SLIDER_POS + 0.05);
			if(VelSlider2 <= 0.0) ForceSlider2 = SPRING*(PosSlider2+MAX_SLIDER_POS);
			else				  ForceSlider2 = SPRING*(PosSlider2+MAX_SLIDER_POS) + ADD_FORCE;
			if(VelSlider3 <= 0.0) ForceSlider3 = SPRING*(PosSlider3+MAX_SLIDER_POS);
			else				  ForceSlider3 = SPRING*(PosSlider3+MAX_SLIDER_POS) + ADD_FORCE;
			TorqueLegY2	 = k3Y*(AngPitch) + k4Y*AngVelY + k5Y*VelX;
			TorqueLegX2  = k3X*(AngRoll)  + k4X*AngVelX + k5X*(-VelY);
			TorqueLegY3  = TorqueLegY2;
			TorqueLegX3  = TorqueLegX2;

			// Leg 1,4 at opposite angle
			dJointSetSliderParam(Joint_LS1, dParamLoStop, MIN_SLIDER_POS);
			dJointSetSliderParam(Joint_LS4, dParamLoStop, MIN_SLIDER_POS);
			ForceSlider1 = SLIDER_FORCE;
			ForceSlider4 = SLIDER_FORCE;

			DesLegY1Ang  = -DesLegY2Ang;
			DesLegY4Ang  =  DesLegY1Ang;
			DesLegX1Ang  =  0;
			DesLegX4Ang  =  0;
			TorqueLegY1	 = -k1Y*(AngLegY1 - DesLegY1Ang) - k2Y*AngVelLegY1;
			TorqueLegY4  = -k1Y*(AngLegY4 - DesLegY4Ang) - k2Y*AngVelLegY4;
			TorqueLegX1  = -k1X*(AngLegX1 - DesLegX1Ang) - k2X*AngVelLegX1;
			TorqueLegX4  = -k1X*(AngLegX4 - DesLegX4Ang) - k2X*AngVelLegX4;

			break;
	}
	
	dJointAddSliderForce     (Joint_LS1, ForceSlider1);					// Enter force into slider joint 1
	dJointAddUniversalTorques(Joint_BL1, TorqueLegY1, TorqueLegX1);		// Enter torques into universal joint 1
	dJointAddSliderForce     (Joint_LS2, ForceSlider2);					// Enter force into slider joint 2
	dJointAddUniversalTorques(Joint_BL2, TorqueLegY2, TorqueLegX2);		// Enter torques into universal joint 2
	dJointAddSliderForce     (Joint_LS3, ForceSlider3);					// Enter force into slider joint 3
	dJointAddUniversalTorques(Joint_BL3, TorqueLegY3, TorqueLegX3);		// Enter torques into universal joint 3
	dJointAddSliderForce     (Joint_LS4, ForceSlider4);					// Enter force into slider joint 4
	dJointAddUniversalTorques(Joint_BL4, TorqueLegY4, TorqueLegX4);		// Enter torques into universal joint 4

	// Updating display view
	XYZ[0] = 0.60+PosX;								// Move the fulcrum to match the robot's position
	dsSetViewpoint(XYZ, HPR);						// Camera aettings updated

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
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLegy, DesLegyAng, AngVelLegy, TorqueLegy);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// Process at the end of one loop
//  printf("%3.1lf\t%3.1lf\n",Time, PosX);
	Time += S_TIME;
}

void show_robot(void) {								// Display robot
	dReal sides[3];

	// Body display
	dsSetColor(0.0,1.0,0.0); 
	sides[0]=Body.x;
	sides[1]=Body.y;
	sides[2]=Body.z;
	dsDrawBox(dBodyGetPosition(Body.body), dBodyGetRotation(Body.body), sides);

	// Leg display
	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg1.body), dBodyGetRotation(Leg1.body), Leg1.l, Leg1.r);

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg2.body), dBodyGetRotation(Leg2.body), Leg2.l, Leg2.r);

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg3.body), dBodyGetRotation(Leg3.body), Leg3.l, Leg3.r);

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg4.body), dBodyGetRotation(Leg4.body), Leg4.l, Leg4.r);

	// Slider display
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider1.body), dBodyGetRotation(Slider1.body), Slider1.l, Slider1.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider2.body), dBodyGetRotation(Slider2.body), Slider2.l, Slider2.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider3.body), dBodyGetRotation(Slider3.body), Slider3.l, Slider3.r);

	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider4.body), dBodyGetRotation(Slider4.body), Slider4.l, Slider4.r);
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
	if((isGround == 1) && ((Slider1.geom == o1) || (Slider1.geom == o2)))  State1 = 1;		// If there's contact of slider 1 with the ground
	if((isGround == 0) && ((Slider1.geom == o1) || (Slider1.geom == o2)))  State1 = 0;		// If there's no contact of slider 1 with the ground
	if((isGround == 1) && ((Slider2.geom == o1) || (Slider2.geom == o2)))  State2 = 1;		// If there's contact of slider 2 with the ground
	if((isGround == 0) && ((Slider2.geom == o1) || (Slider2.geom == o2)))  State2 = 0;		// If there's no contact of slider 2 with the ground

	// Generate collision information (n is the number of collision points)
	n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
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

static void makeMonoBot() {						// Create robot bodies and joints
	dReal x0 = 0.0, y0 = 0.0, z0 = 0.73;
	dMass mass;

	// Generate the body in the box
	Body.x = 0.40;
	Body.y = 0.40;
	Body.z = 0.10;
	Body.m = 40.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);				// Generate geometry in the box
	dGeomSetBody(Body.geom, Body.body);									// Body and geometry association

	// Generate legs with capsules
	Leg1.l = 0.3;
	Leg1.r = 0.05;
	Leg1.m = 1.5;

	Leg1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg1.m, 1, Leg1.r, Leg1.l);
	dBodySetMass(Leg1.body,&mass);
	dBodySetPosition(Leg1.body, x0-0.15, y0-0.15, z0-Leg1.l/2);
	Leg1.geom= dCreateCapsule(space, Leg1.r, Leg1.l);					// Generate the geometry of the capsule
	dGeomSetBody(Leg1.geom, Leg1.body);									// Body and geometry association

	Leg2.l = 0.3;
	Leg2.r = 0.05;
	Leg2.m = 1.5;

	Leg2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg2.m, 1, Leg2.r, Leg2.l);
	dBodySetMass(Leg2.body,&mass);
	dBodySetPosition(Leg2.body, x0+0.15, y0-0.15, z0-Leg2.l/2);
	Leg2.geom= dCreateCapsule(space, Leg2.r, Leg2.l);					// Generate the geometry of the capsule
	dGeomSetBody(Leg2.geom, Leg2.body);									// Body and geometry association

	Leg3.l = 0.3;
	Leg3.r = 0.05;
	Leg3.m = 1.5;

	Leg3.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg3.m, 1, Leg3.r, Leg3.l);
	dBodySetMass(Leg3.body,&mass);
	dBodySetPosition(Leg3.body, x0-0.15, y0+0.15, z0-Leg3.l/2);
	Leg3.geom= dCreateCapsule(space, Leg3.r, Leg3.l);					// Generate the geometry of the capsule
	dGeomSetBody(Leg3.geom, Leg3.body);									// Body and geometry association

	Leg4.l = 0.3;
	Leg4.r = 0.05;
	Leg4.m = 1.5;

	Leg4.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg4.m, 1, Leg4.r, Leg4.l);
	dBodySetMass(Leg4.body,&mass);
	dBodySetPosition(Leg4.body, x0+0.15, y0+0.15, z0-Leg4.l/2);
	Leg4.geom= dCreateCapsule(space, Leg4.r, Leg4.l);					// Generate the geometry of the capsule
	dGeomSetBody(Leg4.geom, Leg4.body);									// Body and geometry association

	// Generate sliders with capsules
	Slider1.l = 0.4;
	Slider1.r = 0.03;
	Slider1.m = 0.35;

	Slider1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider1.m, 1, Slider1.r, Slider1.l);
	dBodySetMass(Slider1.body,&mass);
	dBodySetPosition(Slider1.body, x0-0.15, y0-0.15, z0-(Leg1.l-Slider1.l/2));
	Slider1.geom= dCreateCapsule(space, Slider1.r, Slider1.l);			// Generate the geometry of the capsule
	dGeomSetBody(Slider1.geom, Slider1.body);							// Body and geometry association
	Slider2.l = 0.4;
	Slider2.r = 0.03;
	Slider2.m = 0.35;

	Slider2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider2.m, 1, Slider2.r, Slider2.l);
	dBodySetMass(Slider2.body,&mass);
	dBodySetPosition(Slider2.body, x0+0.15, y0-0.15, z0-(Leg2.l-Slider2.l/2));
	Slider2.geom= dCreateCapsule(space, Slider2.r, Slider2.l);			// Generate the geometry of the capsule
	dGeomSetBody(Slider2.geom, Slider2.body);							// Body and geometry association

	Slider3.l = 0.4;
	Slider3.r = 0.03;
	Slider3.m = 0.35;

	Slider3.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider3.m, 1, Slider3.r, Slider3.l);
	dBodySetMass(Slider3.body,&mass);
	dBodySetPosition(Slider3.body, x0-0.15, y0+0.15, z0-(Leg3.l-Slider3.l/2));
	Slider3.geom= dCreateCapsule(space, Slider3.r, Slider3.l);			// Generate the geometry of the capsule
	dGeomSetBody(Slider3.geom, Slider3.body);							// Body and geometry association

	Slider4.l = 0.4;
	Slider4.r = 0.03;
	Slider4.m = 0.35;

	Slider4.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider4.m, 1, Slider4.r, Slider4.l);
	dBodySetMass(Slider4.body,&mass);
	dBodySetPosition(Slider4.body, x0+0.15, y0+0.15, z0-(Leg4.l-Slider4.l/2));
	Slider4.geom= dCreateCapsule(space, Slider4.r, Slider4.l);			// Generate the geometry of the capsule
	dGeomSetBody(Slider4.geom, Slider4.body);							// Body and geometry association

	// Generate joints
	Joint_BL1 = dJointCreateUniversal(world, 0);						// Generate universal joint
	dJointAttach (Joint_BL1, Leg1.body, Body.body);						// Attach bodies to joint
	dJointSetUniversalAnchor(Joint_BL1, x0-0.15, y0-0.15, z0);			// Set anchor point
	dJointSetUniversalAxis1 (Joint_BL1,  0,  1,  0);					// Set joint axis 1 (y-axis)
	dJointSetUniversalAxis2 (Joint_BL1,  1,  0,  0);					// Set joint axis 2 (x-axis)
	dJointSetUniversalParam (Joint_BL1, dParamLoStop, -MAX_LEG_ANG);	// Set min and max rotation angles along both axes
	dJointSetUniversalParam (Joint_BL1, dParamHiStop,  MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL1, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL1, dParamHiStop2,  MAX_LEG_ANG);

	Joint_BL2 = dJointCreateUniversal(world, 0);						// Generate universal joint
	dJointAttach (Joint_BL2, Leg2.body, Body.body);						// Attach bodies to joint
	dJointSetUniversalAnchor(Joint_BL2, x0+0.15, y0-0.15, z0);			// Set anchor point
	dJointSetUniversalAxis1 (Joint_BL2,  0,  1,  0);					// Set joint axis 1 (y-axis)
	dJointSetUniversalAxis2 (Joint_BL2,  1,  0,  0);					// Set joint axis 2 (x-axis)
	dJointSetUniversalParam (Joint_BL2, dParamLoStop, -MAX_LEG_ANG);	// Set min and max rotation angles along both axes
	dJointSetUniversalParam (Joint_BL2, dParamHiStop,  MAX_LEG_ANG); 
	dJointSetUniversalParam (Joint_BL2, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL2, dParamHiStop2,  MAX_LEG_ANG);

	Joint_BL3 = dJointCreateUniversal(world, 0);						// Generate universal joint
	dJointAttach (Joint_BL3, Leg3.body, Body.body);						// Attach bodies to joint
	dJointSetUniversalAnchor(Joint_BL3, x0-0.15, y0+0.15, z0);			// Set anchor point
	dJointSetUniversalAxis1 (Joint_BL3,  0,  1,  0);					// Set joint axis 1 (y-axis)
	dJointSetUniversalAxis2 (Joint_BL3,  1,  0,  0);					// Set joint axis 2 (x-axis)
	dJointSetUniversalParam (Joint_BL3, dParamLoStop, -MAX_LEG_ANG);	// Set min and max rotation angles along both axes
	dJointSetUniversalParam (Joint_BL3, dParamHiStop,  MAX_LEG_ANG); 
	dJointSetUniversalParam (Joint_BL3, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL3, dParamHiStop2,  MAX_LEG_ANG);

	Joint_BL4 = dJointCreateUniversal(world, 0);						// Generate universal joint
	dJointAttach (Joint_BL4, Leg4.body, Body.body);						// Attach bodies to joint
	dJointSetUniversalAnchor(Joint_BL4, x0+0.15, y0+0.15, z0);			// Set anchor point
	dJointSetUniversalAxis1 (Joint_BL4,  0,  1,  0);					// Set joint axis 1 (y-axis)
	dJointSetUniversalAxis2 (Joint_BL4,  1,  0,  0);					// Set joint axis 2 (x-axis)
	dJointSetUniversalParam (Joint_BL4, dParamLoStop, -MAX_LEG_ANG);	// Set min and max rotation angles along both axes
	dJointSetUniversalParam (Joint_BL4, dParamHiStop,  MAX_LEG_ANG); 
	dJointSetUniversalParam (Joint_BL4, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL4, dParamHiStop2,  MAX_LEG_ANG);

	Joint_LS1 = dJointCreateSlider(world, 0);							// Generate slider joint
	dJointAttach        (Joint_LS1, Leg1.body, Slider1.body);			// Attach bodies to joint
	dJointSetSliderAxis (Joint_LS1, 0, 0, 1);							// Set joint axis (z-axis)
	dJointSetSliderParam(Joint_LS1, dParamLoStop, MIN_SLIDER_POS);		// Set min and max slider positions
	dJointSetSliderParam(Joint_LS1, dParamHiStop, MAX_SLIDER_POS);

	Joint_LS2 = dJointCreateSlider(world, 0);							// Generate slider joint
	dJointAttach        (Joint_LS2, Leg2.body, Slider2.body);			// Attach bodies to joint
	dJointSetSliderAxis (Joint_LS2, 0, 0, 1);							// Set joint axis (z-axis)
	dJointSetSliderParam(Joint_LS2, dParamLoStop, MIN_SLIDER_POS);		// Set min and max slider positions
	dJointSetSliderParam(Joint_LS2, dParamHiStop, MAX_SLIDER_POS);

	Joint_LS3 = dJointCreateSlider(world, 0);							// Generate slider joint
	dJointAttach        (Joint_LS3, Leg3.body, Slider3.body);			// Attach bodies to joint
	dJointSetSliderAxis (Joint_LS3, 0, 0, 1);							// Set joint axis (z-axis)
	dJointSetSliderParam(Joint_LS3, dParamLoStop, MIN_SLIDER_POS);		// Set min and max slider positions
	dJointSetSliderParam(Joint_LS3, dParamHiStop, MAX_SLIDER_POS);

	Joint_LS4 = dJointCreateSlider(world, 0);							// Generate slider joint
	dJointAttach        (Joint_LS4, Leg4.body, Slider4.body);			// Attach bodies to joint
	dJointSetSliderAxis (Joint_LS4, 0, 0, 1);							// Set joint axis (z-axis)
	dJointSetSliderParam(Joint_LS4, dParamLoStop, MIN_SLIDER_POS);		// Set min and max slider positions
	dJointSetSliderParam(Joint_LS4, dParamHiStop, MAX_SLIDER_POS);
}

void start() {
	dsSetViewpoint(XYZ, HPR);							// Camera settings
}

void destroyMonoBot() {									// Destroy joints and bodies
	dJointDestroy(Joint_BL1);
	dJointDestroy(Joint_BL2);
	dJointDestroy(Joint_BL3);
	dJointDestroy(Joint_BL4);
	dJointDestroy(Joint_LS1);
	dJointDestroy(Joint_LS2);
	dJointDestroy(Joint_LS3);
	dJointDestroy(Joint_LS4);
	dBodyDestroy(Body.body);
	dBodyDestroy(Leg1.body);
	dBodyDestroy(Leg2.body);
	dBodyDestroy(Leg3.body);
	dBodyDestroy(Leg4.body);
	dBodyDestroy(Slider1.body);
	dBodyDestroy(Slider2.body);
	dBodyDestroy(Slider3.body);
	dBodyDestroy(Slider4.body);
}

static void restart() {									// To restart the simulation
	destroyMonoBot();
	dJointGroupDestroy(contactgroup);
	contactgroup = dJointGroupCreate(0);
	makeMonoBot();
}

void command (int cmd) {									// Keyboard commands
	float xyz[3], hpr[3];

	switch (cmd) {
		case 'r': restart(); break;
		case 's':
			dsGetViewpoint(xyz, hpr);
			printf("xyz=%4.2f, %4.2f, %4.2f \n", xyz[0], xyz[1], xyz[2]);
			printf("hpr=%6.2f, %6.2f, %6.2f \n", hpr[0], hpr[1], hpr[2]);
			break;
		default: printf("Input r or s\n"); break;
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