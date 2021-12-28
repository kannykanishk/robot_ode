#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "texturepath.h"
#include "ttconstant100.h"
#include "ttsystem.h"
#include "ttcontrol100.h"

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
dJointGroupID	contactgroup;				// Contact group
dsFunctions fn;

typedef struct {							// MyCapsuleObject structure
	dBodyID body;							// ID number of the body (rigid body) (for dynamics calculation)
	dGeomID geom;							// ID number of the geometry (for collision detection calculation)
	dReal l, x, y, z, r, m;					// Length l, x, y, z [m], radius r [m], mass m [kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg1, Slider1, Leg2, Slider2;

#include "ttstatevalue.h"

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

	DesPos = 42195.0;							// Target position

	cal_state_value();							// Calculating state quantities

	// Deriving the target speed (x-axis)
	DesVel = DesPos - PosX;
	if(DesVel >  MAX_VEL) DesVel =  MAX_VEL;
	if(DesVel < -MAX_VEL) DesVel = -MAX_VEL;

	// Checking state of both legs (see switch-case)
	if(SKY1    && SKY2    && legState == 4) legState = 1;
	if(GROUND1 && SKY2    && legState == 1) { legState = 2; contact_time = 0; }
	if(SKY1    && SKY2    && legState == 2) legState = 3;
	if(SKY1    && GROUND2 && legState == 3) { legState = 4; contact_time = 0; }
	
	// Control algorithms
	switch(legState) {

		case 1:				// Both legs are in sky, previously Leg 2 was in contact with the ground
			// Leg 1 prepare for landing
			dJointSetSliderParam(Joint_LS1, dParamLoStop, MIN_SLIDER_POS);
			if(VelZ < 0) ForceSlider1 = SPRING*(PosSlider1 + MAX_SLIDER_POS);
			else         ForceSlider1 = SLIDER_FORCE;
			
			DesContactPos = ((GroundTime1)/2.0 * VelX)/2.0 + 0.07*(VelX - DesVel);
			if     ( DesContactPos/(Leg1.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG-AngPitch;
			else if( DesContactPos/(Leg1.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG-AngPitch;
			else tmp_ang  = asin(DesContactPos/(Leg1.l + MAX_SLIDER_POS));
			DesLeg1Ang    =  AngPitch - tmp_ang;
			TorqueLeg1    = -k1*(AngLeg1  - DesLeg1Ang) - k2*AngVelLeg1;
						
			// Leg 2 retract, at opposite angle
			dJointSetSliderParam(Joint_LS2, dParamLoStop,  MIN_SLIDER_POS);
			ForceSlider2  = SLIDER_FORCE;
			DesLeg2Ang    = -DesLeg2Ang;
			TorqueLeg2 	  = -k1*(AngLeg2 - DesLeg2Ang) - k2*AngVelLeg2;
		
			break;

		case 2:				// Leg 1 is in contact with the ground
			// Leg 1 landed
			dJointSetSliderParam(Joint_LS1, dParamLoStop,  MIN_SLIDER_POS + 0.05);
			if(VelSlider1 <= 0.0) ForceSlider1 = SPRING*(PosSlider1+MAX_SLIDER_POS);
			else				  ForceSlider1 = SPRING*(PosSlider1+MAX_SLIDER_POS) + ADD_FORCE;
			TorqueLeg1   = k3*(AngPitch) + k4*AngVelY + k5*VelX;

			// Leg 2 retract, at opposite angle
			dJointSetSliderParam(Joint_LS2, dParamLoStop,  MIN_SLIDER_POS);
			ForceSlider2 = SLIDER_FORCE;
			DesLeg2Ang   = -DesLeg1Ang;
			TorqueLeg2   = -k1*(AngLeg2 - DesLeg2Ang) - k2*AngVelLeg2;
		
			break;

		case 3:				// Both legs are in sky, previously Leg 1 was in contact with the ground
			// Leg 2 prepare for landing
			dJointSetSliderParam(Joint_LS2, dParamLoStop,  MIN_SLIDER_POS);
			if(VelZ < 0) ForceSlider2  = SPRING*(PosSlider2 + MAX_SLIDER_POS);
			else         ForceSlider2  = SLIDER_FORCE;
			
			DesContactPos =  ((GroundTime2)/2.0 * VelX)/2.0 + 0.07*(VelX - DesVel);
			if     ( DesContactPos/(Leg2.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG-AngPitch;
			else if( DesContactPos/(Leg2.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG-AngPitch;
			else tmp_ang  = asin(DesContactPos/(Leg2.l + MAX_SLIDER_POS));
			DesLeg2Ang    =  AngPitch - tmp_ang;
			TorqueLeg2    = -k1*(AngLeg2  - DesLeg2Ang) - k2*AngVelLeg2;

			// Leg 1 retract, at opposite angle
			dJointSetSliderParam(Joint_LS1, dParamLoStop,  MIN_SLIDER_POS);
			ForceSlider1  = SLIDER_FORCE;
			DesLeg1Ang    = -DesLeg2Ang;
			TorqueLeg1	  = -k1*(AngLeg1 - DesLeg1Ang) - k2*AngVelLeg1;
		
			break;

		case 4:				// Leg 2 is in contact with the ground
			// Leg 2 landed
			dJointSetSliderParam(Joint_LS2, dParamLoStop,  MIN_SLIDER_POS + 0.05);
			if(VelSlider2 <= 0.0) ForceSlider2 = SPRING*(PosSlider2+MAX_SLIDER_POS);
			else				  ForceSlider2 = SPRING*(PosSlider2+MAX_SLIDER_POS) + ADD_FORCE;
			TorqueLeg2   = k3*(AngPitch) + k4*AngVelY + k5*VelX;

			// Leg 1 retract, at opposite angle
			dJointSetSliderParam(Joint_LS1, dParamLoStop,  MIN_SLIDER_POS);
			DesLeg1Ang   = -DesLeg2Ang;
			TorqueLeg1	 = -k1*(AngLeg1 - DesLeg1Ang) - k2*AngVelLeg1;
			ForceSlider1 = SLIDER_FORCE;

			break;
	}

	dJointAddSliderForce(Joint_LS1, ForceSlider1);		// Enter force into slider joint 1
	dJointAddHingeTorque(Joint_BL1, TorqueLeg1);		// Enter torque into hinge joint 1
	dJointAddSliderForce(Joint_LS2, ForceSlider2);		// Enter force into slider joint 2
	dJointAddHingeTorque(Joint_BL2, TorqueLeg2);		// Enter torque into hinge joint 2

	// Updating display view
	XYZ[0] = 0.60+PosX;								// Move the fulcrum to match the robot's position
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
	dsDrawBox(dBodyGetPosition(Body.body), dBodyGetRotation(Body.body), sides);

	// Leg display
	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg1.body), dBodyGetRotation(Leg1.body), Leg1.l, Leg1.r);

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg2.body), dBodyGetRotation(Leg2.body), Leg2.l, Leg2.r);

	// Slider display
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider1.body), dBodyGetRotation(Slider1.body), Slider1.l, Slider1.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider2.body), dBodyGetRotation(Slider2.body), Slider2.l, Slider2.r);
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

static void makeMonoBot() {								// Create robot bodies and joints
	dReal x0 = 0.0, y0 = 0.0, z0 = 0.6;
	dMass mass;

	// Generate the body in the box
	Body.x = 0.80;
	Body.y = 0.10;
	Body.z = 0.10;
	Body.m = 10.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);			// Generate the geometry of the box
	dGeomSetBody(Body.geom, Body.body);								// Body and geometry association

	// Generate legs with capsules
	Leg1.l = 0.3;
	Leg1.r = 0.05;
	Leg1.m = 0.2;

	Leg1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg1.m, 1, Leg1.r, Leg1.l);
	dBodySetMass(Leg1.body,&mass);
	dBodySetPosition(Leg1.body, x0, y0, z0-Leg1.l/2.0);
	Leg1.geom= dCreateCapsule(space, Leg1.r, Leg1.l);				// Generate the geometry of the capsule
	dGeomSetBody(Leg1.geom, Leg1.body);								// Body and geometry association

	Leg2.l = 0.3;
	Leg2.r = 0.05;
	Leg2.m = 0.2;

	Leg2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg2.m, 1, Leg2.r, Leg2.l);
	dBodySetMass(Leg2.body,&mass);
	dBodySetPosition(Leg2.body, x0, y0, z0-Leg2.l/2.0);
	Leg2.geom= dCreateCapsule(space, Leg2.r, Leg2.l);				// Generate the geometry of the capsule
	dGeomSetBody(Leg2.geom, Leg2.body);								// Body and geometry association

	// Generate sliders with capsules
	Slider1.l = 0.4;
	Slider1.r = 0.03;
	Slider1.m = 0.1;

	Slider1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider1.m, 1, Slider1.r, Slider1.l);
	dBodySetMass(Slider1.body,&mass);
	dBodySetPosition(Slider1.body, x0, y0, z0-(Leg1.l));
	Slider1.geom = dCreateCapsule(space, Slider1.r, Slider1.l);		// Generate the geometry of the capsule
	dGeomSetBody(Slider1.geom, Slider1.body);						// Body and geometry association

	Slider2.l = 0.4;
	Slider2.r = 0.03;
	Slider2.m = 0.1;

	Slider2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider2.m, 1, Slider2.r, Slider2.l);
	dBodySetMass(Slider2.body,&mass);
	dBodySetPosition(Slider2.body, x0, y0, z0-(Leg2.l));
	Slider2.geom = dCreateCapsule(space, Slider2.r, Slider2.l);		// Generate the geometry of the capsule
	dGeomSetBody(Slider2.geom, Slider2.body);						// Body and geometry association

	// Generate joints
	Joint_BL1 = dJointCreateHinge(world, 0);						// Generate hinge joint
	dJointAttach        (Joint_BL1, Leg1.body, Body.body);			// Attach bodies to joint
	dJointSetHingeAnchor(Joint_BL1, x0, y0, z0);					// Set anchor point
	dJointSetHingeAxis  (Joint_BL1,  0,  1,  0);					// Set joint axis (y-axis)
	dJointSetHingeParam (Joint_BL1, dParamLoStop, -MAX_LEG_ANG);	// Set min and max rotation angles
	dJointSetHingeParam (Joint_BL1, dParamHiStop,  MAX_LEG_ANG);

	Joint_BL2 = dJointCreateHinge(world, 0);						// Generate hinge joint
	dJointAttach        (Joint_BL2, Leg2.body, Body.body);			// Attach bodies to joint
	dJointSetHingeAnchor(Joint_BL2, x0, y0, z0);					// Set anchor point
	dJointSetHingeAxis  (Joint_BL2,  0,  1,  0);					// Set joint axis (y-axis)
	dJointSetHingeParam (Joint_BL2, dParamLoStop, -MAX_LEG_ANG);	// Set min and max rotation angles
	dJointSetHingeParam (Joint_BL2, dParamHiStop,  MAX_LEG_ANG); 

	Joint_LS1 = dJointCreateSlider(world, 0);						// Generate slider joint
	dJointAttach        (Joint_LS1, Leg1.body, Slider1.body);		// Attach bodies to joint
	dJointSetSliderAxis (Joint_LS1, 0, 0, 1);						// Set joint axis (z-axis)
	dJointSetSliderParam(Joint_LS1, dParamLoStop, MIN_SLIDER_POS);	// Set min and max rotation angles
	dJointSetSliderParam(Joint_LS1, dParamHiStop, MAX_SLIDER_POS);

	Joint_LS2 = dJointCreateSlider(world, 0);						// Generate slider joint
	dJointAttach        (Joint_LS2, Leg2.body, Slider2.body);		// Attach bodies to joint
	dJointSetSliderAxis (Joint_LS2, 0, 0, 1);						// Set joint axis (z-axis)
	dJointSetSliderParam(Joint_LS2, dParamLoStop, MIN_SLIDER_POS);	// Set min and max rotation angles
	dJointSetSliderParam(Joint_LS2, dParamHiStop, MAX_SLIDER_POS);
}

void start() {
	dsSetViewpoint(XYZ, HPR);							// Camera settings
}

void destroyMonoBot() {									// Destroy joints and bodies
	dJointDestroy(Joint_BL1);
	dJointDestroy(Joint_BL2);
	dJointDestroy(Joint_LS1);
	dJointDestroy(Joint_LS2);
	dBodyDestroy(Body.body);
	dBodyDestroy(Leg1.body);
	dBodyDestroy(Leg2.body);
	dBodyDestroy(Slider1.body);
	dBodyDestroy(Slider2.body);
}

static void restart() {									// To restart the simulation
	destroyMonoBot();
	dJointGroupDestroy(contactgroup);
	contactgroup = dJointGroupCreate(0);
	makeMonoBot();
}

void command (int cmd) {								// Keyboard commands
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