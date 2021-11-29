#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#include "ttconstant100.h"
#include "ttsystem.h"
#include "ttcontrol100.h"

#ifdef dDOUBLE								// �P���x�Ɣ{���x�̗����ɑΉ����邽�߂̂��܂��Ȃ�
	#define dsDrawBox	dsDrawBoxD
	#define dsDrawSphere	dsDrawSphereD
	#define dsDrawCylinder	dsDrawCylinderD
	#define dsDrawCapsule	dsDrawCapsuleD
#endif

dWorldID		world;						// ���͊w�v�Z�p���[���h
dSpaceID		space;						// �Փˌ��o�p�X�y�[�X
dGeomID			ground;						// �n��
dJointID		Joint_BL1;					// �{�̂Ƌr���q����]�W���C���g
dJointID		Joint_LS1;					// �r�ƃX���C�_�[���q�������W���C���g
dJointID		Joint_BS;					// �{�̂ƃZ���T���q���Œ�W���C���g
dJointID		Joint_BL2;
dJointID		Joint_LS2;
dJointID		Joint_BL3;
dJointID		Joint_LS3;
dJointID		Joint_BL4;
dJointID		Joint_LS4;
dJointGroupID	contactgroup;				// �R���^�N�g�O���[�v
dsFunctions fn;

typedef struct {							// MyCapsuleObject�\����
	dBodyID body;							// �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;							// �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	dReal l, x, y, z, r, m;					// ���� l, x, y, z[m], ���a r[m]�C���� m[kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg1, Slider1, Leg2, Slider2, Leg3, Slider3, Leg4, Slider4;

#include "ttstatevalue.h"

static float XYZ[3] = { 0.10, -2.50, 0.80};	// ���_�̈ʒu
static float HPR[3] = { 101 , -6.50, 0   };	// �����̕���

static void nearCallback(void *data, dGeomID o1, dGeomID o2);
static void simLoop(int pause);
static void makeMonoBot();
void start();
void destroyMonoBot();
static void restart();
void command (int cmd);
void setDrawStuff();
void show_robot(void);

int main(int argc, char **argv){
	setDrawStuff();
	dInitODE();									// ODE�̏�����
	world = dWorldCreate();
	dWorldSetGravity(world,0,0,-9.8);			//�@�d�͂̐ݒ�
	space        = dHashSpaceCreate(0);			// �Փ˗p��Ԃ̑n��
	contactgroup = dJointGroupCreate(0);		// �W���C���g�O���[�v�̐���
	ground = dCreatePlane(space, 0, 0, 1, 0);	// ���ʃW�I���g���̐���
	makeMonoBot();								// MonoBot�̍쐬
	dsSimulationLoop(argc,argv,1200, 700,&fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();								// ODE�̏I��
	
	return 0;
}

static void simLoop(int pause){					// �V�~�����[�V�������[�v
	dReal tmp_ang = 0;

	DesPosX	= 0.0;					// �ڕW�ʒu
	DesPosY = 0.0;

	// ��ԗʂ̌v�Z
	cal_state_value();

	// �ڕW���x�̓��o
	DesVelX = DesPosX - PosX;
	DesVelY = DesPosY - PosY;
	if(DesVelX >  MAX_VEL) DesVelX =  MAX_VEL;
	if(DesVelX < -MAX_VEL) DesVelX = -MAX_VEL;
	if(DesVelY >  MAX_VEL) DesVelY =  MAX_VEL;
	if(DesVelY < -MAX_VEL) DesVelY = -MAX_VEL;

	// �V�r���̐��䑥
	if(SKY){ 
		if(StepLeg1) {
			// Leg 4,3 prepare for landing
			ForceSlider4  = SPRING*MIN_SLIDER_POS/4;
			ForceSlider3  = SPRING*MIN_SLIDER_POS/4;

			// x axis motion (y axis rotation)
			DesContactPosX =  (GroundTime * VelX)/2.0 + 0.07*(VelX - DesVelX);
			if     ( DesContactPosX/(Leg4.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG + AngPitch;
			else if( DesContactPosX/(Leg4.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG + AngPitch;
			else tmp_ang  =  asin(DesContactPosX/(Leg4.l + MAX_SLIDER_POS));
			DesLegY4Ang   =  AngPitch - tmp_ang;
			DesLegY3Ang   =  DesLegY2Ang;
			TorqueLegY4   = -k1Y*(AngLegY4 - DesLegY4Ang) - k2Y*AngVelLegY4;
			TorqueLegY3   = -k1Y*(AngLegY3 - DesLegY3Ang) - k2Y*AngVelLegY3;

			// y axis motion (x axis rotation)
			DesContactPosY = (GroundTime * VelY)/2.0 + 0.07*(VelY - DesVelY);
			if     ( DesContactPosY/(Leg4.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngRoll) ) tmp_ang =  MAX_LEG_ANG + AngRoll;
			else if( DesContactPosY/(Leg4.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngRoll) ) tmp_ang = -MAX_LEG_ANG + AngRoll;
			else tmp_ang  =  asin(DesContactPosY/(Leg4.l + MAX_SLIDER_POS));
			DesLegX4Ang   =  AngPitch - tmp_ang;
			DesLegX3Ang   =  DesLegX4Ang;
			TorqueLegX4   = -k1X*(AngLegX4 - DesLegX4Ang) - k2X*AngVelLegX4;
			TorqueLegX3   = -k1X*(AngLegX3 - DesLegX3Ang) - k2X*AngVelLegX3;

			// Leg 1,2 retract and at opposite angles
			ForceSlider1  = -SPRING*MIN_SLIDER_POS/4;
			ForceSlider2  = -SPRING*MIN_SLIDER_POS/4;

			DesLegY1Ang   = -DesLegY4Ang;
			DesLegY2Ang   =  DesLegY1Ang;
			TorqueLegY1	  = -k1Y*(AngLegY1 - DesLegY1Ang) - k2Y*AngVelLegY1;
			TorqueLegY2   = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;
		}

		if(StepLeg2) {
			// Leg 1,2 prepare for landing
			ForceSlider1   = SPRING*MIN_SLIDER_POS/4;
			ForceSlider2   = SPRING*MIN_SLIDER_POS/4;

			//x axis motion (y axis rotation)
			DesContactPosX = (GroundTime * VelX)/2.0 + 0.07*(VelX - DesVelX);
			if     ( DesContactPosX/(Leg1.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG+AngPitch;
			else if( DesContactPosX/(Leg1.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG+AngPitch;
			else tmp_ang   =  asin(DesContactPosX/(Leg1.l + MAX_SLIDER_POS));
			DesLegY1Ang    =  AngPitch - tmp_ang;
			DesLegY2Ang    =  DesLegY1Ang;
			TorqueLegY1    = -k1Y*(AngLegY1 - DesLegY1Ang) - k2Y*AngVelLegY1;
			TorqueLegY2    = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;

			//y axis motion (x axis rotation)
			DesContactPosY = (GroundTime * VelY)/2.0 + 0.07*(VelY - DesVelY);
			if     ( DesContactPosY/(Leg1.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngRoll) ) tmp_ang =  MAX_LEG_ANG+AngRoll;
			else if( DesContactPosY/(Leg1.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngRoll) ) tmp_ang = -MAX_LEG_ANG+AngRoll;
			else tmp_ang   =  asin(DesContactPosY/(Leg1.l + MAX_SLIDER_POS));
			DesLegX1Ang    =  AngPitch - tmp_ang;
			DesLegX2Ang    =  DesLegX1Ang;
			TorqueLegX1    = -k1X*(AngLegX1 - DesLegX1Ang) - k2X*AngVelLegX1;
			TorqueLegX2    = -k1X*(AngLegX2 - DesLegX2Ang) - k2X*AngVelLegX2;

			// Leg 4,3 retract and at opposite angles
			ForceSlider4   = -SPRING*MIN_SLIDER_POS/4;
			ForceSlider3   = -SPRING*MIN_SLIDER_POS/4;

			DesLegY4Ang    = -DesLegY1Ang;
			DesLegY3Ang    =  DesLegY4Ang;
			TorqueLegY2    = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;
			TorqueLegY4    = -k1Y*(AngLegY4 - DesLegY4Ang) - k2Y*AngVelLegY4;
		}
	}

	// �x���r���̐��䑥
	else {
		if(StepLeg1) {
			// Leg 1,2 touching the ground, Leg 4,3 still retracted
			if(VelSlider1 < 0)	ForceSlider1 = SPRING*MAX_SLIDER_POS;
			else				ForceSlider1 = SPRING*(PosSlider1+0.05);
			
			if(VelSlider2 < 0)	ForceSlider2 = SPRING*MAX_SLIDER_POS;
			else				ForceSlider2 = SPRING*(PosSlider4+0.05);

			TorqueLegY1   = k3Y*(AngPitch) + k4Y*AngVelY + k5Y*VelX;
			TorqueLegX1   = k3X*(AngRoll)  + k4X*AngVelX + k5X*VelY;
			TorqueLegY2   = TorqueLegY2;
			TorqueLegX2	  = TorqueLegX2;

			// Leg 2,3 retract and at opposite angles
			ForceSlider4  = -SPRING*MIN_SLIDER_POS/4;
			ForceSlider3  = -SPRING*MIN_SLIDER_POS/4;

			DesLegY4Ang   = -DesLegY1Ang;
			DesLegY3Ang   =  DesLegY4Ang;
			TorqueLegY4	  = -k1Y*(AngLegY4 - DesLegY4Ang) - k2Y*AngVelLegY4;
			TorqueLegY3   = -k1Y*(AngLegY3 - DesLegY3Ang) - k2Y*AngVelLegY3;
		}

		if(StepLeg2) {
			// Leg 4,3 touching the ground, Leg 1,2 still retracted
			if(VelSlider4 < 0)	ForceSlider4 = SPRING*MAX_SLIDER_POS;
			else				ForceSlider4 = SPRING*(PosSlider4+0.05);

			if(VelSlider3 < 0)	ForceSlider3 = SPRING*MAX_SLIDER_POS;
			else				ForceSlider3 = SPRING*(PosSlider3+0.05);

			TorqueLegY4	 = k3Y*(AngPitch) + k4Y*AngVelY + k5Y*VelX;
			TorqueLegX4  = k3X*(AngRoll)  + k4X*AngVelX + k5X*VelY;
			TorqueLegY3  = TorqueLegY4;
			TorqueLegX3  = TorqueLegX4;

			// Leg 1,2 at opposite angle
			ForceSlider1 = -SPRING*MIN_SLIDER_POS/4;
			ForceSlider2 = -SPRING*MIN_SLIDER_POS/4;

			DesLegY1Ang  = -DesLegY4Ang;
			DesLegY2Ang  =  DesLegY1Ang;
			TorqueLegY1	 = -k1Y*(AngLegY1 - DesLegY1Ang) - k2Y*AngVelLegY1;
			TorqueLegY2  = -k1Y*(AngLegY2 - DesLegY2Ang) - k2Y*AngVelLegY2;
		}
	}
//	printf("%d %d 1: %3.5f \t 4: %3.5f \t 2: %3.5f \t 3: %3.5f \n", StepLeg1, StepLeg2, PosSlider1, PosSlider4, PosSlider2, PosSlider3);
	dJointAddSliderForce     (Joint_LS1, ForceSlider1);					// �X���C�_�[�ɗ͂����
	dJointAddUniversalTorques(Joint_BL1, 0, 0);		// �r�Ƀg���N�����
	dJointAddSliderForce     (Joint_LS2, ForceSlider2);
	dJointAddUniversalTorques(Joint_BL2, 0, 0);
	dJointAddSliderForce     (Joint_LS3, ForceSlider3);
	dJointAddUniversalTorques(Joint_BL3, 0, 0);
	dJointAddSliderForce     (Joint_LS4, ForceSlider4);
	dJointAddUniversalTorques(Joint_BL4, 0, 0);

	// �\��������������߂�
	XYZ[0] = 0.10+PosX;								// ���{�b�g�̈ʒu�ɍ��킹�Ďx�_�𓮂���
	dsSetViewpoint(XYZ, HPR);						// �J�����̐ݒ�

	// �����܂�̏���
	dSpaceCollide(space,0,&nearCallback);			// �Փˌ��o�֐�
	dWorldStep(world, S_TIME);
	dJointGroupEmpty(contactgroup);					// �W���C���g�O���[�v����ɂ���
	show_robot();									// ���{�b�g��\��

	// �V�~�����[�V�������ʂ��t�@�C���ɕۑ�
//	fprintf(fp_data,"%lf,", Time);
//	fprintf(fp_data,"%d,%lf,%lf,", state, f_state, GroundTime);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosX, PosY, PosZ);
//	fprintf(fp_data,"%lf,%lf,%lf,", VelX, VelY, VelZ);
//	fprintf(fp_data,"%lf,%lf,%lf,", AngRoll*180/M_PI, AngPitch*180/M_PI, AngYaw*180/M_PI);
//	fprintf(fp_data,"%lf,%lf,%lf,", AngVelX*180/M_PI, AngVelY*180/M_PI,  AngVelZ*180/M_PI);
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLegy, DesLegyAng, AngVelLegy, TorqueLegy);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// 1���[�v�Ō�ɂ��鏈��
//	printf("%3.1lf\t%3.1lf\n",Time, PosX);
//	printf("1: %3.3f\t\t2: %3.3f\t3: %3.3f\t4:%3.3f \n", PosSlider1, PosSlider2, PosSlider3, PosSlider4);
	Time +=S_TIME;
}

void show_robot(void){
	dReal sides[3];

	// �{�̂̕\��
	dsSetColor(0.0,1.0,0.0); 
	sides[0]=Body.x;
	sides[1]=Body.y;
	sides[2]=Body.z;
	dsDrawBox(dBodyGetPosition(Body.body),  dBodyGetRotation(Body.body),   sides);

	// �r�̕\��
	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg1.body),    dBodyGetRotation(Leg1.body),    Leg1.l,    Leg1.r   );

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg2.body),    dBodyGetRotation(Leg2.body),    Leg2.l,    Leg2.r   );

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg3.body),    dBodyGetRotation(Leg3.body),    Leg3.l,    Leg3.r   );

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg4.body),    dBodyGetRotation(Leg4.body),    Leg4.l,    Leg4.r   );

	// �X���C�_�̕\��
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider1.body), dBodyGetRotation(Slider1.body), Slider1.l, Slider1.r);

	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider2.body), dBodyGetRotation(Slider2.body), Slider2.l, Slider2.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider3.body), dBodyGetRotation(Slider3.body), Slider3.l, Slider3.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider4.body), dBodyGetRotation(Slider4.body), Slider4.l, Slider4.r);
}

// �R�[���o�b�N�֐�
static void nearCallback(void *data, dGeomID o1, dGeomID o2){
	static const int N = 10;						// �ڐG�_���̍ő�l
	dContact contact[N];							// �ڐG�_
	dJointID c;
	int isGround;
	int n;
	int i;

	// �ڐG���Ă��镨�̂̂ǂ��炩���n�ʂȂ�isGround�ɔ�0���Z�b�g
	isGround = ((ground == o1) || (ground == o2)); 

	// �X���C�_�[�ƒn�ʂ��ڐG���Ă��邩�𔻒�
	if((isGround == 1) && ((Slider4.geom == o1) || (Slider4.geom == o2) || (Slider2.geom == o1) || (Slider2.geom == o2)))  State = 1;		// �ڐG�����ꍇ
	if((isGround == 0) && ((Slider4.geom == o1) || (Slider4.geom == o2) || (Slider2.geom == o1) || (Slider2.geom == o2)))  State = 0;		// �ڐG���Ă��Ȃ��ꍇ


	// �Փˏ��̐��� n�͏Փ˓_��
	n= dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if(isGround == 1){

		for(i= 0; i < n; i++){
			contact[i].surface.mode			= dContactBounce;	// �ڐG�ʂ̔�������ݒ�
			contact[i].surface.bounce		= 0.0;				// �����W��(0.0����1.0)
			contact[i].surface.bounce_vel	= 0.0;				// �����ɕK�v�ȍŒᑬ�x
			contact[i].surface.mu			= dInfinity;		// �����ɕK�v�ȍŒᑬ�x
			
			// �ڐG�W���C���g�̐���
			c = dJointCreateContact(world, contactgroup, &contact[i]);
			
			// �ڐG���Ă���Q�̍��̂�ڐG�W���C���g�ɂ��S��
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}
}

static void makeMonoBot(){
	dReal x0 = 0.0, y0 = 0.0, z0 = 0.73;
	dMass mass;

	// �{�b�N�X�Ŗ{�̂𐶐�
	Body.x = 0.30;
	Body.y = 0.30;
	Body.z = 0.10;
	Body.m = 20.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);			// �{�b�N�X�ŃW�I���g���̐���
	dGeomSetBody(Body.geom, Body.body);		


	// �J�v�Z���ŋr�𐶐�
	Leg1.l = 0.3;
	Leg1.r = 0.05;
	Leg1.m = 1.5;

	Leg1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg1.m, 1, Leg1.r, Leg1.l);
	dBodySetMass(Leg1.body,&mass);
	dBodySetPosition(Leg1.body, x0-0.10, y0-0.10, z0 -Leg1.l/2);
	Leg1.geom= dCreateCapsule(space, Leg1.r, Leg1.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg1.geom, Leg1.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	Leg2.l = 0.3;
	Leg2.r = 0.05;
	Leg2.m = 1.5;

	Leg2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg2.m, 1, Leg2.r, Leg2.l);
	dBodySetMass(Leg2.body,&mass);
	dBodySetPosition(Leg2.body, x0+0.10, y0-0.10, z0 -Leg2.l/2);
	Leg2.geom= dCreateCapsule(space, Leg2.r, Leg2.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg2.geom, Leg2.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	Leg3.l = 0.3;
	Leg3.r = 0.05;
	Leg3.m = 1.5;

	Leg3.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg3.m, 1, Leg3.r, Leg3.l);
	dBodySetMass(Leg3.body,&mass);
	dBodySetPosition(Leg3.body, x0-0.10, y0+0.10, z0 -Leg3.l/2);
	Leg3.geom= dCreateCapsule(space, Leg3.r, Leg3.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg3.geom, Leg3.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	Leg4.l = 0.3;
	Leg4.r = 0.05;
	Leg4.m = 1.5;

	Leg4.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg4.m, 1, Leg4.r, Leg4.l);
	dBodySetMass(Leg4.body,&mass);
	dBodySetPosition(Leg4.body, x0+0.10, y0+0.10, z0 -Leg4.l/2);
	Leg4.geom= dCreateCapsule(space, Leg4.r, Leg4.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg4.geom, Leg4.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	// �J�v�Z���ő��𐶐�
	Slider1.l = 0.4;
	Slider1.r = 0.03;
	Slider1.m = 0.35;

	Slider1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider1.m, 1, Slider1.r, Slider1.l);
	dBodySetMass(Slider1.body,&mass);
	dBodySetPosition(Slider1.body, x0-0.10, y0-0.10, z0 -(Leg1.l-Slider1.l/2));
	Slider1.geom= dCreateCapsule(space, Slider1.r, Slider1.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider1.geom, Slider1.body);							// �{�f�B�ƃW�I���g���̊֘A�t��

	Slider2.l = 0.4;
	Slider2.r = 0.03;
	Slider2.m = 0.35;

	Slider2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider2.m, 1, Slider2.r, Slider2.l);
	dBodySetMass(Slider2.body,&mass);
	dBodySetPosition(Slider2.body, x0+0.10, y0-0.10, z0 -(Leg2.l-Slider2.l/2));
	Slider2.geom= dCreateCapsule(space, Slider2.r, Slider2.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider2.geom, Slider2.body);							// �{�f�B�ƃW�I���g���̊֘A�t��

	Slider3.l = 0.4;
	Slider3.r = 0.03;
	Slider3.m = 0.35;

	Slider3.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider3.m, 1, Slider3.r, Slider3.l);
	dBodySetMass(Slider3.body,&mass);
	dBodySetPosition(Slider3.body, x0-0.10, y0+0.10, z0 -(Leg3.l-Slider3.l/2));
	Slider3.geom= dCreateCapsule(space, Slider3.r, Slider3.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider3.geom, Slider3.body);							// �{�f�B�ƃW�I���g���̊֘A�t��

	Slider4.l = 0.4;
	Slider4.r = 0.03;
	Slider4.m = 0.35;

	Slider4.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider4.m, 1, Slider4.r, Slider4.l);
	dBodySetMass(Slider4.body,&mass);
	dBodySetPosition(Slider4.body, x0+0.10, y0+0.10, z0 -(Leg4.l-Slider4.l/2));
	Slider4.geom= dCreateCapsule(space, Slider4.r, Slider4.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider4.geom, Slider4.body);							// �{�f�B�ƃW�I���g���̊֘A�t��

	Joint_BL1 = dJointCreateUniversal(world, 0);
	dJointAttach (Joint_BL1, Leg1.body, Body.body);
	dJointSetUniversalAnchor(Joint_BL1, x0-0.10, y0-0.10, z0);
	dJointSetUniversalAxis1 (Joint_BL1,  0,  1,  0);
	dJointSetUniversalAxis2 (Joint_BL1,  1,  0,  0);
	dJointSetUniversalParam (Joint_BL1, dParamLoStop, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL1, dParamHiStop,  MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL1, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL1, dParamHiStop2,  MAX_LEG_ANG);

	Joint_BL2 = dJointCreateUniversal(world, 0);
	dJointAttach (Joint_BL2, Leg2.body, Body.body);
	dJointSetUniversalAnchor(Joint_BL2, x0+0.10, y0-0.10, z0);
	dJointSetUniversalAxis1 (Joint_BL2,  0,  1,  0);
	dJointSetUniversalAxis2 (Joint_BL2,  1,  0,  0);
	dJointSetUniversalParam (Joint_BL2, dParamLoStop, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL2, dParamHiStop,  MAX_LEG_ANG); 
	dJointSetUniversalParam (Joint_BL2, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL2, dParamHiStop2,  MAX_LEG_ANG);

	Joint_BL3 = dJointCreateUniversal(world, 0);
	dJointAttach (Joint_BL3, Leg3.body, Body.body);
	dJointSetUniversalAnchor(Joint_BL3, x0-0.10, y0+0.10, z0);
	dJointSetUniversalAxis1 (Joint_BL3,  0,  1,  0);
	dJointSetUniversalAxis2 (Joint_BL3,  1,  0,  0);
	dJointSetUniversalParam (Joint_BL3, dParamLoStop, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL3, dParamHiStop,  MAX_LEG_ANG); 
	dJointSetUniversalParam (Joint_BL3, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL3, dParamHiStop2,  MAX_LEG_ANG);

	Joint_BL4 = dJointCreateUniversal(world, 0);
	dJointAttach (Joint_BL4, Leg4.body, Body.body);
	dJointSetUniversalAnchor(Joint_BL4, x0+0.10, y0+0.10, z0);
	dJointSetUniversalAxis1 (Joint_BL4,  0,  1,  0);
	dJointSetUniversalAxis2 (Joint_BL4,  1,  0,  0);
	dJointSetUniversalParam (Joint_BL4, dParamLoStop, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL4, dParamHiStop,  MAX_LEG_ANG); 
	dJointSetUniversalParam (Joint_BL4, dParamLoStop2, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL4, dParamHiStop2,  MAX_LEG_ANG);

	Joint_LS1 = dJointCreateSlider(world, 0);
	dJointAttach        (Joint_LS1, Leg1.body, Slider1.body);
	dJointSetSliderAxis (Joint_LS1, 0, 0, 1);
	dJointSetSliderParam(Joint_LS1, dParamLoStop,  MIN_SLIDER_POS); 
	dJointSetSliderParam(Joint_LS1, dParamHiStop,  MAX_SLIDER_POS);

	Joint_LS2 = dJointCreateSlider(world, 0);
	dJointAttach        (Joint_LS2, Leg2.body, Slider2.body);
	dJointSetSliderAxis (Joint_LS2, 0, 0, 1);
	dJointSetSliderParam(Joint_LS2, dParamLoStop,  MIN_SLIDER_POS);
	dJointSetSliderParam(Joint_LS2, dParamHiStop,  MAX_SLIDER_POS);

	Joint_LS3 = dJointCreateSlider(world, 0);
	dJointAttach        (Joint_LS3, Leg3.body, Slider3.body);
	dJointSetSliderAxis (Joint_LS3, 0, 0, 1);
	dJointSetSliderParam(Joint_LS3, dParamLoStop,  MIN_SLIDER_POS);
	dJointSetSliderParam(Joint_LS3, dParamHiStop,  MAX_SLIDER_POS);

	Joint_LS4 = dJointCreateSlider(world, 0);
	dJointAttach        (Joint_LS4, Leg4.body, Slider4.body);
	dJointSetSliderAxis (Joint_LS4, 0, 0, 1);
	dJointSetSliderParam(Joint_LS4, dParamLoStop,  MIN_SLIDER_POS);
	dJointSetSliderParam(Joint_LS4, dParamHiStop,  MAX_SLIDER_POS);

}

void start() {
	dsSetViewpoint(XYZ, HPR);							// �J�����̐ݒ�
}

void destroyMonoBot(){
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

static void restart(){
	destroyMonoBot();
	dJointGroupDestroy(contactgroup);
	contactgroup = dJointGroupCreate(0);
	makeMonoBot();
}

void command (int cmd){
	float xyz[3], hpr[3];

	switch (cmd){
		case 'r': restart(); break;
		case 's':
			dsGetViewpoint(xyz, hpr);
			printf("xyz=%4.2f, %4.2f, %4.2f \n", xyz[0], xyz[1], xyz[2]);
			printf("hpr=%6.2f, %6.2f, %6.2f \n", hpr[0], hpr[1], hpr[2]);
			break;
		default:printf("Input r or s\n"); break;
	}
}

// �`��֐��̐ݒ�
void setDrawStuff(){
  fn.version = DS_VERSION;							// �h���[�X�^�b�t�̃o�[�W����
  fn.start   = &start;								// �O���� start�֐��̃|�C���^
  fn.step    = &simLoop;							// simLoop�֐��̃|�C���^
  fn.command = &command;							// �L�[���͂ɂ��Ăяo�������֐��̃A�h���X
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;		// �e�N�X�`��
}