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
dJointGroupID	contactgroup;				// �R���^�N�g�O���[�v
dsFunctions fn;

typedef struct {							// MyCapsuleObject�\����
	dBodyID body;							// �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;							// �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	dReal l, x, y, z, r, m;					// ���� l, x, y, z[m], ���a r[m]�C���� m[kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg1, Slider1, Leg2, Slider2;

#include "ttstatevalue.h"

static float XYZ[3] = { 0.60, -3.26, 0.80};	// ���_�̈ʒu
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
	dsSimulationLoop(argc,argv,1000, 600,&fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();								// ODE�̏I��
	
	return 0;
}

static void simLoop(int pause){					// �V�~�����[�V�������[�v
	dReal tmp_ang = 0;

	DesPos			= 42195.0;				// �ڕW�ʒu

	// ��ԗʂ̌v�Z
	cal_state_value();

	// �ڕW���x�̓��o
	DesVel =(DesPos- PosX);
	if(DesVel> MAX_VEL) DesVel= MAX_VEL;
	if(DesVel<-MAX_VEL) DesVel=-MAX_VEL;

	// �V�r���̐��䑥
	if(SKY){ 
		if(StepLeg1) {
			// Leg 1 retract, Leg 2 prepare for landing
			ForceSlider1 = -1000*MIN_SLIDER_POS;

			// Leg 2 prepare for landing
			ForceSlider2 = 1000*PosSlider2;
			DesContactPos =  (GroundTime * VelX)/2.0 + 0.07*(VelX - DesVel);
			if     ( DesContactPos/(Leg2.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG+AngPitch;
			else if( DesContactPos/(Leg2.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG+AngPitch;
			else tmp_ang = asin(DesContactPos/(Leg2.l + MAX_SLIDER_POS));
			DesLeg2Ang     =  AngPitch - tmp_ang;
			TorqueLeg2     = -k1*(AngLeg2  - DesLeg2Ang) - k2*AngVelLeg2;

			// Leg 1 at opposite angle
			DesLeg1Ang = -DesLeg2Ang;
			TorqueLeg1	 = -k1*(AngLeg1 - DesLeg1Ang) - k2*AngVelLeg1;
		}

		if(StepLeg2) {
			// Leg 2 retract, Leg 1 prepare for landing
			ForceSlider2 = -1000*MIN_SLIDER_POS;

			// Leg 1 prepare for landing
			ForceSlider1 = 1000*PosSlider2;
			DesContactPos =  (GroundTime * VelX)/2.0 + 0.07*(VelX - DesVel);
			if     ( DesContactPos/(Leg1.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG+AngPitch;
			else if( DesContactPos/(Leg1.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG+AngPitch;
			else tmp_ang = asin(DesContactPos/(Leg1.l + MAX_SLIDER_POS));
			DesLeg1Ang     =  AngPitch - tmp_ang;
			TorqueLeg1     = -k1*(AngLeg1  - DesLeg1Ang) - k2*AngVelLeg1;

			// Leg 2 at opposite angle
			DesLeg2Ang = -DesLeg1Ang;
			TorqueLeg2 	 = -k1*(AngLeg2 - DesLeg2Ang) - k2*AngVelLeg2;
		}
	}

	// �x���r���̐��䑥
	else {
		if(StepLeg1) {
			// Leg 1 touching the ground, Leg 2 still retracted
			if(VelSlider1 < 0)	ForceSlider1 = SPRING*PosSlider1;
			else				ForceSlider1 = SPRING*(PosSlider1+0.05);

			TorqueLeg1   = k3*(AngPitch)+ k4*AngVelY + k5*VelX;

			// Leg 2 at opposite angles
			DesLeg2Ang = -DesLeg1Ang;
			TorqueLeg2 	 = -k1*(AngLeg2 - DesLeg2Ang) - k2*AngVelLeg2;
			ForceSlider2 = -1000*MIN_SLIDER_POS;
		}

		if(StepLeg2) {
			// Leg 1 touching the ground, Leg 2 still retracted
			if(VelSlider2 < 0)	ForceSlider2 = SPRING*PosSlider2;
			else				ForceSlider2 = SPRING*(PosSlider2+0.05);

			TorqueLeg2	 = k3*(AngPitch)+ k4*AngVelY + k5*VelX;

			// Leg 1 at opposite angle
			DesLeg1Ang = -DesLeg2Ang;
			TorqueLeg1	 = -k1*(AngLeg1 - DesLeg1Ang) - k2*AngVelLeg1;
			ForceSlider1 = -1000*MIN_SLIDER_POS;
		}
	}
	//printf("%.3f, %.3f\n", AngLeg1, AngLeg2);
	dJointAddSliderForce(Joint_LS1, ForceSlider1);		// �X���C�_�[�ɗ͂����
	dJointAddHingeTorque(Joint_BL1, TorqueLeg1);		// �r�Ƀg���N�����
	dJointAddSliderForce(Joint_LS2, ForceSlider2);
	dJointAddHingeTorque(Joint_BL2, TorqueLeg2);

	// �\��������������߂�
	XYZ[0] = 0.60+PosX;								// ���{�b�g�̈ʒu�ɍ��킹�Ďx�_�𓮂���
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
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLeg, DesLegAng, AngVelLeg, TorqueLeg);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// 1���[�v�Ō�ɂ��鏈��
//	printf("%3.1lf\t%3.1lf\n",Time, PosX);
//	printf("AngLeg1: %3.3f\t\tDesLeg1Ang: %3.3f\tAngVelLeg1: %3.3f\tAngLeg2: %3.3f\t\tDesLeg2Ang: %3.3f\tAngVelLeg2: %3.3f\n", AngLeg1, DesLeg1Ang, AngVelLeg1, AngLeg2, DesLeg2Ang, AngVelLeg2);
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

	// �X���C�_�̕\��
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider1.body), dBodyGetRotation(Slider1.body), Slider1.l, Slider1.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider2.body), dBodyGetRotation(Slider2.body), Slider2.l, Slider2.r);
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
	if((isGround == 1) && ((Slider1.geom == o1) || (Slider1.geom == o2) || (Slider2.geom == o1) || (Slider2.geom == o2)))  State = 1;		// �ڐG�����ꍇ
	if((isGround == 0) && ((Slider1.geom == o1) || (Slider1.geom == o2) || (Slider2.geom == o1) || (Slider2.geom == o2)))  State = 0;		// �ڐG���Ă��Ȃ��ꍇ


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
	dReal x0 = 0.0, y0 = 0.0, z0 = 0.6;
	dMass mass;

	// �{�b�N�X�Ŗ{�̂𐶐�
	Body.x = 0.80;
	Body.y = 0.10;
	Body.z = 0.10;
	Body.m = 40.0;

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
	Leg1.m = 3.0;

	Leg1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg1.m, 1, Leg1.r, Leg1.l);
	dBodySetMass(Leg1.body,&mass);
	dBodySetPosition(Leg1.body, x0, y0, z0 -Leg1.l/2);
	Leg1.geom= dCreateCapsule(space, Leg1.r, Leg1.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg1.geom, Leg1.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	Leg2.l = 0.3;
	Leg2.r = 0.05;
	Leg2.m = 3.0;

	Leg2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg2.m, 1, Leg2.r, Leg2.l);
	dBodySetMass(Leg2.body,&mass);
	dBodySetPosition(Leg2.body, x0, y0, z0 -Leg2.l/2);
	Leg2.geom= dCreateCapsule(space, Leg2.r, Leg2.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg2.geom, Leg2.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	// �J�v�Z���ő��𐶐�
	Slider1.l = 0.4;
	Slider1.r = 0.03;
	Slider1.m = 0.7;

	Slider1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider1.m, 1, Slider1.r, Slider1.l);
	dBodySetMass(Slider1.body,&mass);
	dBodySetPosition(Slider1.body, x0, y0, z0 -(Leg1.l-Slider1.l/2));
	Slider1.geom= dCreateCapsule(space, Slider1.r, Slider1.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider1.geom, Slider1.body);							// �{�f�B�ƃW�I���g���̊֘A�t��

	Slider2.l = 0.4;
	Slider2.r = 0.03;
	Slider2.m = 0.7;

	Slider2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider2.m, 1, Slider2.r, Slider2.l);
	dBodySetMass(Slider2.body,&mass);
	dBodySetPosition(Slider2.body, x0, y0, z0 -(Leg2.l-Slider2.l/2));
	Slider2.geom= dCreateCapsule(space, Slider2.r, Slider2.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider2.geom, Slider2.body);							// �{�f�B�ƃW�I���g���̊֘A�t��


	Joint_BL1 = dJointCreateHinge(world, 0);
	dJointAttach        (Joint_BL1, Leg1.body, Body.body);
	dJointSetHingeAnchor(Joint_BL1, x0, y0, z0);
	dJointSetHingeAxis  (Joint_BL1,  0,  1,  0);
	dJointSetHingeParam (Joint_BL1, dParamLoStop, -MAX_LEG_ANG);
	dJointSetHingeParam (Joint_BL1, dParamHiStop,  MAX_LEG_ANG);

	Joint_BL2 = dJointCreateHinge(world, 0);
	dJointAttach        (Joint_BL2, Leg2.body, Body.body);
	dJointSetHingeAnchor(Joint_BL2, x0, y0, z0);
	dJointSetHingeAxis  (Joint_BL2,  0,  1,  0);
	dJointSetHingeParam (Joint_BL2, dParamLoStop, -MAX_LEG_ANG);
	dJointSetHingeParam (Joint_BL2, dParamHiStop,  MAX_LEG_ANG); 

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

}

void start() {
	dsSetViewpoint(XYZ, HPR);							// �J�����̐ݒ�
}

void destroyMonoBot(){
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
		default:printf("Input a or 1\n"); break;
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