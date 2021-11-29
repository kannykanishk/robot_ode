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
dJointID		Joint_BL;					// �{�̂Ƌr���q����]�W���C���g
dJointID		Joint_LS;					// �r�ƃX���C�_�[���q�������W���C���g
dJointID		Joint_BS;					// �{�̂ƃZ���T���q���Œ�W���C���g
dJointGroupID	contactgroup;				// �R���^�N�g�O���[�v
dsFunctions fn;

typedef struct {							// MyCapsuleObject�\����
	dBodyID body;							// �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;							// �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	dReal l, x, y, z, r, m;					// ���� l, x, y, z[m], ���a r[m]�C���� m[kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg, Slider;

#include "ttstatevalue.h"

static float XYZ[3] = { 0.60, -3.26, 0.80 };	// ���_�̈ʒu
static float HPR[3] = { 101 , -6.50, 0    };	// �����̕���

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
	dsSimulationLoop(argc,argv,1000,750,&fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();								// ODE�̏I��
	
	return 0;
}


static void simLoop(int pause){					// �V�~�����[�V�������[�v
	dReal tmp_ang = 0;

	DesPosX	= 42195.0;				// �ڕW�ʒu
	DesPosY = 42195.0;

	// ��ԗʂ̌v�Z
	cal_state_value();

	// �ڕW���x�̓��o
	DesVelX = DesPosX - PosX;
	if(DesVelX > MAX_VEL) DesVelX = MAX_VEL;
	if(DesVelX <-MAX_VEL) DesVelX =-MAX_VEL;

	DesVelY = DesPosY - PosY;
	if(DesVelY > MAX_VEL) DesVelY = MAX_VEL;
	if(DesVelY <-MAX_VEL) DesVelY =-MAX_VEL;

	// �V�r���̐��䑥
	if(SKY){
		// �X���C�_�[�ɉ������
		ForceSlider = SPRING*PosSlider;

		// �r�ɉ������
		DesContactPosX = (GroundTime * VelX)/2.0 + 0.07*(VelX - DesVelX);
		DesContactPosY = (GroundTime * VelY)/2.0 + 0.07*(VelY - DesVelY);

		if     ( DesContactPosX/(Leg.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngPitch) ) tmp_ang =  MAX_LEG_ANG+AngPitch;
		else if( DesContactPosX/(Leg.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngPitch) ) tmp_ang = -MAX_LEG_ANG+AngPitch;
		else tmp_ang = asin(DesContactPosX/(Leg.l + MAX_SLIDER_POS));
		DesLegYAng     =  AngPitch - tmp_ang;

		if     ( DesContactPosY/(Leg.l + MAX_SLIDER_POS) > sin( MAX_LEG_ANG+AngRoll) ) tmp_ang =  MAX_LEG_ANG+AngRoll;
		else if( DesContactPosY/(Leg.l + MAX_SLIDER_POS) < sin(-MAX_LEG_ANG+AngRoll) ) tmp_ang = -MAX_LEG_ANG+AngRoll;
		else tmp_ang = asin(DesContactPosY/(Leg.l + MAX_SLIDER_POS));
		DesLegXAng     =  AngRoll - tmp_ang;

		TorqueLegY     = -k1Y*(AngLegY  - DesLegYAng) - k2Y*AngVelLegY;
		TorqueLegX     = -k1X*(AngLegX  - DesLegXAng) - k2X*AngVelLegX;

	}

	// �x���r���̐��䑥
	else {
		// �X���C�_�[�ɉ������
		if(VelSlider < 0)	ForceSlider = SPRING*PosSlider;
		else				ForceSlider = SPRING*(PosSlider+0.05);

		// �r�ɉ������
		TorqueLegY   = k3Y*(AngPitch) + k4Y*AngVelY + k5Y*VelX;
		TorqueLegX   = k3X*(AngRoll)  + k4X*AngVelX + k5X*VelY;

	}

	dJointAddSliderForce(Joint_LS, ForceSlider);						// �X���C�_�[�ɗ͂����
	dJointAddUniversalTorques(Joint_BL, TorqueLegY, TorqueLegX);		// �r�Ƀg���N�����

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
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLegY, DesLegYAng, AngVelLegY, TorqueLegY);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// 1���[�v�Ō�ɂ��鏈��
//	printf("%3.1lf\t%3.1lf\n",Time, PosX);
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
	dsSetColor(0.0,1.0,1.0);
	dsDrawCapsule(dBodyGetPosition(Leg.body),    dBodyGetRotation(Leg.body),    Leg.l,    Leg.r   );

	// �X���C�_�̕\��
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider.body), dBodyGetRotation(Slider.body), Slider.l, Slider.r);
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
	if((isGround == 1) && ((Slider.geom == o1) || (Slider.geom == o2))) State= 1;	// �ڐG�����ꍇ
	if((isGround == 0) && ((Slider.geom == o1) || (Slider.geom == o2))) State= 0;		// �ڐG���Ă��Ȃ��ꍇ

	// �Փˏ��̐��� n�͏Փ˓_��
	n= dCollide(o1, o2 ,N ,&contact[0].geom, sizeof(dContact));
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
	dReal x0 = 0.0, y0 = 0.0, z0 = 1.0;
	dMass mass;

	// �{�b�N�X�Ŗ{�̂𐶐�
	Body.x = 0.60;
	Body.y = 0.60;
	Body.z = 0.10;
	Body.m = 10.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);			// �{�b�N�X�ŃW�I���g���̐���
	dGeomSetBody(Body.geom, Body.body);		

	// �J�v�Z���ŋr�𐶐�
	Leg.l = 0.3;
	Leg.r = 0.05;
	Leg.m = 3.0;

	Leg.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg.m, 1, Leg.r, Leg.l);
	dBodySetMass(Leg.body,&mass);
	dBodySetPosition(Leg.body, x0, y0, z0 -Leg.l/2);
	Leg.geom= dCreateCapsule(space, Leg.r, Leg.l);					// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Leg.geom, Leg.body);								// �{�f�B�ƃW�I���g���̊֘A�t��

	// �J�v�Z���ő��𐶐�
	Slider.l = 0.4;
	Slider.r = 0.03;
	Slider.m = 0.7;

	Slider.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider.m, 1, Slider.r, Slider.l);
	dBodySetMass(Slider.body,&mass);
	dBodySetPosition(Slider.body, x0, y0, z0 -(Leg.l-Slider.l/2));
	Slider.geom= dCreateCapsule(space, Slider.r, Slider.l);			// �J�v�Z���̃W�I���g���̐���
	dGeomSetBody(Slider.geom, Slider.body);							// �{�f�B�ƃW�I���g���̊֘A�t��


	Joint_BL = dJointCreateUniversal(world, 0);
	dJointAttach        (Joint_BL, Leg.body, Body.body);
	dJointSetUniversalAnchor(Joint_BL, x0, y0, z0);
	dJointSetUniversalAxis1  (Joint_BL,  0,  1,  0);
	dJointSetUniversalAxis2  (Joint_BL,  1,  0,  0);
	dJointSetUniversalParam (Joint_BL, dParamLoStop, -MAX_LEG_ANG);
	dJointSetUniversalParam (Joint_BL, dParamHiStop,  MAX_LEG_ANG);  

	Joint_LS = dJointCreateSlider(world, 0);
	dJointAttach        (Joint_LS, Leg.body, Slider.body);
	dJointSetSliderAxis (Joint_LS, 0, 0, 1);
	dJointSetSliderParam(Joint_LS, dParamLoStop,  MIN_SLIDER_POS);
	dJointSetSliderParam(Joint_LS, dParamHiStop,  MAX_SLIDER_POS);
}

void start() {
  dsSetViewpoint(XYZ, HPR);							// �J�����̐ݒ�
}

void destroyMonoBot(){
	dJointDestroy(Joint_BL);
	dJointDestroy(Joint_LS);
	dBodyDestroy(Body.body);
	dBodyDestroy(Leg.body);
	dBodyDestroy(Slider.body);
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


