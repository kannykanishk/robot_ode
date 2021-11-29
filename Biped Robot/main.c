#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#include "ttconstant100.h"
#include "ttsystem.h"
#include "ttcontrol100.h"

#ifdef dDOUBLE								// 単精度と倍精度の両方に対応するためのおまじない
	#define dsDrawBox	dsDrawBoxD
	#define dsDrawSphere	dsDrawSphereD
	#define dsDrawCylinder	dsDrawCylinderD
	#define dsDrawCapsule	dsDrawCapsuleD
#endif

dWorldID		world;						// 動力学計算用ワールド
dSpaceID		space;						// 衝突検出用スペース
dGeomID			ground;						// 地面
dJointID		Joint_BL1;					// 本体と脚を繋ぐ回転ジョイント
dJointID		Joint_LS1;					// 脚とスライダーを繋ぐ直動ジョイント
dJointID		Joint_BS;					// 本体とセンサを繋ぐ固定ジョイント
dJointID		Joint_BL2;
dJointID		Joint_LS2;
dJointGroupID	contactgroup;				// コンタクトグループ
dsFunctions fn;

typedef struct {							// MyCapsuleObject構造体
	dBodyID body;							// ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;							// ジオメトリのID番号(衝突検出計算用）
	dReal l, x, y, z, r, m;					// 長さ l, x, y, z[m], 半径 r[m]，質量 m[kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg1, Slider1, Leg2, Slider2;

#include "ttstatevalue.h"

static float XYZ[3] = { 0.60, -3.26, 0.80};	// 視点の位置
static float HPR[3] = { 101 , -6.50, 0   };	// 視線の方向

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
	dInitODE();									// ODEの初期化
	world = dWorldCreate();
	dWorldSetGravity(world,0,0,-9.8);			//　重力の設定
	space        = dHashSpaceCreate(0);			// 衝突用空間の創造
	contactgroup = dJointGroupCreate(0);		// ジョイントグループの生成
	ground = dCreatePlane(space, 0, 0, 1, 0);	// 平面ジオメトリの生成
	makeMonoBot();								// MonoBotの作成
	dsSimulationLoop(argc,argv,1000, 600,&fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();								// ODEの終了
	
	return 0;
}

static void simLoop(int pause){					// シミュレーションループ
	dReal tmp_ang = 0;

	DesPos			= 42195.0;				// 目標位置

	// 状態量の計算
	cal_state_value();

	// 目標速度の導出
	DesVel =(DesPos- PosX);
	if(DesVel> MAX_VEL) DesVel= MAX_VEL;
	if(DesVel<-MAX_VEL) DesVel=-MAX_VEL;

	// 遊脚時の制御則
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

	// 支持脚時の制御則
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
	dJointAddSliderForce(Joint_LS1, ForceSlider1);		// スライダーに力を入力
	dJointAddHingeTorque(Joint_BL1, TorqueLeg1);		// 脚にトルクを入力
	dJointAddSliderForce(Joint_LS2, ForceSlider2);
	dJointAddHingeTorque(Joint_BL2, TorqueLeg2);

	// 表示を見る方向を定める
	XYZ[0] = 0.60+PosX;								// ロボットの位置に合わせて支点を動かす
	dsSetViewpoint(XYZ, HPR);						// カメラの設定

	// お決まりの処理
	dSpaceCollide(space,0,&nearCallback);			// 衝突検出関数
	dWorldStep(world, S_TIME);
	dJointGroupEmpty(contactgroup);					// ジョイントグループを空にする
	show_robot();									// ロボットを表示

	// シミュレーション結果をファイルに保存
//	fprintf(fp_data,"%lf,", Time);
//	fprintf(fp_data,"%d,%lf,%lf,", state, f_state, GroundTime);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosX, PosY, PosZ);
//	fprintf(fp_data,"%lf,%lf,%lf,", VelX, VelY, VelZ);
//	fprintf(fp_data,"%lf,%lf,%lf,", AngRoll*180/M_PI, AngPitch*180/M_PI, AngYaw*180/M_PI);
//	fprintf(fp_data,"%lf,%lf,%lf,", AngVelX*180/M_PI, AngVelY*180/M_PI,  AngVelZ*180/M_PI);
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLeg, DesLegAng, AngVelLeg, TorqueLeg);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// 1ループ最後にする処理
//	printf("%3.1lf\t%3.1lf\n",Time, PosX);
//	printf("AngLeg1: %3.3f\t\tDesLeg1Ang: %3.3f\tAngVelLeg1: %3.3f\tAngLeg2: %3.3f\t\tDesLeg2Ang: %3.3f\tAngVelLeg2: %3.3f\n", AngLeg1, DesLeg1Ang, AngVelLeg1, AngLeg2, DesLeg2Ang, AngVelLeg2);
	Time +=S_TIME;
}

void show_robot(void){
	dReal sides[3];

	// 本体の表示
	dsSetColor(0.0,1.0,0.0); 
	sides[0]=Body.x;
	sides[1]=Body.y;
	sides[2]=Body.z;
	dsDrawBox(dBodyGetPosition(Body.body),  dBodyGetRotation(Body.body),   sides);

	// 脚の表示
	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg1.body),    dBodyGetRotation(Leg1.body),    Leg1.l,    Leg1.r   );

	dsSetColor(0.0,1.0,0.0);
	dsDrawCapsule(dBodyGetPosition(Leg2.body),    dBodyGetRotation(Leg2.body),    Leg2.l,    Leg2.r   );

	// スライダの表示
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider1.body), dBodyGetRotation(Slider1.body), Slider1.l, Slider1.r);

	dsSetColor(1.0,0.0,0.0); 
	dsDrawCapsule(dBodyGetPosition(Slider2.body), dBodyGetRotation(Slider2.body), Slider2.l, Slider2.r);
}

// コールバック関数
static void nearCallback(void *data, dGeomID o1, dGeomID o2){
	static const int N = 10;						// 接触点数の最大値
	dContact contact[N];							// 接触点
	dJointID c;
	int isGround;
	int n;
	int i;

	// 接触している物体のどちらかが地面ならisGroundに非0をセット
	isGround = ((ground == o1) || (ground == o2)); 

	// スライダーと地面が接触しているかを判定
	if((isGround == 1) && ((Slider1.geom == o1) || (Slider1.geom == o2) || (Slider2.geom == o1) || (Slider2.geom == o2)))  State = 1;		// 接触した場合
	if((isGround == 0) && ((Slider1.geom == o1) || (Slider1.geom == o2) || (Slider2.geom == o1) || (Slider2.geom == o2)))  State = 0;		// 接触していない場合


	// 衝突情報の生成 nは衝突点数
	n= dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if(isGround == 1){

		for(i= 0; i < n; i++){
			contact[i].surface.mode			= dContactBounce;	// 接触面の反発性を設定
			contact[i].surface.bounce		= 0.0;				// 反発係数(0.0から1.0)
			contact[i].surface.bounce_vel	= 0.0;				// 反発に必要な最低速度
			contact[i].surface.mu			= dInfinity;		// 反発に必要な最低速度
			
			// 接触ジョイントの生成
			c = dJointCreateContact(world, contactgroup, &contact[i]);
			
			// 接触している２つの剛体を接触ジョイントにより拘束
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
		}
	}
}

static void makeMonoBot(){
	dReal x0 = 0.0, y0 = 0.0, z0 = 0.6;
	dMass mass;

	// ボックスで本体を生成
	Body.x = 0.80;
	Body.y = 0.10;
	Body.z = 0.10;
	Body.m = 40.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);			// ボックスでジオメトリの生成
	dGeomSetBody(Body.geom, Body.body);		


	// カプセルで脚を生成
	Leg1.l = 0.3;
	Leg1.r = 0.05;
	Leg1.m = 3.0;

	Leg1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg1.m, 1, Leg1.r, Leg1.l);
	dBodySetMass(Leg1.body,&mass);
	dBodySetPosition(Leg1.body, x0, y0, z0 -Leg1.l/2);
	Leg1.geom= dCreateCapsule(space, Leg1.r, Leg1.l);					// カプセルのジオメトリの生成
	dGeomSetBody(Leg1.geom, Leg1.body);								// ボディとジオメトリの関連付け

	Leg2.l = 0.3;
	Leg2.r = 0.05;
	Leg2.m = 3.0;

	Leg2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg2.m, 1, Leg2.r, Leg2.l);
	dBodySetMass(Leg2.body,&mass);
	dBodySetPosition(Leg2.body, x0, y0, z0 -Leg2.l/2);
	Leg2.geom= dCreateCapsule(space, Leg2.r, Leg2.l);					// カプセルのジオメトリの生成
	dGeomSetBody(Leg2.geom, Leg2.body);								// ボディとジオメトリの関連付け

	// カプセルで足を生成
	Slider1.l = 0.4;
	Slider1.r = 0.03;
	Slider1.m = 0.7;

	Slider1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider1.m, 1, Slider1.r, Slider1.l);
	dBodySetMass(Slider1.body,&mass);
	dBodySetPosition(Slider1.body, x0, y0, z0 -(Leg1.l-Slider1.l/2));
	Slider1.geom= dCreateCapsule(space, Slider1.r, Slider1.l);			// カプセルのジオメトリの生成
	dGeomSetBody(Slider1.geom, Slider1.body);							// ボディとジオメトリの関連付け

	Slider2.l = 0.4;
	Slider2.r = 0.03;
	Slider2.m = 0.7;

	Slider2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider2.m, 1, Slider2.r, Slider2.l);
	dBodySetMass(Slider2.body,&mass);
	dBodySetPosition(Slider2.body, x0, y0, z0 -(Leg2.l-Slider2.l/2));
	Slider2.geom= dCreateCapsule(space, Slider2.r, Slider2.l);			// カプセルのジオメトリの生成
	dGeomSetBody(Slider2.geom, Slider2.body);							// ボディとジオメトリの関連付け


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
	dsSetViewpoint(XYZ, HPR);							// カメラの設定
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

// 描画関数の設定
void setDrawStuff(){
  fn.version = DS_VERSION;							// ドロースタッフのバージョン
  fn.start   = &start;								// 前処理 start関数のポインタ
  fn.step    = &simLoop;							// simLoop関数のポインタ
  fn.command = &command;							// キー入力により呼び出しされる関数のアドレス
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;		// テクスチャ
}