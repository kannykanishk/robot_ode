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
dJointID		Joint_BL;					// 本体と脚を繋ぐ回転ジョイント
dJointID		Joint_LS;					// 脚とスライダーを繋ぐ直動ジョイント
dJointID		Joint_BS;					// 本体とセンサを繋ぐ固定ジョイント
dJointGroupID	contactgroup;				// コンタクトグループ
dsFunctions fn;

typedef struct {							// MyCapsuleObject構造体
	dBodyID body;							// ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;							// ジオメトリのID番号(衝突検出計算用）
	dReal l, x, y, z, r, m;					// 長さ l, x, y, z[m], 半径 r[m]，質量 m[kg]
} MyCapsuleObject;
MyCapsuleObject Body, Leg, Slider;

#include "ttstatevalue.h"

static float XYZ[3] = { 0.60, -3.26, 0.80 };	// 視点の位置
static float HPR[3] = { 101 , -6.50, 0    };	// 視線の方向

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
	dsSimulationLoop(argc,argv,1000,750,&fn);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();								// ODEの終了
	
	return 0;
}


static void simLoop(int pause){					// シミュレーションループ
	dReal tmp_ang = 0;

	DesPosX	= 42195.0;				// 目標位置
	DesPosY = 42195.0;

	// 状態量の計算
	cal_state_value();

	// 目標速度の導出
	DesVelX = DesPosX - PosX;
	if(DesVelX > MAX_VEL) DesVelX = MAX_VEL;
	if(DesVelX <-MAX_VEL) DesVelX =-MAX_VEL;

	DesVelY = DesPosY - PosY;
	if(DesVelY > MAX_VEL) DesVelY = MAX_VEL;
	if(DesVelY <-MAX_VEL) DesVelY =-MAX_VEL;

	// 遊脚時の制御則
	if(SKY){
		// スライダーに加える力
		ForceSlider = SPRING*PosSlider;

		// 脚に加える力
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

	// 支持脚時の制御則
	else {
		// スライダーに加える力
		if(VelSlider < 0)	ForceSlider = SPRING*PosSlider;
		else				ForceSlider = SPRING*(PosSlider+0.05);

		// 脚に加える力
		TorqueLegY   = k3Y*(AngPitch) + k4Y*AngVelY + k5Y*VelX;
		TorqueLegX   = k3X*(AngRoll)  + k4X*AngVelX + k5X*VelY;

	}

	dJointAddSliderForce(Joint_LS, ForceSlider);						// スライダーに力を入力
	dJointAddUniversalTorques(Joint_BL, TorqueLegY, TorqueLegX);		// 脚にトルクを入力

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
//	fprintf(fp_data,"%lf,%lf,%lf,%lf,", AngLegY, DesLegYAng, AngVelLegY, TorqueLegY);
//	fprintf(fp_data,"%lf,%lf,%lf,", PosSlider, VelSlider, ForceSlider);
//	fprintf(fp_data,"\n");

	// 1ループ最後にする処理
//	printf("%3.1lf\t%3.1lf\n",Time, PosX);
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
	dsSetColor(0.0,1.0,1.0);
	dsDrawCapsule(dBodyGetPosition(Leg.body),    dBodyGetRotation(Leg.body),    Leg.l,    Leg.r   );

	// スライダの表示
	dsSetColor(0.0,0.0,1.0); 
	dsDrawCapsule(dBodyGetPosition(Slider.body), dBodyGetRotation(Slider.body), Slider.l, Slider.r);
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
	if((isGround == 1) && ((Slider.geom == o1) || (Slider.geom == o2))) State= 1;	// 接触した場合
	if((isGround == 0) && ((Slider.geom == o1) || (Slider.geom == o2))) State= 0;		// 接触していない場合

	// 衝突情報の生成 nは衝突点数
	n= dCollide(o1, o2 ,N ,&contact[0].geom, sizeof(dContact));
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
	dReal x0 = 0.0, y0 = 0.0, z0 = 1.0;
	dMass mass;

	// ボックスで本体を生成
	Body.x = 0.60;
	Body.y = 0.60;
	Body.z = 0.10;
	Body.m = 10.0;

	Body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, Body.m, Body.x, Body.y, Body.z);
	dBodySetMass(Body.body,&mass);
	dBodySetPosition(Body.body, x0, y0, z0);
	Body.geom= dCreateBox(space, Body.x, Body.y, Body.z);			// ボックスでジオメトリの生成
	dGeomSetBody(Body.geom, Body.body);		

	// カプセルで脚を生成
	Leg.l = 0.3;
	Leg.r = 0.05;
	Leg.m = 3.0;

	Leg.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Leg.m, 1, Leg.r, Leg.l);
	dBodySetMass(Leg.body,&mass);
	dBodySetPosition(Leg.body, x0, y0, z0 -Leg.l/2);
	Leg.geom= dCreateCapsule(space, Leg.r, Leg.l);					// カプセルのジオメトリの生成
	dGeomSetBody(Leg.geom, Leg.body);								// ボディとジオメトリの関連付け

	// カプセルで足を生成
	Slider.l = 0.4;
	Slider.r = 0.03;
	Slider.m = 0.7;

	Slider.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCapsuleTotal(&mass, Slider.m, 1, Slider.r, Slider.l);
	dBodySetMass(Slider.body,&mass);
	dBodySetPosition(Slider.body, x0, y0, z0 -(Leg.l-Slider.l/2));
	Slider.geom= dCreateCapsule(space, Slider.r, Slider.l);			// カプセルのジオメトリの生成
	dGeomSetBody(Slider.geom, Slider.body);							// ボディとジオメトリの関連付け


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
  dsSetViewpoint(XYZ, HPR);							// カメラの設定
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

// 描画関数の設定
void setDrawStuff(){
  fn.version = DS_VERSION;							// ドロースタッフのバージョン
  fn.start   = &start;								// 前処理 start関数のポインタ
  fn.step    = &simLoop;							// simLoop関数のポインタ
  fn.command = &command;							// キー入力により呼び出しされる関数のアドレス
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;		// テクスチャ
}


