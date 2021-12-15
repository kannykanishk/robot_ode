#define SKY				(StateFilter	 <  StateTh)
#define GROUND			(StateFilter	 >= StateTh)
#define SKY_PAST		(StateFilterPast <  StateTh)
#define GROUND_PAST		(StateFilterPast >= StateTh)

double	Time			= 0;
int		State			= 0;				// 遊脚:0 か 支持脚:1 の状態を示す
double	StateFilter		= 0;				// 遊脚:0 からスタート Stateにlpフィルタを通したもの　チャタリング防止
double	StateFilterPast	= 0;				// 過去のフィルタを通した状態
double	StateTh			= 0.2;				// この値を超えると指示脚と判断する

int LegChecker = 0;
int StepLeg1 = 0;							// If Leg1 touches ground, this value is 1
int StepLeg2 = 1;							// If Leg2 touches ground, this value is 1

//------- 状態量 -------
// 本体の姿勢センサ
dReal AngRoll		= 0;					// ロール角
dReal AngPitch		= 0;					// ピッチ角
dReal AngYaw		= 0;					// ヨー角

dReal AngVelX		= 0;					// X軸角速度
dReal AngVelY		= 0;					// Y軸角速度
dReal AngVelZ		= 0;					// Z軸角速度

dReal AccX			= 0;					// X軸加速度
dReal AccY			= 0;					// Y軸加速度
dReal AccZ			= 0;					// Z軸加速度

// 脚
dReal AngLegY1		= 0;					// 脚の角度	
dReal AngVelLegY1	= 0;					// 脚の角速度
dReal TorqueLegY1	= 0;					// 脚に加えるトルク
dReal AngLegX1		= 0;					// 脚の角度	
dReal AngVelLegX1	= 0;					// 脚の角速度
dReal TorqueLegX1	= 0;					// 脚に加えるトルク


dReal AngLegY2		= 0;					// 脚の角度	
dReal AngVelLegY2	= 0;					// 脚の角速度
dReal TorqueLegY2	= 0;					// 脚に加えるトルク
dReal AngLegX2		= 0;					// 脚の角度	
dReal AngVelLegX2	= 0;					// 脚の角速度
dReal TorqueLegX2	= 0;					// 脚に加えるトルク

dReal AngLegY3		= 0;					// 脚の角度	
dReal AngVelLegY3	= 0;					// 脚の角速度
dReal TorqueLegY3	= 0;					// 脚に加えるトルク
dReal AngLegX3		= 0;					// 脚の角度	
dReal AngVelLegX3	= 0;					// 脚の角速度
dReal TorqueLegX3	= 0;					// 脚に加えるトルク

dReal AngLegY4		= 0;					// 脚の角度	
dReal AngVelLegY4	= 0;					// 脚の角速度
dReal TorqueLegY4	= 0;					// 脚に加えるトルク
dReal AngLegX4		= 0;					// 脚の角度	
dReal AngVelLegX4	= 0;					// 脚の角速度
dReal TorqueLegX4	= 0;					// 脚に加えるトルク

// スライダー
dReal PosSlider1	= 0;					// スライダの位置	
dReal VelSlider1	= 0;					// スライダの速度
dReal ForceSlider1	= 0;					// スライダに加える力

dReal PosSlider2	= 0;					// スライダの位置	
dReal VelSlider2	= 0;					// スライダの速度
dReal ForceSlider2	= 0;					// スライダに加える力

dReal PosSlider3	= 0;					// スライダの位置	
dReal VelSlider3	= 0;					// スライダの速度
dReal ForceSlider3	= 0;					// スライダに加える力

dReal PosSlider4	= 0;					// スライダの位置	
dReal VelSlider4	= 0;					// スライダの速度
dReal ForceSlider4	= 0;					// スライダに加える力

// 本体の位置(実際には計測できない)
dReal PosX			= 0;					// 本体の位置x
dReal PosY			= 0;					// 本体の位置y
dReal PosZ			= 0;					// 本体の位置z

dReal VelX			= 0;					// 本体の速度x
dReal VelY			= 0;					// 本体の速度y
dReal VelZ			= 0;					// 本体の速度z

// 目標値
dReal DesVelX		= 0;					// 目標速度
dReal DesVelY		= 0;
dReal DesPosX		= 0;					// 目標位置
dReal DesPosY		= 0;
dReal DesLegY1Ang	= 0;					// 脚の目標角度
dReal DesLegY2Ang	= 0;					// 脚の目標角度
dReal DesLegY3Ang	= 0;					// 脚の目標角度
dReal DesLegY4Ang	= 0;					// 脚の目標角度
dReal DesLegX1Ang	= 0;					// 脚の目標角度
dReal DesLegX2Ang	= 0;					// 脚の目標角度
dReal DesLegX3Ang	= 0;					// 脚の目標角度
dReal DesLegX4Ang	= 0;					// 脚の目標角度
dReal DesContactPosY = 0;					// 脚の目標接地位置
dReal DesContactPosX = 0;					// 脚の目標接地位置

// その他
dReal GroundTime	= 0.16;					// 支持脚時間(自動更新される)


void cal_state_value(void){
	static double time_tmp = 0;
	
	// IMUから取得を想定
	const dReal *R	= dBodyGetRotation(Body.body);			// 本体の回転行列
	const dReal *pos= dBodyGetPosition(Body.body);			// 本体の位置

	AngRoll	 = atan2( R[9], R[10]);							// ロール角
	AngPitch = atan2(-R[8], sqrt(R[9]*R[9]+R[10]*R[10]));	// ピッチ角
	AngYaw   = atan2( R[4], R[0] );							// ヨー角

	AngVelX  = cal_d_1st_filter(0, AngRoll , OMEGA);		// x軸角速度
	AngVelY  = cal_d_1st_filter(1, AngPitch, OMEGA);		// y軸角速度
	AngVelZ  = cal_d_1st_filter(2, AngYaw  , OMEGA);		// z軸角速度
	
	PosX =pos[0];											// 位置x　（実際には計測できない）
	PosY =pos[1];											// 位置y　（実際には計測できない）
	PosZ =pos[2];											// 位置z　（実際には計測できない）

	VelX = cal_d_1st_filter(3, PosX, OMEGA);				// 速度x　（実際には計測できない）
	VelY = cal_d_1st_filter(4, PosY, OMEGA);				// 速度y　（実際には計測できない）
	VelZ = cal_d_1st_filter(5, PosZ, OMEGA);				// 速度z　（実際には計測できない）
	
	AccX     = cal_d_1st_filter(6 , VelX , OMEGA);			// x軸加速度　(ロボットの進行方向)
	AccY     = cal_d_1st_filter(7 , VelY , OMEGA);			// y軸加速度　(ロボットの横方向)
	AccZ     = cal_d_1st_filter(8, VelZ , OMEGA);			// z軸加速度	

	// 脚の状態量
	AngLegX1    = dJointGetHingeAngle(Joint_BL1);			// 脚の角度
	AngVelLegX1 = cal_d_1st_filter(9, AngLegX1, OMEGA);		// 脚の角速度
	
	AngLegX2    = dJointGetHingeAngle(Joint_BL2);			// 脚の角度
	AngVelLegX2 = cal_d_1st_filter(10, AngLegX2, OMEGA);	// 脚の角速度

	AngLegX3    = dJointGetHingeAngle(Joint_BL3);			// 脚の角度
	AngVelLegX3 = cal_d_1st_filter(11, AngLegX3, OMEGA);	// 脚の角速度

	AngLegX4    = dJointGetHingeAngle(Joint_BL4);			// 脚の角度
	AngVelLegX4 = cal_d_1st_filter(12, AngLegX4, OMEGA);	// 脚の角速度

	// スライダー
	PosSlider1 = dJointGetSliderPosition(Joint_LS1);			// スライダーの位置
	VelSlider1 = cal_d_1st_filter(13, PosSlider1, OMEGA);		// スライダーの速度

	PosSlider2 = dJointGetSliderPosition(Joint_LS2);			// スライダーの位置
	VelSlider2 = cal_d_1st_filter(14, PosSlider2, OMEGA);		// スライダーの速度

	PosSlider3 = dJointGetSliderPosition(Joint_LS3);			// スライダーの位置
	VelSlider3 = cal_d_1st_filter(15, PosSlider3, OMEGA);		// スライダーの速度

	PosSlider4 = dJointGetSliderPosition(Joint_LS4);			// スライダーの位置
	VelSlider4 = cal_d_1st_filter(16, PosSlider4, OMEGA);		// スライダーの速度
	
	// 遊脚・支持脚の判断
	StateFilterPast = StateFilter;
	StateFilter = cal_lp_1st_filter(0, (double)(State), 50.0);
	
	if(SKY && GROUND_PAST) { 
		GroundTime = Time - time_tmp;
		LegChecker += 1; 
	}	// 遊脚になった   支持脚時間の計算
	if(GROUND && SKY_PAST) { 
		time_tmp   = Time;
		LegChecker += 1; 
	}				// 支持脚になった 支持脚時間の計算に利用

	if((LegChecker%4 == 0) || (LegChecker%4 == 3)) {			
		StepLeg1 = 0;	StepLeg2 = 1;
	}
	else {								
		StepLeg1 = 1; StepLeg2 = 0;
	}
}
