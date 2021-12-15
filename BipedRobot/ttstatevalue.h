#define SKY1			(StateFilter1	  <  StateTh)
#define SKY2			(StateFilter2	  <  StateTh)
#define GROUND1			(StateFilter1	  >= StateTh)
#define GROUND2			(StateFilter2	  >= StateTh)
#define SKY_PAST1		(StateFilterPast1 <  StateTh)
#define SKY_PAST2		(StateFilterPast2 <  StateTh)
#define GROUND_PAST1	(StateFilterPast1 >= StateTh)
#define GROUND_PAST2	(StateFilterPast2 >= StateTh)

double	Time			 = 0;
int		State1			 = 0;				// 脚1について  遊脚:0 か 支持脚:1 の状態を示す
int		State2			 = 0;				// 脚２について  遊脚:0 か 支持脚:1 の状態を示す
double	StateFilter1		 = 0;				// 脚1について  遊脚:0 からスタート Stateにlpフィルタを通したもの　チャタリング防止
double	StateFilter2		 = 0;				// 脚２について  遊脚:0 からスタート Stateにlpフィルタを通したもの　チャタリング防止
double	StateFilterPast1 = 0;				// 過去のフィルタを通した状態
double	StateFilterPast2 = 0;				// 過去のフィルタを通した状態
double	StateTh			 = 0.2;				// この値を超えると指示脚と判断する

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
dReal AngLeg1		= 0;					// 脚の角度	
dReal AngVelLeg1	= 0;					// 脚の角速度
dReal TorqueLeg1	= 0;					// 脚に加えるトルク

dReal AngLeg2		= 0;					// 脚の角度	
dReal AngVelLeg2	= 0;					// 脚の角速度
dReal TorqueLeg2	= 0;					// 脚に加えるトルク

// スライダー
dReal PosSlider1	= 0;					// スライダの位置	
dReal VelSlider1	= 0;					// スライダの速度
dReal ForceSlider1	= 0;					// スライダに加える力

dReal PosSlider2	= 0;					// スライダの位置	
dReal VelSlider2	= 0;					// スライダの速度
dReal ForceSlider2	= 0;					// スライダに加える力

// 本体の位置(実際には計測できない)
dReal PosX			= 0;					// 本体の位置x
dReal PosY			= 0;					// 本体の位置y
dReal PosZ			= 0;					// 本体の位置z

dReal VelX			= 0;					// 本体の速度x
dReal VelY			= 0;					// 本体の速度y
dReal VelZ			= 0;					// 本体の速度z

// 目標値
dReal DesVel		= 0;					// 目標速度
dReal DesPos		= 0;					// 目標位置
dReal DesLeg1Ang	= 0;					// 脚の目標角度
dReal DesLeg2Ang	= 0;					// 脚の目標角度
dReal DesContactPos = 0;					// 脚の目標接地位置

// その他
dReal GroundTime1	= 0.096;					// 支持脚時間(自動更新される)
dReal GroundTime2	= 0.096;					// 支持脚時間(自動更新される)


void cal_state_value(void){
	static double time_tmp1 = 0;
	static double time_tmp2 = 0;
	
	// IMUから取得を想定
	const dReal *R	= dBodyGetRotation(Body.body);			// 本体の回転行列
	const dReal *pos= dBodyGetPosition(Body.body);			// 本体の位置

	AngRoll	   = atan2( R[9], R[10]);						// ロール角
	AngPitch   = atan2(-R[8], sqrt(R[9]*R[9]+R[10]*R[10]));	// ピッチ角
	AngYaw     = atan2( R[4], R[0] );						// ヨー角

	AngVelX    = cal_d_1st_filter(0, AngRoll , OMEGA);		// x軸角速度
	AngVelY    = cal_d_1st_filter(1, AngPitch, OMEGA);		// y軸角速度
	AngVelZ    = cal_d_1st_filter(2, AngYaw  , OMEGA);		// z軸角速度
	
	PosX =pos[0];											// 位置x　（実際には計測できない）
	PosY =pos[1];											// 位置y　（実際には計測できない）
	PosZ =pos[2];											// 位置z　（実際には計測できない）

	VelX 	   = cal_d_1st_filter(3, PosX, OMEGA);			// 速度x　（実際には計測できない）
	VelY 	   = cal_d_1st_filter(4, PosY, OMEGA);			// 速度y　（実際には計測できない）
	VelZ 	   = cal_d_1st_filter(5, PosZ, OMEGA);			// 速度z　（実際には計測できない）
	
	AccX       = cal_d_1st_filter(6 , VelX , OMEGA);		// x軸加速度　(ロボットの進行方向)
	AccY       = cal_d_1st_filter(7 , VelY , OMEGA);		// y軸加速度　(ロボットの横方向)
	AccZ       = cal_d_1st_filter(8 , VelZ , OMEGA);		// z軸加速度	

	// 脚の状態量
	AngLeg1    = dJointGetHingeAngle(Joint_BL1);			// 脚の角度
	AngVelLeg1 = cal_d_1st_filter(9, AngLeg1, OMEGA);		// 脚の角速度

	AngLeg2    = dJointGetHingeAngle(Joint_BL2);			// 脚の角度
	AngVelLeg2 = cal_d_1st_filter(10, AngLeg2, OMEGA);		// 脚の角速度

	// スライダー
	PosSlider1 = dJointGetSliderPosition(Joint_LS1);		// スライダーの位置
	VelSlider1 = cal_d_1st_filter(11, PosSlider1, OMEGA);	// スライダーの速度

	PosSlider2 = dJointGetSliderPosition(Joint_LS2);		// スライダーの位置
	VelSlider2 = cal_d_1st_filter(12, PosSlider2, OMEGA);	// スライダーの速度
	
	// 遊脚・支持脚の判断
	StateFilterPast1 = StateFilter1;
	StateFilterPast2 = StateFilter2;
	StateFilter1 = cal_lp_1st_filter(0, (double)(State1), 200.0);
	StateFilter2 = cal_lp_1st_filter(1, (double)(State2), 200.0);
	
	if(SKY1 && GROUND_PAST1) GroundTime1 = Time - time_tmp1;	// 脚１が遊脚になった   支持脚時間の計算
	if(GROUND1 && SKY_PAST1) time_tmp1   = Time;				// 脚１が支持脚になった 支持脚時間の計算に利用
	if(SKY2 && GROUND_PAST2) GroundTime2 = Time - time_tmp2;	// 脚2が遊脚になった   支持脚時間の計算
	if(GROUND2 && SKY_PAST2) time_tmp2   = Time;				// 脚2が支持脚になった 支持脚時間の計算に利用
	
}
