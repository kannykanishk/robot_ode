#define SKY				(StateFilter	 <  StateTh)
#define GROUND			(StateFilter	 >= StateTh)
#define SKY_PAST		(StateFilterPast <  StateTh)
#define GROUND_PAST		(StateFilterPast >= StateTh)

double	Time			= 0;
int		State			= 0;				// 遊脚:0 か 支持脚:1 の状態を示す
double	StateFilter		= 0;				// 遊脚:0 からスタート Stateにlpフィルタを通したもの　チャタリング防止
double	StateFilterPast	= 0;				// 過去のフィルタを通した状態
double	StateTh			= 0.2;				// この値を超えると指示脚と判断する


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
dReal AngLeg		= 0;					// 脚の角度	
dReal AngVelLeg		= 0;					// 脚の角速度
dReal TorqueLeg		= 0;					// 脚に加えるトルク

// スライダー
dReal PosSlider		= 0;					// スライダの位置	
dReal VelSlider		= 0;					// スライダの速度
dReal ForceSlider	= 0;					// スライダに加える力

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
dReal DesLegAng		= 0;					// 脚の目標角度
dReal DesContactPos = 0;					// 脚の目標接地位置

// その他
dReal GroundTime	= 0.16;					// 支持脚時間(自動更新される)


void cal_state_value(void){
	static double time_tmp =0;
	
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

	VelX = cal_d_1st_filter(5, PosX, OMEGA);				// 速度x　（実際には計測できない）
	VelY = cal_d_1st_filter(6, PosY, OMEGA);				// 速度y　（実際には計測できない）
	VelZ = cal_d_1st_filter(7, PosZ, OMEGA);				// 速度z　（実際には計測できない）
	
	AccX     = cal_d_1st_filter(8 , VelX , OMEGA);			// x軸加速度　(ロボットの進行方向)
	AccY     = cal_d_1st_filter(9 , VelY , OMEGA);			// y軸加速度　(ロボットの横方向)
	AccZ     = cal_d_1st_filter(10, VelZ , OMEGA);			// z軸加速度	

	// 脚の状態量
	AngLeg    = dJointGetHingeAngle(Joint_BL);				// 脚の角度
	AngVelLeg = cal_d_1st_filter(3, AngLeg, OMEGA);			// 脚の角速度

	// スライダー
	PosSlider = dJointGetSliderPosition(Joint_LS);			// スライダーの位置
	VelSlider = cal_d_1st_filter(4, PosSlider, OMEGA);		// スライダーの速度

	// 遊脚・支持脚の判断
	StateFilterPast = StateFilter;
	StateFilter = cal_lp_1st_filter(0, (double)(State), 50.0);
	
	if(SKY && GROUND_PAST) GroundTime = Time - time_tmp;	// 遊脚になった   支持脚時間の計算
	if(GROUND && SKY_PAST) time_tmp   = Time;				// 支持脚になった 支持脚時間の計算に利用
}
