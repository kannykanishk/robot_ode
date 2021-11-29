#define S_TIME			0.005				// サンプリング時間[s]

#define SPRING			2000.0				// ばね定数
#define MAX_VEL			1.2					// ロボットの最高速度
#define MAX_LEG_ANG		(30.0*D2R)			// 脚の可動範囲[rad]
#define MAX_SLIDER_POS	0.17				// スライダーの最大位置
#define MIN_SLIDER_POS	0.13				// スライダーの最小位置

#define OMEGA			30.0				// 1次，2次微分フィルタの折線角周波数
#define ZETA			0.71				// 2次微分フィルタの減衰に関する係数

#define k1Y              100.0
#define k2Y              10.0
#define k3Y              1000.0
#define k4Y              300.0
#define k5Y              280.0

#define k1X              100.0
#define k2X              10.0
#define k3X              1000.0
#define k4X              300.0
#define k5X              280.0