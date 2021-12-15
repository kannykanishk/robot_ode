#define S_TIME			0.004				// サンプリング時間[s]

#define SPRING			400.0				// バネ定数
#define MAX_VEL			1.5					// 最高速度
#define MAX_LEG_ANG		(30.0*D2R)			// 脚の最大関節角度[rad]
#define MAX_SLIDER_POS	0.075				// スライダーの最大値
#define MIN_SLIDER_POS	0.0					// スライダーの最小値
#define ADD_FORCE		400.0
#define SLIDER_FORCE	-100.0


#define OMEGA			30.0				// 折点角周波数
#define ZETA			0.71				// ２次フィルターの減衰に関する係数

#define k1              100.0
#define k2              10.0
#define k3              800.0
#define k4              200.0
#define k5              40.0
