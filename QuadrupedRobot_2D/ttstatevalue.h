#define SKY1			(StateFilter1	  <  StateTh)
#define SKY2			(StateFilter2	  <  StateTh)
#define GROUND1			(StateFilter1	  >= StateTh)
#define GROUND2			(StateFilter2	  >= StateTh)
#define SKY_PAST1		(StateFilterPast1 <  StateTh)
#define SKY_PAST2		(StateFilterPast2 <  StateTh)
#define GROUND_PAST1	(StateFilterPast1 >= StateTh)
#define GROUND_PAST2	(StateFilterPast2 >= StateTh)

double	Time			= 0;
int		State1			= 0;				// �V�r:0 �� �x���r:1 �̏�Ԃ�����
int		State2			= 0;
double	StateFilter1	= 0;				// �V�r:0 ����X�^�[�g State��lp�t�B���^��ʂ������́@�`���^�����O�h�~
double  StateFilter2	= 0;
double	StateFilterPast1= 0;				// �ߋ��̃t�B���^��ʂ������
double	StateFilterPast2= 0;
double	StateTh			= 0.2;				// ���̒l�𒴂���Ǝw���r�Ɣ��f����

int LegChecker = 0;
int StepLeg1 = 0;							// If Leg1 touches ground, this value is 1
int StepLeg2 = 1;							// If Leg2 touches ground, this value is 1

//------- ��ԗ� -------
// �{�̂̎p���Z���T
dReal AngRoll		= 0;					// ���[���p
dReal AngPitch		= 0;					// �s�b�`�p
dReal AngYaw		= 0;					// ���[�p

dReal AngVelX		= 0;					// X���p���x
dReal AngVelY		= 0;					// Y���p���x
dReal AngVelZ		= 0;					// Z���p���x

dReal AccX			= 0;					// X�������x
dReal AccY			= 0;					// Y�������x
dReal AccZ			= 0;					// Z�������x

// �r
dReal AngLegY1		= 0;					// �r�̊p�x	
dReal AngVelLegY1	= 0;					// �r�̊p���x
dReal TorqueLegY1	= 0;					// �r�ɉ�����g���N
dReal AngLegX1		= 0;					// �r�̊p�x	
dReal AngVelLegX1	= 0;					// �r�̊p���x
dReal TorqueLegX1	= 0;					// �r�ɉ�����g���N

dReal AngLegY2		= 0;					// �r�̊p�x	
dReal AngVelLegY2	= 0;					// �r�̊p���x
dReal TorqueLegY2	= 0;					// �r�ɉ�����g���N
dReal AngLegX2		= 0;					// �r�̊p�x	
dReal AngVelLegX2	= 0;					// �r�̊p���x
dReal TorqueLegX2	= 0;					// �r�ɉ�����g���N

dReal AngLegY3		= 0;					// �r�̊p�x	
dReal AngVelLegY3	= 0;					// �r�̊p���x
dReal TorqueLegY3	= 0;					// �r�ɉ�����g���N
dReal AngLegX3		= 0;					// �r�̊p�x	
dReal AngVelLegX3	= 0;					// �r�̊p���x
dReal TorqueLegX3	= 0;					// �r�ɉ�����g���N

dReal AngLegY4		= 0;					// �r�̊p�x	
dReal AngVelLegY4	= 0;					// �r�̊p���x
dReal TorqueLegY4	= 0;					// �r�ɉ�����g���N
dReal AngLegX4		= 0;					// �r�̊p�x	
dReal AngVelLegX4	= 0;					// �r�̊p���x
dReal TorqueLegX4	= 0;					// �r�ɉ�����g���N

// �X���C�_�[
dReal PosSlider1	= 0;					// �X���C�_�̈ʒu	
dReal VelSlider1	= 0;					// �X���C�_�̑��x
dReal ForceSlider1	= 0;					// �X���C�_�ɉ������

dReal PosSlider2	= 0;					// �X���C�_�̈ʒu	
dReal VelSlider2	= 0;					// �X���C�_�̑��x
dReal ForceSlider2	= 0;					// �X���C�_�ɉ������

dReal PosSlider3	= 0;					// �X���C�_�̈ʒu	
dReal VelSlider3	= 0;					// �X���C�_�̑��x
dReal ForceSlider3	= 0;					// �X���C�_�ɉ������

dReal PosSlider4	= 0;					// �X���C�_�̈ʒu	
dReal VelSlider4	= 0;					// �X���C�_�̑��x
dReal ForceSlider4	= 0;					// �X���C�_�ɉ������

// �{�̂̈ʒu(���ۂɂ͌v���ł��Ȃ�)
dReal PosX			= 0;					// �{�̂̈ʒux
dReal PosY			= 0;					// �{�̂̈ʒuy
dReal PosZ			= 0;					// �{�̂̈ʒuz

dReal VelX			= 0;					// �{�̂̑��xx
dReal VelY			= 0;					// �{�̂̑��xy
dReal VelZ			= 0;					// �{�̂̑��xz

// �ڕW�l
dReal DesVelX		= 0;					// �ڕW���x
dReal DesVelY		= 0;
dReal DesPosX		= 0;					// �ڕW�ʒu
dReal DesPosY		= 0;
dReal DesLegY1Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegY2Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegY3Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegY4Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegX1Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegX2Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegX3Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLegX4Ang	= 0;					// �r�̖ڕW�p�x
dReal DesContactPosY = 0;					// �r�̖ڕW�ڒn�ʒu
dReal DesContactPosX = 0;					// �r�̖ڕW�ڒn�ʒu

// ���̑�
dReal GroundTime1	= 0.096;					// �x���r����(�����X�V�����)
dReal GroundTime2	= 0.096;

void cal_state_value(void){
	static double time_tmp1 = 0;
	static double time_tmp2 = 0;

	// IMU����擾��z��
	const dReal *R	= dBodyGetRotation(Body.body);			// �{�̂̉�]�s��
	const dReal *pos= dBodyGetPosition(Body.body);			// �{�̂̈ʒu

	AngRoll	 = atan2( R[9], R[10]);							// ���[���p
	AngPitch = atan2(-R[8], sqrt(R[9]*R[9]+R[10]*R[10]));	// �s�b�`�p
	AngYaw   = atan2( R[4], R[0] );							// ���[�p

	AngVelX  = cal_d_1st_filter(0, AngRoll , OMEGA);		// x���p���x
	AngVelY  = cal_d_1st_filter(1, AngPitch, OMEGA);		// y���p���x
	AngVelZ  = cal_d_1st_filter(2, AngYaw  , OMEGA);		// z���p���x
	
	PosX =pos[0];											// �ʒux�@�i���ۂɂ͌v���ł��Ȃ��j
	PosY =pos[1];											// �ʒuy�@�i���ۂɂ͌v���ł��Ȃ��j
	PosZ =pos[2];											// �ʒuz�@�i���ۂɂ͌v���ł��Ȃ��j

	VelX = cal_d_1st_filter(3, PosX, OMEGA);				// ���xx�@�i���ۂɂ͌v���ł��Ȃ��j
	VelY = cal_d_1st_filter(4, PosY, OMEGA);				// ���xy�@�i���ۂɂ͌v���ł��Ȃ��j
	VelZ = cal_d_1st_filter(5, PosZ, OMEGA);				// ���xz�@�i���ۂɂ͌v���ł��Ȃ��j
	
	AccX     = cal_d_1st_filter(6 , VelX , OMEGA);			// x�������x�@(���{�b�g�̐i�s����)
	AccY     = cal_d_1st_filter(7 , VelY , OMEGA);			// y�������x�@(���{�b�g�̉�����)
	AccZ     = cal_d_1st_filter(8, VelZ , OMEGA);			// z�������x	

	// �r�̏�ԗ�
	AngLegY1    = dJointGetHingeAngle(Joint_BL1);		// �r�̊p�x
	AngVelLegY1 = cal_d_1st_filter(9, AngLegY1, OMEGA);		// �r�̊p���x
	
	AngLegY2    = dJointGetHingeAngle(Joint_BL2);		// �r�̊p�x
	AngVelLegY2 = cal_d_1st_filter(10, AngLegY2, OMEGA);	// �r�̊p���x

	AngLegY3    = dJointGetHingeAngle(Joint_BL3);		// �r�̊p�x
	AngVelLegY3 = cal_d_1st_filter(11, AngLegY3, OMEGA);	// �r�̊p���x

	AngLegY4    = dJointGetHingeAngle(Joint_BL4);		// �r�̊p�x
	AngVelLegY4 = cal_d_1st_filter(12, AngLegY4, OMEGA);	// �r�̊p���x

	// �X���C�_�[
	PosSlider1 = dJointGetSliderPosition(Joint_LS1);			// �X���C�_�[�̈ʒu
	VelSlider1 = cal_d_1st_filter(13, PosSlider1, OMEGA);		// �X���C�_�[�̑��x

	PosSlider2 = dJointGetSliderPosition(Joint_LS2);			// �X���C�_�[�̈ʒu
	VelSlider2 = cal_d_1st_filter(14, PosSlider2, OMEGA);		// �X���C�_�[�̑��x

	PosSlider3 = dJointGetSliderPosition(Joint_LS3);			// �X���C�_�[�̈ʒu
	VelSlider3 = cal_d_1st_filter(15, PosSlider3, OMEGA);		// �X���C�_�[�̑��x

	PosSlider4 = dJointGetSliderPosition(Joint_LS4);			// �X���C�_�[�̈ʒu
	VelSlider4 = cal_d_1st_filter(16, PosSlider4, OMEGA);		// �X���C�_�[�̑��x
	
	// �V�r�E�x���r�̔��f
	StateFilterPast1 = StateFilter1;
	StateFilterPast2 = StateFilter2;
	StateFilter1 = cal_lp_1st_filter(0, (double)(State1), 200.0);
	StateFilter2 = cal_lp_1st_filter(1, (double)(State2), 200.0);
	
	if(SKY1 && GROUND_PAST1) GroundTime1 = Time - time_tmp1;	// �r�P���V�r�ɂȂ���   �x���r���Ԃ̌v�Z
	if(GROUND1 && SKY_PAST1) time_tmp1   = Time;				// �r�P���x���r�ɂȂ��� �x���r���Ԃ̌v�Z�ɗ��p
	if(SKY2 && GROUND_PAST2) GroundTime2 = Time - time_tmp2;	// �r2���V�r�ɂȂ���   �x���r���Ԃ̌v�Z
	if(GROUND2 && SKY_PAST2) time_tmp2   = Time;				// �r2���x���r�ɂȂ��� �x���r���Ԃ̌v�Z�ɗ��p
}
