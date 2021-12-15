#define SKY1			(StateFilter1	  <  StateTh)
#define SKY2			(StateFilter2	  <  StateTh)
#define GROUND1			(StateFilter1	  >= StateTh)
#define GROUND2			(StateFilter2	  >= StateTh)
#define SKY_PAST1		(StateFilterPast1 <  StateTh)
#define SKY_PAST2		(StateFilterPast2 <  StateTh)
#define GROUND_PAST1	(StateFilterPast1 >= StateTh)
#define GROUND_PAST2	(StateFilterPast2 >= StateTh)

double	Time			 = 0;
int		State1			 = 0;				// �r1�ɂ���  �V�r:0 �� �x���r:1 �̏�Ԃ�����
int		State2			 = 0;				// �r�Q�ɂ���  �V�r:0 �� �x���r:1 �̏�Ԃ�����
double	StateFilter1		 = 0;				// �r1�ɂ���  �V�r:0 ����X�^�[�g State��lp�t�B���^��ʂ������́@�`���^�����O�h�~
double	StateFilter2		 = 0;				// �r�Q�ɂ���  �V�r:0 ����X�^�[�g State��lp�t�B���^��ʂ������́@�`���^�����O�h�~
double	StateFilterPast1 = 0;				// �ߋ��̃t�B���^��ʂ������
double	StateFilterPast2 = 0;				// �ߋ��̃t�B���^��ʂ������
double	StateTh			 = 0.2;				// ���̒l�𒴂���Ǝw���r�Ɣ��f����

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
dReal AngLeg1		= 0;					// �r�̊p�x	
dReal AngVelLeg1	= 0;					// �r�̊p���x
dReal TorqueLeg1	= 0;					// �r�ɉ�����g���N

dReal AngLeg2		= 0;					// �r�̊p�x	
dReal AngVelLeg2	= 0;					// �r�̊p���x
dReal TorqueLeg2	= 0;					// �r�ɉ�����g���N

// �X���C�_�[
dReal PosSlider1	= 0;					// �X���C�_�̈ʒu	
dReal VelSlider1	= 0;					// �X���C�_�̑��x
dReal ForceSlider1	= 0;					// �X���C�_�ɉ������

dReal PosSlider2	= 0;					// �X���C�_�̈ʒu	
dReal VelSlider2	= 0;					// �X���C�_�̑��x
dReal ForceSlider2	= 0;					// �X���C�_�ɉ������

// �{�̂̈ʒu(���ۂɂ͌v���ł��Ȃ�)
dReal PosX			= 0;					// �{�̂̈ʒux
dReal PosY			= 0;					// �{�̂̈ʒuy
dReal PosZ			= 0;					// �{�̂̈ʒuz

dReal VelX			= 0;					// �{�̂̑��xx
dReal VelY			= 0;					// �{�̂̑��xy
dReal VelZ			= 0;					// �{�̂̑��xz

// �ڕW�l
dReal DesVel		= 0;					// �ڕW���x
dReal DesPos		= 0;					// �ڕW�ʒu
dReal DesLeg1Ang	= 0;					// �r�̖ڕW�p�x
dReal DesLeg2Ang	= 0;					// �r�̖ڕW�p�x
dReal DesContactPos = 0;					// �r�̖ڕW�ڒn�ʒu

// ���̑�
dReal GroundTime1	= 0.096;					// �x���r����(�����X�V�����)
dReal GroundTime2	= 0.096;					// �x���r����(�����X�V�����)


void cal_state_value(void){
	static double time_tmp1 = 0;
	static double time_tmp2 = 0;
	
	// IMU����擾��z��
	const dReal *R	= dBodyGetRotation(Body.body);			// �{�̂̉�]�s��
	const dReal *pos= dBodyGetPosition(Body.body);			// �{�̂̈ʒu

	AngRoll	   = atan2( R[9], R[10]);						// ���[���p
	AngPitch   = atan2(-R[8], sqrt(R[9]*R[9]+R[10]*R[10]));	// �s�b�`�p
	AngYaw     = atan2( R[4], R[0] );						// ���[�p

	AngVelX    = cal_d_1st_filter(0, AngRoll , OMEGA);		// x���p���x
	AngVelY    = cal_d_1st_filter(1, AngPitch, OMEGA);		// y���p���x
	AngVelZ    = cal_d_1st_filter(2, AngYaw  , OMEGA);		// z���p���x
	
	PosX =pos[0];											// �ʒux�@�i���ۂɂ͌v���ł��Ȃ��j
	PosY =pos[1];											// �ʒuy�@�i���ۂɂ͌v���ł��Ȃ��j
	PosZ =pos[2];											// �ʒuz�@�i���ۂɂ͌v���ł��Ȃ��j

	VelX 	   = cal_d_1st_filter(3, PosX, OMEGA);			// ���xx�@�i���ۂɂ͌v���ł��Ȃ��j
	VelY 	   = cal_d_1st_filter(4, PosY, OMEGA);			// ���xy�@�i���ۂɂ͌v���ł��Ȃ��j
	VelZ 	   = cal_d_1st_filter(5, PosZ, OMEGA);			// ���xz�@�i���ۂɂ͌v���ł��Ȃ��j
	
	AccX       = cal_d_1st_filter(6 , VelX , OMEGA);		// x�������x�@(���{�b�g�̐i�s����)
	AccY       = cal_d_1st_filter(7 , VelY , OMEGA);		// y�������x�@(���{�b�g�̉�����)
	AccZ       = cal_d_1st_filter(8 , VelZ , OMEGA);		// z�������x	

	// �r�̏�ԗ�
	AngLeg1    = dJointGetHingeAngle(Joint_BL1);			// �r�̊p�x
	AngVelLeg1 = cal_d_1st_filter(9, AngLeg1, OMEGA);		// �r�̊p���x

	AngLeg2    = dJointGetHingeAngle(Joint_BL2);			// �r�̊p�x
	AngVelLeg2 = cal_d_1st_filter(10, AngLeg2, OMEGA);		// �r�̊p���x

	// �X���C�_�[
	PosSlider1 = dJointGetSliderPosition(Joint_LS1);		// �X���C�_�[�̈ʒu
	VelSlider1 = cal_d_1st_filter(11, PosSlider1, OMEGA);	// �X���C�_�[�̑��x

	PosSlider2 = dJointGetSliderPosition(Joint_LS2);		// �X���C�_�[�̈ʒu
	VelSlider2 = cal_d_1st_filter(12, PosSlider2, OMEGA);	// �X���C�_�[�̑��x
	
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
