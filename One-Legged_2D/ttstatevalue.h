#define SKY				(StateFilter	 <  StateTh)
#define GROUND			(StateFilter	 >= StateTh)
#define SKY_PAST		(StateFilterPast <  StateTh)
#define GROUND_PAST		(StateFilterPast >= StateTh)

double	Time			= 0;
int		State			= 0;				// �V�r:0 �� �x���r:1 �̏�Ԃ�����
double	StateFilter		= 0;				// �V�r:0 ����X�^�[�g State��lp�t�B���^��ʂ������́@�`���^�����O�h�~
double	StateFilterPast	= 0;				// �ߋ��̃t�B���^��ʂ������
double	StateTh			= 0.2;				// ���̒l�𒴂���Ǝw���r�Ɣ��f����


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
dReal AngLeg		= 0;					// �r�̊p�x	
dReal AngVelLeg		= 0;					// �r�̊p���x
dReal TorqueLeg		= 0;					// �r�ɉ�����g���N

// �X���C�_�[
dReal PosSlider		= 0;					// �X���C�_�̈ʒu	
dReal VelSlider		= 0;					// �X���C�_�̑��x
dReal ForceSlider	= 0;					// �X���C�_�ɉ������

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
dReal DesLegAng		= 0;					// �r�̖ڕW�p�x
dReal DesContactPos = 0;					// �r�̖ڕW�ڒn�ʒu

// ���̑�
dReal GroundTime	= 0.16;					// �x���r����(�����X�V�����)


void cal_state_value(void){
	static double time_tmp =0;
	
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

	VelX = cal_d_1st_filter(5, PosX, OMEGA);				// ���xx�@�i���ۂɂ͌v���ł��Ȃ��j
	VelY = cal_d_1st_filter(6, PosY, OMEGA);				// ���xy�@�i���ۂɂ͌v���ł��Ȃ��j
	VelZ = cal_d_1st_filter(7, PosZ, OMEGA);				// ���xz�@�i���ۂɂ͌v���ł��Ȃ��j
	
	AccX     = cal_d_1st_filter(8 , VelX , OMEGA);			// x�������x�@(���{�b�g�̐i�s����)
	AccY     = cal_d_1st_filter(9 , VelY , OMEGA);			// y�������x�@(���{�b�g�̉�����)
	AccZ     = cal_d_1st_filter(10, VelZ , OMEGA);			// z�������x	

	// �r�̏�ԗ�
	AngLeg    = dJointGetHingeAngle(Joint_BL);				// �r�̊p�x
	AngVelLeg = cal_d_1st_filter(3, AngLeg, OMEGA);			// �r�̊p���x

	// �X���C�_�[
	PosSlider = dJointGetSliderPosition(Joint_LS);			// �X���C�_�[�̈ʒu
	VelSlider = cal_d_1st_filter(4, PosSlider, OMEGA);		// �X���C�_�[�̑��x

	// �V�r�E�x���r�̔��f
	StateFilterPast = StateFilter;
	StateFilter = cal_lp_1st_filter(0, (double)(State), 50.0);
	
	if(SKY && GROUND_PAST) GroundTime = Time - time_tmp;	// �V�r�ɂȂ���   �x���r���Ԃ̌v�Z
	if(GROUND && SKY_PAST) time_tmp   = Time;				// �x���r�ɂȂ��� �x���r���Ԃ̌v�Z�ɗ��p
}
