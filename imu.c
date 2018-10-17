#include "imu.h"
#include <math.h>

int IMU_Init(Sensor_data * body, Absolute_coordinates * abs){
	
	//�����x���珉���p�̐ݒ�
	abs->direction.q0 = 0.0;
	abs->direction.q1 = 1.0;
	abs->direction.q2 = 0.0;
	abs->direction.q3 = 0.0;

	//�n���C�̏����p�̐ݒ�


	//�������W�̐ݒ�
	abs->c0.x = 0.0;
	abs->c0.y = 0.0;
	abs->c0.z = 0.0;
	
	abs->c.x = 0.0;
	abs->c.y = 0.0;
	abs->c.z = 0.0;

	//�����x�̐ݒ�
	abs->v0.x = 0.0;
	abs->v0.y = 0.0;
	abs->v0.z = 0.0;

	abs->v.x = 0.0;
	abs->v.y = 0.0;
	abs->v.z = 0.0;

	//�������x�̐ݒ�
	abs->a0.x = 0.0;
	abs->a0.y = 0.0;
	abs->a0.z = 0.0;

	abs->a.x = 0.0;
	abs->a.y = 0.0;
	abs->a.z = 0.0;

	return 0;
}

int IMU_Calculation_B2A(Sensor_data *body, Absolute_coordinates *abs,float dt){

	char use = 0;
	switch (body->flight_mode) {

	case 0: //���S�ɐÎ~
		use = LOCK_V;
		break;
	case 1: //��G
		use = USE_AV | USE_M;
		break;
	case 2: //�����d��
		use = USE_AV | USE_M;
		break;
	case 3: //�p���V���[�g�W�J�E���n
		use = USE_AV | USE_M;
		break;
	case 4: //�ق�1G
		use = USE_AV | USE_M | USE_A;
		break;
	case 5: //���i�����ɐÎ~
		use = LOCK_V | USE_AV | USE_M | USE_A;
		break;

	default:
		use = USE_AV | USE_M | USE_A;
		break;
	}

	float w;
	quaternion qw, qa, qm, q;

	//�p���x�����]�l�����𐶐�
	if ((use & USE_AV) != 0) {
		w = sqrt(body->av.x*body->av.x + body->av.y* body->av.y + body->av.z*body->av.z)*dt; //��]�p
		qw = Quaternion_MakeRotate(body->av.x, body->av.y, body->av.z, w);
	}
	//�����x�����]�l�����𐶐�
	if ((use & USE_A) != 0) {
		qa;
	}
	//�n���C�����]�l�����𐶐�
	if ((use & USE_M) != 0) {
		qm;
	}

	//�Z���T�t���[�W���� https://twitter.com/n_yosihisa/status/1038418416988647425
	//��]�ʂ̓����Ɋւ��鏈��
	switch (body->flight_mode) {

	case 0: //���S�ɐÎ~
		//��]��0
		q.q0 = 1.0;
		q.q1 = 0.0;
		q.q2 = 0.0;
		q.q3 = 0.0;

		//���x0�@�˓_�Œ莞�ł��邽�ߗ�O�I�ɋL�q
		abs->v.x = 0.0;
		abs->v.y = 0.0;
		abs->v.z = 0.0;

		break;

	case 1: //��G
		//��]�ʁ@�p���x�E�n���C���琶��
		q = qw;
		break;

	case 2: //�����d��
		//��]�ʁ@�p���x�E�n���C���琶��
		q = qw;
		break;

	case 3: //�p���V���[�g�W�J�E���n
		q = qw;
		break;

	case 4: //�ق�1G
		q = qw;
		break;

	case 5: //���i�����ɐÎ~
		q = qw;

		//���x0
		abs->v.x = 0.0;
		abs->v.y = 0.0;
		abs->v.z = 0.0;
		break;

	default:
		q = qw;
		break;
	}

	//�@�̂̌������X�V
	abs->direction = Quaternion_QP(&q, &abs->direction);

	//�����x�̍��W�n��ϊ�
	abs->a = Quaternion_3DRotation(&abs->direction, &body->a); //�n����W�n�̉����x����]
	abs->a.y -= GRAVITY_ACCELERATION;

	//�ϕ�
	//�Î~���Ă��Ȃ���΍��W���X�V
	if ((use & LOCK_V) == 0) {
		abs->v.x += abs->a.x *dt;
		abs->v.y += abs->a.y *dt;
		abs->v.z += abs->a.z *dt;

		abs->c.x += abs->v.x *dt;
		abs->c.y += abs->v.y *dt;
		abs->c.z += abs->v.z *dt;
	}
	return 0;
}