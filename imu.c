#include "imu.h"
#include <math.h>

int IMU_Init(Sensor_data * body, Absolute_coordinates * abs){
	
	//加速度から初期角の設定
	abs->direction.q0 = 0.0;
	abs->direction.q1 = 1.0;
	abs->direction.q2 = 0.0;
	abs->direction.q3 = 0.0;

	//地磁気の初期角の設定


	//初期座標の設定
	abs->c0.x = 0.0;
	abs->c0.y = 0.0;
	abs->c0.z = 0.0;
	
	abs->c.x = 0.0;
	abs->c.y = 0.0;
	abs->c.z = 0.0;

	//初速度の設定
	abs->v0.x = 0.0;
	abs->v0.y = 0.0;
	abs->v0.z = 0.0;

	abs->v.x = 0.0;
	abs->v.y = 0.0;
	abs->v.z = 0.0;

	//初加速度の設定
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

	case 0: //完全に静止
		use = LOCK_V;
		break;
	case 1: //高G
		use = USE_AV | USE_M;
		break;
	case 2: //微小重力
		use = USE_AV | USE_M;
		break;
	case 3: //パラシュート展開・着地
		use = USE_AV | USE_M;
		break;
	case 4: //ほぼ1G
		use = USE_AV | USE_M | USE_A;
		break;
	case 5: //並進方向に静止
		use = LOCK_V | USE_AV | USE_M | USE_A;
		break;

	default:
		use = USE_AV | USE_M | USE_A;
		break;
	}

	float w;
	quaternion qw, qa, qm, q;

	//角速度から回転四元数を生成
	if ((use & USE_AV) != 0) {
		w = sqrt(body->av.x*body->av.x + body->av.y* body->av.y + body->av.z*body->av.z)*dt; //回転角
		qw = Quaternion_MakeRotate(body->av.x, body->av.y, body->av.z, w);
	}
	//加速度から回転四元数を生成
	if ((use & USE_A) != 0) {
		qa;
	}
	//地磁気から回転四元数を生成
	if ((use & USE_M) != 0) {
		qm;
	}

	//センサフュージョン https://twitter.com/n_yosihisa/status/1038418416988647425
	//回転量の統合に関する処理
	switch (body->flight_mode) {

	case 0: //完全に静止
		//回転量0
		q.q0 = 1.0;
		q.q1 = 0.0;
		q.q2 = 0.0;
		q.q3 = 0.0;

		//速度0　射点固定時であるため例外的に記述
		abs->v.x = 0.0;
		abs->v.y = 0.0;
		abs->v.z = 0.0;

		break;

	case 1: //高G
		//回転量　角速度・地磁気から生成
		q = qw;
		break;

	case 2: //微小重力
		//回転量　角速度・地磁気から生成
		q = qw;
		break;

	case 3: //パラシュート展開・着地
		q = qw;
		break;

	case 4: //ほぼ1G
		q = qw;
		break;

	case 5: //並進方向に静止
		q = qw;

		//速度0
		abs->v.x = 0.0;
		abs->v.y = 0.0;
		abs->v.z = 0.0;
		break;

	default:
		q = qw;
		break;
	}

	//機体の向きを更新
	abs->direction = Quaternion_QP(&q, &abs->direction);

	//加速度の座標系を変換
	abs->a = Quaternion_3DRotation(&abs->direction, &body->a); //地上座標系の加速度を回転
	abs->a.y -= GRAVITY_ACCELERATION;

	//積分
	//静止していなければ座標を更新
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