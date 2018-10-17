/**
 * @file imu.h
 * @brief �@�̍��W�n�ł̊ϑ��l���@�̍��W�ւ̕ϊ����s���B
 */

#include "quaternion.h"

#define GRAVITY_ACCELERATION (float)9.806 //�d�͉����x

//��s���[�h�Ɋ֌W
#define USE_AV 0b00000001 //�p������Ɋp���x���g�p
#define USE_A  0b00000010 //�p������ɉ����x���g�p
#define USE_M  0b00000100 //�p������ɒn���C���g�p
#define LOCK_V 0b00001000 //���x��0�ɌŒ肷��


/*! @struct Body_coordinates
	@brief  �@�̍��W�n�ł̊e��ϐ�
*/
typedef struct {
	Descartes_3D a;		//�����x
	Descartes_3D av;	//�p���x
	Descartes_3D m;		//�n���C
	int flight_mode; //��s���[�h
}Sensor_data;

/*! @struct Absolute_coordinates
	@brief  �n����W�n�ł̊e��ϐ�
*/
typedef struct {
	Descartes_3D c;		//���W
	Descartes_3D v;		//���x
	Descartes_3D a;		//�����x
	quaternion direction;	//����(�n����W�n���猩��)(�l����)

	Descartes_3D a0;	//���������x
	Descartes_3D v0;	//�������x
	Descartes_3D c0;	//�������W
	Descartes_3D m0;	//�����n���C


}Absolute_coordinates;

/**
* @fn IMU_Calculation_B2A(Body_coordinates *, Absolute_coordinates *);
* @brief �@�̂ɌŒ肵��Sensor_data�ϑ��l����͂��A��΍��W�n�ɂ����錻�݂̍��W���X�V����B
* @param[in] �@�̍��W�n�ł̋@�̏�Ԃ̍\����
* @param[out] ��΍��W�n�ł̋@�̏�Ԃ̍\����
* @param[in] �O�񂩂�̎��ԍ�
* @return int
*/
int IMU_Calculation_B2A(Sensor_data *body, Absolute_coordinates *abs,float dt);

/**
* @fn IMU_Init(Absolute_coordinates *abs)
* @brief �����l�̐ݒ�
* @return int
*/
int IMU_Init(Sensor_data *body, Absolute_coordinates *abs);

