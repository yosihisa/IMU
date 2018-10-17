/**
 * @file quaternion.h
 * @brief �l�����֌W�̌v�Z���C�u����
 */

 /*! @struct quaternion
	 @brief  �l�����̒�`
 */
typedef struct{
	float q0,q1,q2,q3;
}quaternion;


/*! @struct Descartes_3D
	@brief  3�����������W���`
*/
typedef struct {
	float x, y, z;
}Descartes_3D;


/**
* @fn Quaternion_QP(quaternion *q, quaternion *p)
* @brief �N�H�[�^�j�I���̐� P�ɍ�����Q��������B
* @param[in] q �����炩����l����
* @param[in] p�@�X�V�����l����
* @return int
*/
quaternion Quaternion_QP(quaternion *q, quaternion *p);


/**
* @fn Quaternion_3DRotation(quaternion *q, quaternion *p)
* @brief 3������̓_������l�����ŉ�]������B
* @param[in] ��]��\���l����
* @param[in] 3�������W
* @return int
*/
Descartes_3D Quaternion_3DRotation(quaternion *q, Descartes_3D *r);

/**
* @fn Quaternion_MakeRotate(float x, float y, float z, float a,quaternion *q)
* @brief ��]��\���l�����𐶐�����B���K���͂���ĂȂ��Ă��悢�B
* @param[in] x y z ��]��
* @param[in] a ��]�p[rad]
* @param[out] ���������l����
* @return int
*/
quaternion Quaternion_MakeRotate(float x, float y, float z, float a);