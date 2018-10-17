/**
 * @file quaternion.h
 * @brief 四元数関係の計算ライブラリ
 */

 /*! @struct quaternion
	 @brief  四元数の定義
 */
typedef struct{
	float q0,q1,q2,q3;
}quaternion;


/*! @struct Descartes_3D
	@brief  3次元直交座標を定義
*/
typedef struct {
	float x, y, z;
}Descartes_3D;


/**
* @fn Quaternion_QP(quaternion *q, quaternion *p)
* @brief クォータニオンの積 Pに左からQをかける。
* @param[in] q 左からかける四元数
* @param[in] p　更新される四元数
* @return int
*/
quaternion Quaternion_QP(quaternion *q, quaternion *p);


/**
* @fn Quaternion_3DRotation(quaternion *q, quaternion *p)
* @brief 3次元上の点をある四元数で回転させる。
* @param[in] 回転を表す四元数
* @param[in] 3次元座標
* @return int
*/
Descartes_3D Quaternion_3DRotation(quaternion *q, Descartes_3D *r);

/**
* @fn Quaternion_MakeRotate(float x, float y, float z, float a,quaternion *q)
* @brief 回転を表す四元数を生成する。正規化はされてなくてもよい。
* @param[in] x y z 回転軸
* @param[in] a 回転角[rad]
* @param[out] 生成される四元数
* @return int
*/
quaternion Quaternion_MakeRotate(float x, float y, float z, float a);