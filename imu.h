/**
 * @file imu.h
 * @brief 機体座標系での観測値→機体座標への変換を行う。
 */

#include "quaternion.h"

#define GRAVITY_ACCELERATION (float)9.806 //重力加速度

//飛行モードに関係
#define USE_AV 0b00000001 //姿勢推定に角速度を使用
#define USE_A  0b00000010 //姿勢推定に加速度を使用
#define USE_M  0b00000100 //姿勢推定に地磁気を使用
#define LOCK_V 0b00001000 //速度を0に固定する


/*! @struct Body_coordinates
	@brief  機体座標系での各種変数
*/
typedef struct {
	Descartes_3D a;		//加速度
	Descartes_3D av;	//角速度
	Descartes_3D m;		//地磁気
	int flight_mode; //飛行モード
}Sensor_data;

/*! @struct Absolute_coordinates
	@brief  地上座標系での各種変数
*/
typedef struct {
	Descartes_3D c;		//座標
	Descartes_3D v;		//速度
	Descartes_3D a;		//加速度
	quaternion direction;	//方向(地上座標系から見た)(四元数)

	Descartes_3D a0;	//初期加速度
	Descartes_3D v0;	//初期速度
	Descartes_3D c0;	//初期座標
	Descartes_3D m0;	//初期地磁気


}Absolute_coordinates;

/**
* @fn IMU_Calculation_B2A(Body_coordinates *, Absolute_coordinates *);
* @brief 機体に固定したSensor_data観測値を入力し、絶対座標系における現在の座標を更新する。
* @param[in] 機体座標系での機体状態の構造体
* @param[out] 絶対座標系での機体状態の構造体
* @param[in] 前回からの時間差
* @return int
*/
int IMU_Calculation_B2A(Sensor_data *body, Absolute_coordinates *abs,float dt);

/**
* @fn IMU_Init(Absolute_coordinates *abs)
* @brief 初期値の設定
* @return int
*/
int IMU_Init(Sensor_data *body, Absolute_coordinates *abs);

