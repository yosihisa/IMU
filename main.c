#include <stdio.h>
#include "imu.h"


#define PI (float)3.141592653589793238462
#define ACCELERATION_Threshold 40
#define ACCELERATION_Threshold2 ACCELERATION_Threshold*ACCELERATION_Threshold

void main() {
	
	printf("IMU_quaternion \n");
	
	Sensor_data data_in;
	Absolute_coordinates data_out;
	FILE *in;
	FILE *out;

	int t_us;
	float dt, ax, ay, az, Ax, Ay, Az;
	int t = 0;
	float  press, temp;
	char str[200];

	int servo, v, i;

	if (fopen_s(&in, "IN.csv", "r") != 0) {
		printf_s("ファイルを読み込めませんでした。\n");
		return;
	}

	if (fopen_s(&out, "OUT.csv", "w") != 0) {
		printf_s("ファイルを作製できませんでした。\n");
		return;
	}
	IMU_Init(&data_in,&data_out);

	fprintf_s(out,"t,press,temp,fm,ax,ay,az,avx,avy,avz,q0,q1,q2,q3,ax,ay,az,vx,xy,vz,x,y,z\n");

	while (fgets(str, sizeof(str), in) != NULL) {

		sscanf_s(str, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d",
			&t_us,
			&press,
			&temp,
			&ax, &ay, &az,
			&data_in.av.x, &data_in.av.y, &data_in.av.z,
			&data_in.m.x, &data_in.m.y, &data_in.m.z,
			&Ax, &Ay, &Az,
			&servo, &v, &i,&data_in.flight_mode);

		dt = 0.001*0.001*t_us;
		t += t_us;

		data_in.av.x *= PI / 180.0;
		data_in.av.y *= PI / 180.0;
		data_in.av.z *= PI / 180.0;


		if ((Ax*Ax + Ay*Ay + Az*Az) > ACCELERATION_Threshold2) {
			data_in.a.x = Ax;
			data_in.a.y = Ay;
			data_in.a.z = Az;
		} else {
			data_in.a.x = ax;
			data_in.a.y = ay;
			data_in.a.z = az;
		}

		//変換
		IMU_Calculation_B2A(&data_in, &data_out, dt);

		//出力
		printf("%f\n", 0.001*0.001*t);
		fprintf_s(out, "%f,%f,%f,%d, %f,%f,%f, %f,%f,%f, %f,%f,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f\n",
			0.001*0.001*t,press,temp,
			data_in.flight_mode,
			data_in.a.x, data_in.a.y, data_in.a.z,
			data_in.av.x, data_in.av.y, data_in.av.z,
			data_out.direction.q0, data_out.direction.q1, data_out.direction.q2, data_out.direction.q3,
			data_out.a.x, data_out.a.y, data_out.a.z,
			data_out.v.x, data_out.v.y, data_out.v.z,
			data_out.c.x, data_out.c.y, data_out.c.z);
	}

	fclose(in);
	fclose(out);
	printf_s("変換が完了しました。\n");

	return;

}