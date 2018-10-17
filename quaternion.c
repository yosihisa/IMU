#include "quaternion.h"
#include <math.h>

quaternion Quaternion_QP(quaternion * q, quaternion * p) {
	quaternion Q;
	Q.q0 = q->q0*p->q0 - q->q1*p->q1 - q->q2*p->q2 - q->q3*p->q3;
	Q.q1 = q->q0*p->q1 + q->q1*p->q0 + q->q2*p->q3 - q->q3*p->q2;
	Q.q2 = q->q0*p->q2 - q->q1*p->q3 + q->q2*p->q0 + q->q3*p->q1;
	Q.q3 = q->q0*p->q3 + q->q1*p->q2 - q->q2*p->q1 + q->q3*p->q0;
	return Q;
}


Descartes_3D Quaternion_3DRotation(quaternion * q, Descartes_3D * r) {
	Descartes_3D R;
	R.x = r->x * (q->q0*q->q0 + q->q1*q->q1 - q->q2*q->q2 - q->q3*q->q3)
		+ r->y * 2 * (q->q1*q->q2 - q->q3*q->q0)
		+ r->z * 2 * (q->q0*q->q2 + q->q1*q->q3);
	
	R.y = r->x * 2 * (q->q0*q->q3 + q->q1*q->q2)
		+ r->y * (q->q0*q->q0 - q->q1*q->q1 + q->q2*q->q2 - q->q3*q->q3)
		+ r->z * 2 * (q->q2*q->q3 - q->q0*q->q1);

	R.z = r->x * 2 * (q->q1*q->q3 - q->q0*q->q2)
		+ r->y * 2 * (q->q2*q->q3 + q->q0*q->q1)
		+ r->z * (q->q0*q->q0 - q->q1*q->q1 - q->q2*q->q2 + q->q3*q->q3);

	return R;
}

quaternion Quaternion_MakeRotate(float x, float y, float z, float a){
	
	quaternion q;
	//‰ñ“]Šp‚ª0‚¾‚Á‚½‚ç
	if (a == 0) {
		q.q0 = 1.0;
		q.q1 = 0.0;
		q.q2 = 0.0;
		q.q3 = 0.0;
		return q;
	}

	//³‹K‰»‚µ‚Ä‘ã“ü
	float norm =sqrt(x*x + y * y + z * z);
	q.q0 = cos(0.5*a);
	q.q1 = sin(0.5*a)*x / norm;
	q.q2 = sin(0.5*a)*y / norm;
	q.q3 = sin(0.5*a)*z / norm;
	return q;
}
