/**
 * @file av_estimator.cpp
 * Filter class method implementation
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 */

#include "../include/av_estimator.h"

void av_estimator::init(Vector3f &a, Vector3f &mu)
{
	float roll, pitch, yaw;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;

	/* Calculate roll and pitch */
	roll = atan2(-a(1), -a(2));
	pitch = atan2(a(0), -a(2));

	/* Calculate subresults only once */
	cosRoll = cosf(roll);
	sinRoll = sinf(roll);
	cosPitch = cosf(pitch);
	sinPitch = sinf(pitch);

	magX = mu(0) * cosPitch + mu(1) * sinRoll * sinPitch + mu(2) * cosRoll * sinPitch;

	magY = mu(1) * cosRoll - mu(2) * sinRoll;

	yaw = atan2f(-magY, magX);

	AngleAxisf rollAngle(roll, Vector3f::UnitX());
	AngleAxisf pitchAngle(pitch, Vector3f::UnitY());
	AngleAxisf yawAngle(yaw, Vector3f::UnitZ());

	Quaternionf q = rollAngle * pitchAngle * yawAngle;
	Rhat = q.matrix();
	Rhat_prev = Rhat;

	/* Initial mu is used later on for reference */
	this->mu_init = mu;

	/* Earth magnetic field reference */
	mu_init(0) = sqrtf(mu_init(0)*mu_init(0)+mu_init(1)*mu_init(1));//0.232f;
	mu_init(1) = 0;//0.0506f;
	//mu_init(2) = -0.53f;
}


void av_estimator::orthonormalize(Matrix3f & mat)
{
	Vector3f c0 = mat.col(0);
	Vector3f c1 = mat.col(1);
	Vector3f c2 = mat.col(2);

	mat.col(0).normalize();

	mat.col(1) = c1-c1.dot(c0)*c0;
	mat.col(1).normalize();
	mat.col(2) = c2 - c2.dot(c0)*c0 - c2.dot(c1)*c1;
	mat.col(2).normalize();
}

Matrix3f av_estimator::skew(Vector3f a)
{
	Matrix3f skewed;
	skewed 	<< 	0.0f, -a(2),a(1),
		  	a(2), 0.0f, -a(0),
		  	-a(1), a(0), 0.0f;
	return skewed;
}
