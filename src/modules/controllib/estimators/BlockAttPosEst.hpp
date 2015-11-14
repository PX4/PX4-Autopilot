#pragma once

#include <controllib/blocks.hpp>
#include <matrix/math.hpp>
#include <drivers/drv_hrt.h>

// publications

// subscriptions
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "BlockAttEst.hpp"

using namespace matrix;

typedef Vector<float, 1> Vec1;
typedef Vector3<float> Vec3;
typedef Vector<float, 6> Vec6;
typedef SquareMatrix<float, 1> SMat1;
typedef SquareMatrix<float, 3> SMat3;
typedef SquareMatrix<float, 6> SMat6;

class BlockAttPosEst : public BlockAttEst {
public:
	BlockAttPosEst(SuperBlock * parent, const char * name);
	void update();
protected:
	// correction
	virtual void correctMagAccel(const Vec6 & y, const SMat6 & R) = 0;
	virtual void correctAbsAlt(const Vec1 & y, const SMat1 & R) = 0;
	virtual void correctRelAlt(const Vec1 & y, const SMat1 & R) = 0;
	virtual void correctPosVel(const Vec6 & y, const SMat6 & R) = 0;

	// prediction
	virtual void predict(const Vec6 & y, const SMat6 & R) = 0;
private:
	// publications
	
	// subscriptions
	uORB::Subscription<sensor_combined_s> _subSensor;
	uORB::Subscription<parameter_update_s> _subParam;
	uORB::Subscription<vehicle_control_mode_s> _subControlMode;
	uORB::Subscription<vehicle_gps_position_s> _subGps;

	// misc
	uint64_t _timestamp;
	uint64_t _timestampGyro;
	uint64_t _timestampBaro;
	uint64_t _timestampMag;
	uint64_t _timestampAccel;

	// subscription handlers
	void handleSensor();
	void handleParam();
	void handleGps();
};
