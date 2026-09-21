#include "SensorFusion.h"

namespace SlimeVR::Sensors {

void SensorFusion::update6D(
	sensor_real_t Axyz[3],
	sensor_real_t Gxyz[3],
	sensor_real_t deltat
) {
	updateAcc(Axyz, deltat);
	updateGyro(Gxyz, deltat);
}

void SensorFusion::update9D(
	sensor_real_t Axyz[3],
	sensor_real_t Gxyz[3],
	sensor_real_t Mxyz[3],
	sensor_real_t deltat
) {
	updateMag(Mxyz, deltat);
	updateAcc(Axyz, deltat);
	updateGyro(Gxyz, deltat);
}

void SensorFusion::updateAcc(const sensor_real_t Axyz[3], sensor_real_t deltat) {
	if (deltat < 0) {
		deltat = accTs;
	}

	std::copy(Axyz, Axyz + 3, bAxyz);
	vqf.updateAcc(Axyz);
}

void SensorFusion::updateMag(const sensor_real_t Mxyz[3], sensor_real_t deltat) {
	if (deltat < 0) {
		deltat = magTs;
	}

	if (!magExist) {
		if (Mxyz[0] != 0.0f || Mxyz[1] != 0.0f || Mxyz[2] != 0.0f) {
			magExist = true;
		} else {
			return;
		}
	}

	for (int i = 0; i < 3; i++) {
		lastMag[i] = Mxyz[i];
	}

	vqf.updateMag(Mxyz);
}

void SensorFusion::updateGyro(const sensor_real_t Gxyz[3], sensor_real_t deltat) {
	if (deltat < 0) {
		deltat = gyrTs;
	}

	vqf.updateGyr(Gxyz, deltat);

	updated = true;
	gravityReady = false;
	linaccelReady = false;
}

bool SensorFusion::isUpdated() { return updated; }

void SensorFusion::clearUpdated() { updated = false; }

sensor_real_t const* SensorFusion::getQuaternion() {
	if (magExist) {
		vqf.getQuat9D(qwxyz);
	} else {
		vqf.getQuat6D(qwxyz);
	}

	return qwxyz;
}

Quat SensorFusion::getQuaternionQuat() {
	getQuaternion();
	return Quat(qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]);
}

sensor_real_t const* SensorFusion::getGravityVec() {
	if (!gravityReady) {
		calcGravityVec(qwxyz, vecGravity);
		gravityReady = true;
	}
	return vecGravity;
}

sensor_real_t const* SensorFusion::getLinearAcc() {
	if (!linaccelReady) {
		getGravityVec();
		calcLinearAcc(bAxyz, vecGravity, linAccel);
		linaccelReady = true;
	}
	return linAccel;
}

void SensorFusion::getLinearAcc(sensor_real_t outLinAccel[3]) {
	getLinearAcc();
	std::copy(linAccel, linAccel + 3, outLinAccel);
}

Vector3 SensorFusion::getLinearAccVec() {
	getLinearAcc();
	return Vector3(linAccel[0], linAccel[1], linAccel[2]);
}

void SensorFusion::calcGravityVec(
	const sensor_real_t qwxyz[4],
	sensor_real_t gravVec[3]
) {
	gravVec[0] = 2 * (qwxyz[1] * qwxyz[3] - qwxyz[0] * qwxyz[2]);
	gravVec[1] = 2 * (qwxyz[0] * qwxyz[1] + qwxyz[2] * qwxyz[3]);
	gravVec[2] = qwxyz[0] * qwxyz[0] - qwxyz[1] * qwxyz[1] - qwxyz[2] * qwxyz[2]
			   + qwxyz[3] * qwxyz[3];
}

void SensorFusion::calcLinearAcc(
	const sensor_real_t accin[3],
	const sensor_real_t gravVec[3],
	sensor_real_t accout[3]
) {
	accout[0] = accin[0] - gravVec[0] * CONST_EARTH_GRAVITY;
	accout[1] = accin[1] - gravVec[1] * CONST_EARTH_GRAVITY;
	accout[2] = accin[2] - gravVec[2] * CONST_EARTH_GRAVITY;
}

void SensorFusion::updateBiasForgettingTime(float biasForgettingTime) {
	vqf.updateBiasForgettingTime(biasForgettingTime);
}

bool SensorFusion::getRestDetected() const { return vqf.getRestDetected(); }

bool SensorFusion::getMagDistDetected() const { return vqf.getMagDistDetected(); }

sensor_real_t SensorFusion::getMagRefNorm() const { return vqf.getMagRefNorm(); }

sensor_real_t SensorFusion::getMagRefDip() const { return vqf.getMagRefDip(); }

sensor_real_t SensorFusion::getDelta() const { return vqf.getDelta(); }

void SensorFusion::getBiasEstimate(sensor_real_t out[3]) const {
	vqf.getBiasEstimate(out);
}

void SensorFusion::getRelativeRestDeviations(sensor_real_t out[2]) const {
	vqf.getRelativeRestDeviations(out);
}

void SensorFusion::disableMag() { magExist = false; }

bool SensorFusion::seedMagRef() {
	if (!magExist) {
		return false;
	}

	sensor_real_t q6[4];
	vqf.getQuat6D(q6);

	// VQF's reference is stored as the norm of the field and its dip angle, both
	// measured in the earth frame, which is the same frame its own disturbance
	// detection compares against.
	sensor_real_t magEarth[3];
	VQF::quatRotate(q6, lastMag, magEarth);

	sensor_real_t fieldNorm = VQF::norm(magEarth, 3);
	if (fieldNorm <= 0.0f) {
		return false;
	}

	vqf.setMagRef(fieldNorm, -asin(magEarth[2] / fieldNorm));

	// updateMag drives delta towards the angle this same field vector implies, but
	// only with a time constant of tauMag (9 s), so adopting the reference on its
	// own leaves the heading creeping for the best part of a minute afterwards --
	// which shows up as a tracker that will not stay where it was zeroed. Placing
	// delta on the equilibrium right away makes the heading hold from the first
	// sample. Drift correction is unaffected: the equilibrium is where the measured
	// field heading and delta agree, so a gyro error rotates the measurement away
	// from it and delta follows again.
	vqf.setDelta(atan2(magEarth[0], magEarth[1]));
	return true;
}

}  // namespace SlimeVR::Sensors
