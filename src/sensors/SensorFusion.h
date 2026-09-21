#ifndef SLIMEVR_SENSORFUSION_H
#define SLIMEVR_SENSORFUSION_H

#include "globals.h"
#include "sensor.h"

#define SENSOR_DOUBLE_PRECISION 0

#define SENSOR_FUSION_TYPE_STRING "vqf"

#include <vqf.h>

#include "../motionprocessing/types.h"

namespace SlimeVR::Sensors {
class SensorFusion {
public:
	SensorFusion(
		VQFParams vqfParams,
		sensor_real_t gyrTs,
		sensor_real_t accTs = -1.0,
		sensor_real_t magTs = -1.0
	)
		: gyrTs(gyrTs)
		, accTs((accTs < 0) ? gyrTs : accTs)
		, magTs((magTs < 0) ? gyrTs : magTs)
		, vqfParams(vqfParams)
		, vqf(this->vqfParams,
			  gyrTs,
			  ((accTs < 0) ? gyrTs : accTs),
			  ((magTs < 0) ? gyrTs : magTs)) {}

	explicit SensorFusion(
		sensor_real_t gyrTs,
		sensor_real_t accTs = -1.0,
		sensor_real_t magTs = -1.0
	)
		// Every driver passes its own VQFParams, so this overload exists for callers
		// that have none and uses the same defaults they do. It used to reference a
		// DefaultVQFParams constant defined here, which no driver ever ran on: the
		// values in force are the VQFParams member initialisers in lib/vqf/vqf.h.
		: SensorFusion(VQFParams{}, gyrTs, accTs, magTs) {}

	void update6D(
		sensor_real_t Axyz[3],
		sensor_real_t Gxyz[3],
		sensor_real_t deltat = -1.0f
	);
	void update9D(
		sensor_real_t Axyz[3],
		sensor_real_t Gxyz[3],
		sensor_real_t Mxyz[3],
		sensor_real_t deltat = -1.0f
	);
	void updateAcc(const sensor_real_t Axyz[3], sensor_real_t deltat = -1.0f);
	void updateMag(const sensor_real_t Mxyz[3], sensor_real_t deltat = -1.0f);
	void updateGyro(const sensor_real_t Gxyz[3], sensor_real_t deltat = -1.0f);

	bool isUpdated();
	void clearUpdated();
	sensor_real_t const* getQuaternion();
	Quat getQuaternionQuat();
	sensor_real_t const* getGravityVec();
	sensor_real_t const* getLinearAcc();
	void getLinearAcc(sensor_real_t outLinAccel[3]);
	Vector3 getLinearAccVec();

	static void calcGravityVec(const sensor_real_t qwxyz[4], sensor_real_t gravVec[3]);
	static void calcLinearAcc(
		const sensor_real_t accin[3],
		const sensor_real_t gravVec[3],
		sensor_real_t accout[3]
	);

	void updateBiasForgettingTime(float biasForgettingTime);

	[[nodiscard]] bool getRestDetected() const;
	[[nodiscard]] bool getMagDistDetected() const;
	// Zero until a magnetic field reference has been accepted, which VQF only
	// does while the tracker is moving.
	[[nodiscard]] sensor_real_t getMagRefNorm() const;
	[[nodiscard]] sensor_real_t getMagRefDip() const;
	// Heading correction angle, in radians.
	[[nodiscard]] sensor_real_t getDelta() const;
	// Gyroscope bias estimate, in rad/s.
	void getBiasEstimate(sensor_real_t out[3]) const;
	// Relative rest deviations for gyro and accel; rest needs both below 1.
	void getRelativeRestDeviations(sensor_real_t out[2]) const;

	// Seeds VQF's magnetic field reference from the field currently observed,
	// brought into the earth frame with the 6D attitude. VQF normally only accepts
	// a reference while the tracker is moving (magNewMinGyr), which a tracker
	// sitting on a table never satisfies: magRefNorm stayed at zero, which leaves
	// magDistDetected permanently tripped, halves the heading correction gain, and
	// makes disturbance rejection unable to tell a real disturbance from the
	// normal field. Returns false when no magnetometer sample has been seen yet.
	//
	// Also snaps the heading correction (delta) onto the angle that field implies,
	// so the 9D heading holds immediately rather than converging over tauMag.
	bool seedMagRef();

	// Returns the filter to 6D. VQF::getQuat9D applies its stored heading
	// correction (state.delta), which only updateMag ever writes, so simply
	// ceasing to feed the mag would leave the tracker reporting a 9D attitude
	// pinned to a stale correction. getQuat6D never applies delta, so dropping
	// magExist is what actually switches the mag off.
	void disableMag();

protected:
	sensor_real_t gyrTs;
	sensor_real_t accTs;
	sensor_real_t magTs;

	VQFParams vqfParams;
	VQF vqf;

	// A also used for linear acceleration extraction
	sensor_real_t bAxyz[3]{0.0f, 0.0f, 0.0f};

	bool magExist = false;
	// Last magnetometer sample, in the body frame, kept so the field reference can
	// be seeded without another trip through the caller.
	sensor_real_t lastMag[3]{0.0f, 0.0f, 0.0f};
	sensor_real_t qwxyz[4]{1.0f, 0.0f, 0.0f, 0.0f};
	bool updated = false;

	bool gravityReady = false;
	sensor_real_t vecGravity[3]{0.0f, 0.0f, 0.0f};
	bool linaccelReady = false;
	sensor_real_t linAccel[3]{0.0f, 0.0f, 0.0f};
#ifdef ESP32
	sensor_real_t linAccel_guard;  // Temporary patch for some weird ESP32 bug
#endif
};
}  // namespace SlimeVR::Sensors

#endif  // SLIMEVR_SENSORFUSION_H
