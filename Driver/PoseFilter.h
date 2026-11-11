#pragma once

#include <cmath>
#include <cstdint>

// Post-processing of one device's driver poses, as plain arithmetic: the
// driver's pose path and the test harness run the same code. Nothing here
// waits or allocates. A pose it cannot use (a value that is not finite, a
// rotation of no length, a device with no valid pose) passes through as it
// came, and the device's filter starts again.
//
// What the filters stand on, measured on a recorded lighthouse session:
//
//  - A lighthouse device makes a new pose 250 times a second, and sends many
//    of them again, up to twelve times within a few milliseconds, with the
//    same pose time (capture time plus poseTimeOffset) and a small change.
//    That change is the driver's own correction from its base-station
//    measurements. A change between two pose times is movement. So a
//    correction and a movement can be told apart with no guess.
//  - The capture time jitters by milliseconds; the pose time does not. Every
//    interval a filter uses is a pose-time interval.
//  - The velocities a driver reports follow the movement with no measurable
//    delay. They can open a low-pass filter the moment the device moves, where
//    a speed worked out from the positions is late and noisy.
//
// A driver that does none of this (a pose time that never moves, no
// velocities) must still get poses that follow the device: see
// MaxSamePoseSeconds and the worked-out speed below.
namespace questcal
{
namespace posefilter
{

// What one device's filter does. Every member is checked by Usable() before
// it reaches Apply.
struct Settings
{
	// Smoothing: a low-pass filter whose cutoff rises with the speed the
	// driver reports (the "One Euro" filter, with the reported speed in the
	// place of its own estimate). At a steady speed v the output trails by
	// v / (2 pi (minCutoffHz + speedGain v)), which is never more than
	// 1 / (2 pi speedGain) meters: 1 mm at a gain of 160.
	bool smooth = false;
	double minCutoffHz = 1.0;            // the cutoff of a device at rest
	double speedGain = 160.0;            // Hz for each m/s
	double turnMinCutoffHz = 1.0;
	double turnGain = 100.0;             // Hz for each rad/s: at most 1 / (2 pi turnGain) rad behind

	// Corrections let out slowly: a correction the driver makes is held back
	// and let out over this time, so the pose glides to its new place. Movement
	// is not held back.
	bool releaseCorrections = false;
	double releaseSeconds = 0.1;

	// The reported velocities times this, as the add-on "OpenVR-SmoothTracking"
	// does: under 1 lowers the runtime's motion prediction. 1 changes nothing.
	double predictionScale = 1.0;
};

namespace limits
{
constexpr double MinCutoffHz = 0.05, MaxCutoffHz = 100.0;
constexpr double MaxGain = 10000.0;
constexpr double MinReleaseSeconds = 0.005, MaxReleaseSeconds = 1.0;
} // namespace limits

inline bool Usable(const Settings &s)
{
	const auto within = [](double value, double low, double high)
	{
		return std::isfinite(value) && value >= low && value <= high;
	};
	return within(s.minCutoffHz, limits::MinCutoffHz, limits::MaxCutoffHz) &&
		within(s.turnMinCutoffHz, limits::MinCutoffHz, limits::MaxCutoffHz) &&
		within(s.speedGain, 0.0, limits::MaxGain) && within(s.turnGain, 0.0, limits::MaxGain) &&
		within(s.releaseSeconds, limits::MinReleaseSeconds, limits::MaxReleaseSeconds) &&
		within(s.predictionScale, 0.0, 1.0);
}

inline bool ChangesNothing(const Settings &s)
{
	return !s.smooth && !s.releaseCorrections && s.predictionScale == 1.0;
}

// The part of a driver pose a filter reads and writes. Rotation is w, x, y, z.
struct Pose
{
	double position[3] = { 0.0, 0.0, 0.0 };
	double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };
	double velocity[3] = { 0.0, 0.0, 0.0 };
	double angularVelocity[3] = { 0.0, 0.0, 0.0 };
	double angularAcceleration[3] = { 0.0, 0.0, 0.0 };
	double timeOffset = 0.0;             // poseTimeOffset, seconds
};

// Two samples whose pose times differ by less than this are one pose sent
// twice. A recorded lighthouse device repeats within a microsecond and makes
// a new pose every 4 ms.
constexpr double SamePoseSeconds = 1.0e-4;
// A silence or a time step longer than this starts the filter again: nothing
// is smoothed or held across it.
constexpr double RestartSeconds = 0.25;
// The longest interval one smoothing step covers.
constexpr double MaxStepSeconds = 0.1;
// A recorded lighthouse device sends one pose time for 0.3 ms as a rule and
// 12 ms at the most. A stream whose pose time stands for longer than this
// while its poses keep coming does not say when they are from: from then on
// its capture time is its pose time.
constexpr double MaxSamePoseSeconds = 0.05;
// The most that is held back. More than this passes at once, so a stream that
// is misread as all corrections still follows the device this closely.
constexpr double MaxHeldMeters = 0.02;
constexpr double MaxHeldRadians = 0.05;
// The speed worked out from the positions counts only above these, and is
// averaged over this time: at rest it is noise.
constexpr double WorkedOutSpeedFloor = 0.02;     // m/s
constexpr double WorkedOutTurnFloor = 0.05;      // rad/s
constexpr double WorkedOutSeconds = 0.05;

struct State
{
	bool primed = false;
	bool captureIsPoseTime = false;      // the stream's pose time does not move: see MaxSamePoseSeconds
	double captureTime = 0.0;            // of the previous sample
	double poseTime = 0.0;
	double poseTimeSince = 0.0;          // the capture time at which that pose time first came
	double rawPosition[3] = { 0.0, 0.0, 0.0 };       // the previous sample as it came
	double rawRotation[4] = { 1.0, 0.0, 0.0, 0.0 };
	// The corrections not let out yet: a displacement, and a turn as a
	// rotation vector in the device's own frame.
	double heldPosition[3] = { 0.0, 0.0, 0.0 };
	double heldTurn[3] = { 0.0, 0.0, 0.0 };
	// The smoothed pose.
	double smoothPosition[3] = { 0.0, 0.0, 0.0 };
	double smoothRotation[4] = { 1.0, 0.0, 0.0, 0.0 };
	// The last pose with a new pose time, and the speeds worked out from one
	// such pose to the next.
	double newPoseTime = 0.0;
	double newPosition[3] = { 0.0, 0.0, 0.0 };
	double newRotation[4] = { 1.0, 0.0, 0.0, 0.0 };
	double workedOutVelocity[3] = { 0.0, 0.0, 0.0 };
	double workedOutTurn = 0.0;
};

namespace detail
{

inline double Length3(const double v[3])
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline bool Finite(const double *values, int count)
{
	for (int i = 0; i < count; ++i)
		if (!std::isfinite(values[i]))
			return false;
	return true;
}

inline void Multiply(const double a[4], const double b[4], double out[4])
{
	const double w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	const double x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	const double y = a[0] * b[2] + a[2] * b[0] + a[3] * b[1] - a[1] * b[3];
	const double z = a[0] * b[3] + a[3] * b[0] + a[1] * b[2] - a[2] * b[1];
	out[0] = w;
	out[1] = x;
	out[2] = y;
	out[3] = z;
}

// conjugate(a) * b: the turn from a to b, in a's own frame.
inline void TurnBetween(const double a[4], const double b[4], double out[4])
{
	const double conjugate[4] = { a[0], -a[1], -a[2], -a[3] };
	Multiply(conjugate, b, out);
}

// The rotation vector of a unit quaternion, the short way round.
inline void Log(const double q[4], double out[3])
{
	const double sign = q[0] < 0.0 ? -1.0 : 1.0;
	const double v[3] = { sign * q[1], sign * q[2], sign * q[3] };
	const double sine = Length3(v);
	if (sine < 1.0e-12)
	{
		out[0] = 2.0 * v[0];
		out[1] = 2.0 * v[1];
		out[2] = 2.0 * v[2];
		return;
	}
	const double scale = 2.0 * std::atan2(sine, sign * q[0]) / sine;
	out[0] = v[0] * scale;
	out[1] = v[1] * scale;
	out[2] = v[2] * scale;
}

inline void Exp(const double r[3], double out[4])
{
	const double angle = Length3(r);
	const double scale = angle < 1.0e-12 ? 0.5 : std::sin(0.5 * angle) / angle;
	out[0] = std::cos(0.5 * angle);
	out[1] = r[0] * scale;
	out[2] = r[1] * scale;
	out[3] = r[2] * scale;
}

inline bool Normalize(double q[4])
{
	const double length = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (!(length > 1.0e-6) || !std::isfinite(length))
		return false;
	for (int i = 0; i < 4; ++i)
		q[i] /= length;
	return true;
}

inline void LimitLength(double v[3], double most)
{
	const double length = Length3(v);
	if (length > most)
		for (int i = 0; i < 3; ++i)
			v[i] *= most / length;
}

// 1 - exp(-2 pi cutoff dt): how far a low-pass step moves to its input.
inline double Blend(double cutoffHz, double seconds)
{
	return -std::expm1(-6.283185307179586 * cutoffHz * seconds);
}

} // namespace detail

inline void Reset(State &state)
{
	state = State();
}

// Scales what the runtime predicts with. The add-on "OpenVR-SmoothTracking"
// scales these three and leaves the linear acceleration alone.
inline void ScalePrediction(Pose &pose, double scale)
{
	for (int i = 0; i < 3; ++i)
	{
		pose.velocity[i] *= scale;
		pose.angularVelocity[i] *= scale;
		pose.angularAcceleration[i] *= scale;
	}
}

// Filters one sample of one device in place. `captureSeconds` is a clock that
// only goes forward (the driver's performance counter, in seconds) read when
// the sample arrived; `usable` is false for a device with no valid pose.
// `settings` must have passed Usable().
inline void Apply(State &state, const Settings &settings, double captureSeconds, bool usable, Pose &pose)
{
	using namespace detail;
	double rotation[4] = { pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3] };
	if (!usable || !std::isfinite(captureSeconds) || !std::isfinite(pose.timeOffset) || !Finite(pose.position, 3) ||
		!Finite(pose.rotation, 4) || !Finite(pose.velocity, 3) || !Finite(pose.angularVelocity, 3) ||
		!Finite(pose.angularAcceleration, 3) || !Normalize(rotation))
	{
		// What is known of the stream's clock outlives the break: a device that
		// goes in and out of tracking must not learn it again each time.
		const bool captureIsPoseTime = state.captureIsPoseTime;
		Reset(state);
		state.captureIsPoseTime = captureIsPoseTime;
		return;
	}

	double poseTime = state.captureIsPoseTime ? captureSeconds : captureSeconds + pose.timeOffset;
	const double sinceCapture = captureSeconds - state.captureTime;
	bool restart = !state.primed || !(sinceCapture >= 0.0) || sinceCapture > RestartSeconds ||
		std::fabs(poseTime - state.poseTime) > RestartSeconds;
	bool again = !restart && std::fabs(poseTime - state.poseTime) < SamePoseSeconds;
	if (again && captureSeconds - state.poseTimeSince > MaxSamePoseSeconds)
	{
		// The pose time stands still while poses keep coming: it is no clock.
		state.captureIsPoseTime = true;
		poseTime = captureSeconds;
		state.poseTime = state.captureTime;
		state.newPoseTime = state.captureTime;
		again = false;
	}
	if (!again)
		state.poseTimeSince = captureSeconds;

	if (restart)
	{
		const bool captureIsPoseTime = state.captureIsPoseTime;
		Reset(state);
		state.primed = true;
		state.captureIsPoseTime = captureIsPoseTime;
		poseTime = captureIsPoseTime ? captureSeconds : captureSeconds + pose.timeOffset;
		for (int i = 0; i < 3; ++i)
			state.rawPosition[i] = state.smoothPosition[i] = state.newPosition[i] = pose.position[i];
		for (int i = 0; i < 4; ++i)
			state.rawRotation[i] = state.smoothRotation[i] = state.newRotation[i] = rotation[i];
		state.captureTime = state.poseTimeSince = captureSeconds;
		state.poseTime = state.newPoseTime = poseTime;
		ScalePrediction(pose, settings.predictionScale);
		return;
	}

	// The speeds worked out from one new pose to the next, for a driver that
	// reports none.
	const double step = again ? 0.0 : (poseTime > state.poseTime ? poseTime - state.poseTime : 0.0);
	if (!again)
	{
		const double sinceNew = poseTime - state.newPoseTime;
		if (sinceNew >= SamePoseSeconds)
		{
			const double share = -std::expm1(-sinceNew / WorkedOutSeconds);
			for (int i = 0; i < 3; ++i)
			{
				const double velocity = (pose.position[i] - state.newPosition[i]) / sinceNew;
				state.workedOutVelocity[i] += share * (velocity - state.workedOutVelocity[i]);
			}
			double turn[4], turned[3];
			TurnBetween(state.newRotation, rotation, turn);
			Log(turn, turned);
			state.workedOutTurn += share * (Length3(turned) / sinceNew - state.workedOutTurn);
			state.newPoseTime = poseTime;
			for (int i = 0; i < 3; ++i)
				state.newPosition[i] = pose.position[i];
			for (int i = 0; i < 4; ++i)
				state.newRotation[i] = rotation[i];
		}
	}

	// Corrections: what the driver changed with no new pose time is held back,
	// and what is held back goes out with time.
	double releasedPosition[3] = { pose.position[0], pose.position[1], pose.position[2] };
	double releasedRotation[4] = { rotation[0], rotation[1], rotation[2], rotation[3] };
	if (settings.releaseCorrections)
	{
		const double keep = std::exp(-sinceCapture / settings.releaseSeconds);
		for (int i = 0; i < 3; ++i)
		{
			state.heldPosition[i] *= keep;
			state.heldTurn[i] *= keep;
		}
		if (again)
		{
			for (int i = 0; i < 3; ++i)
				state.heldPosition[i] += pose.position[i] - state.rawPosition[i];
			double held[4], correction[4], both[4];
			Exp(state.heldTurn, held);
			TurnBetween(state.rawRotation, rotation, correction);
			Multiply(held, correction, both);
			Log(both, state.heldTurn);
		}
		LimitLength(state.heldPosition, MaxHeldMeters);
		LimitLength(state.heldTurn, MaxHeldRadians);
		const double back[3] = { -state.heldTurn[0], -state.heldTurn[1], -state.heldTurn[2] };
		double unturn[4];
		Exp(back, unturn);
		Multiply(rotation, unturn, releasedRotation);
		for (int i = 0; i < 3; ++i)
			releasedPosition[i] -= state.heldPosition[i];
	}
	else
	{
		for (int i = 0; i < 3; ++i)
			state.heldPosition[i] = state.heldTurn[i] = 0.0;
	}

	if (settings.smooth)
	{
		const double seconds = step < MaxStepSeconds ? step : MaxStepSeconds;
		double speed = Length3(pose.velocity);
		const double workedOut = Length3(state.workedOutVelocity) - WorkedOutSpeedFloor;
		if (workedOut > speed)
			speed = workedOut;
		const double blend = Blend(settings.minCutoffHz + settings.speedGain * speed, seconds);
		for (int i = 0; i < 3; ++i)
			state.smoothPosition[i] += blend * (releasedPosition[i] - state.smoothPosition[i]);

		double turnRate = Length3(pose.angularVelocity);
		if (state.workedOutTurn - WorkedOutTurnFloor > turnRate)
			turnRate = state.workedOutTurn - WorkedOutTurnFloor;
		const double turnBlend = Blend(settings.turnMinCutoffHz + settings.turnGain * turnRate, seconds);
		double toInput[4], vector[3], part[4], next[4];
		TurnBetween(state.smoothRotation, releasedRotation, toInput);
		Log(toInput, vector);
		for (int i = 0; i < 3; ++i)
			vector[i] *= turnBlend;
		Exp(vector, part);
		Multiply(state.smoothRotation, part, next);
		if (Normalize(next))
			for (int i = 0; i < 4; ++i)
				state.smoothRotation[i] = next[i];
		else
			for (int i = 0; i < 4; ++i)
				state.smoothRotation[i] = releasedRotation[i];
	}
	else
	{
		for (int i = 0; i < 3; ++i)
			state.smoothPosition[i] = releasedPosition[i];
		for (int i = 0; i < 4; ++i)
			state.smoothRotation[i] = releasedRotation[i];
	}

	for (int i = 0; i < 3; ++i)
		state.rawPosition[i] = pose.position[i];
	for (int i = 0; i < 4; ++i)
		state.rawRotation[i] = rotation[i];
	state.captureTime = captureSeconds;
	state.poseTime = poseTime;

	if (settings.smooth || settings.releaseCorrections)
	{
		// The rotation goes out with the sign it came in with.
		double out[4] = { state.smoothRotation[0], state.smoothRotation[1], state.smoothRotation[2],
			state.smoothRotation[3] };
		if (out[0] * rotation[0] + out[1] * rotation[1] + out[2] * rotation[2] + out[3] * rotation[3] < 0.0)
			for (int i = 0; i < 4; ++i)
				out[i] = -out[i];
		const bool ok = Finite(state.smoothPosition, 3) && Finite(out, 4);
		if (!ok)
		{
			// Arithmetic on finite input should not get here. If it does, the
			// pose goes out as it came and the filter starts again.
			Reset(state);
			ScalePrediction(pose, settings.predictionScale);
			return;
		}
		for (int i = 0; i < 3; ++i)
			pose.position[i] = state.smoothPosition[i];
		for (int i = 0; i < 4; ++i)
			pose.rotation[i] = out[i];
	}
	ScalePrediction(pose, settings.predictionScale);
}

} // namespace posefilter
} // namespace questcal
