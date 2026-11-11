#include "../Driver/PoseFilter.h"
#include "../Driver/ProtocolValidation.h"
#include "../Driver/ServerTrackedDeviceProvider.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>

// The pose filters of Driver/PoseFilter.h on made-up streams shaped like a
// lighthouse device's: a new pose every 4 ms, most of them sent a second time
// with the same pose time and the driver's correction.
namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal::posefilter;

constexpr double Pi = 3.14159265358979323846;
constexpr double Tick = 0.004;

double Distance(const double a[3], const double b[3])
{
	return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

// The angle between two rotations, whatever their signs.
double Angle(const double a[4], const double b[4])
{
	double dot = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
	if (dot > 1.0)
		dot = 1.0;
	return 2.0 * std::acos(dot);
}

void AboutZ(double angle, double out[4])
{
	out[0] = std::cos(0.5 * angle);
	out[1] = 0.0;
	out[2] = 0.0;
	out[3] = std::sin(0.5 * angle);
}

// A repeatable stand-in for noise: values in [-1, 1).
struct Noise
{
	uint32_t seed;
	double Next()
	{
		seed = seed * 1664525u + 1013904223u;
		return static_cast<double>(seed >> 8) / 8388608.0 - 1.0;
	}
};

// One device's stream through one filter. The capture time runs behind the
// pose time's clock by a delay that changes from sample to sample, as a real
// one does; the offset the driver reports makes up for it. The delays keep
// the capture times in order, as the driver's clock is.
struct Stream
{
	State state;
	Settings settings;
	double clock = 100.0;           // the time the device's own clock gives its poses
	Noise jitter{ 7u };
	bool velocities = true;         // false: the driver reports none
	bool poseTimeStands = false;    // true: the offset cancels the capture time

	// A new pose: the clock moves on by `seconds`.
	Pose New(const Pose &raw, double seconds = Tick)
	{
		clock += seconds;
		return Send(raw, 0.0055 + 0.001 * jitter.Next());
	}

	// The same pose time again, a moment later.
	Pose Again(const Pose &raw)
	{
		return Send(raw, 0.0072 + 0.0002 * jitter.Next());
	}

	Pose Send(Pose raw, double delay)
	{
		const double capture = clock + delay;
		raw.timeOffset = poseTimeStands ? -capture : -delay;
		if (!velocities)
			for (int i = 0; i < 3; ++i)
				raw.velocity[i] = raw.angularVelocity[i] = 0.0;
		Apply(state, settings, capture, true, raw);
		return raw;
	}
};

Settings Smooth(double minCutoff = 1.0, double gain = 160.0)
{
	Settings settings;
	settings.smooth = true;
	settings.minCutoffHz = settings.turnMinCutoffHz = minCutoff;
	settings.speedGain = gain;
	return settings;
}

Settings Release(double seconds = 0.1)
{
	Settings settings;
	settings.releaseCorrections = true;
	settings.releaseSeconds = seconds;
	return settings;
}

bool SameBits(const Pose &a, const Pose &b)
{
	return memcmp(&a, &b, sizeof(Pose)) == 0;
}

// A device at rest whose driver shakes it: how much of the shake is left.
double ShakeLeft(const Settings &settings)
{
	Stream filtered;
	filtered.settings = settings;
	Noise noise{ 99u };
	double sumRaw = 0.0, sumFiltered = 0.0;
	Pose pose;
	for (int i = 0; i < 2500; ++i)
	{
		for (int axis = 0; axis < 3; ++axis)
			pose.position[axis] = 1.0 + 0.0001 * noise.Next();
		pose.velocity[0] = 0.002 * noise.Next();
		const Pose out = filtered.New(pose);
		// Half of the poses come again with a correction.
		Pose corrected = pose;
		corrected.position[1] += 0.00006 * noise.Next();
		const Pose last = i % 2 ? filtered.Again(corrected) : out;
		const Pose &sent = i % 2 ? corrected : pose;
		if (i >= 500)
		{
			const double rest[3] = { 1.0, 1.0, 1.0 };
			sumRaw += Distance(sent.position, rest) * Distance(sent.position, rest);
			sumFiltered += Distance(last.position, rest) * Distance(last.position, rest);
		}
	}
	return std::sqrt(sumFiltered / sumRaw);
}

// A device that moves along x at a steady speed: how away the output trails
// once it has settled.
double Trail(Stream &stream, double speed)
{
	Pose pose;
	pose.velocity[0] = speed;
	double trail = 0.0;
	for (int i = 0; i < 1500; ++i)
	{
		pose.position[0] = speed * Tick * i;
		const Pose out = stream.New(pose);
		trail = pose.position[0] - out.position[0];
		if (i % 2)
			stream.Again(pose);
	}
	return trail;
}

void SettingsScenarios(Check check)
{
	// Nothing on: every member of every sample as it came.
	Stream stream;
	Noise noise{ 5u };
	bool same = ChangesNothing(stream.settings);
	Pose pose;
	for (int i = 0; i < 400 && same; ++i)
	{
		for (int axis = 0; axis < 3; ++axis)
		{
			pose.position[axis] += 0.01 * noise.Next();
			pose.velocity[axis] = noise.Next();
			pose.angularVelocity[axis] = noise.Next();
			pose.angularAcceleration[axis] = noise.Next();
		}
		// A rotation that is not of unit length stays as it is.
		pose.rotation[0] = 0.7 + 0.1 * noise.Next();
		pose.rotation[2] = 0.7;
		Pose sent = pose;
		const Pose out = i % 3 ? stream.New(pose) : stream.Again(pose);
		sent.timeOffset = out.timeOffset;
		same = SameBits(out, sent);
	}
	check("pose filter: settings that change nothing leave every sample bit for bit", same, "");

	Settings bad[8];
	bad[0].minCutoffHz = 0.0;
	bad[1].turnMinCutoffHz = 1000.0;
	bad[2].speedGain = -1.0;
	bad[3].turnGain = std::numeric_limits<double>::infinity();
	bad[4].releaseSeconds = 0.0;
	bad[5].releaseSeconds = std::numeric_limits<double>::quiet_NaN();
	bad[6].predictionScale = 1.5;
	bad[7].predictionScale = -0.1;
	bool refused = Usable(Settings()) && Usable(Smooth(0.5, 80.0)) && Usable(Release(0.005));
	for (const Settings &settings : bad)
		refused = refused && !Usable(settings);
	check("pose filter: settings outside their limits are refused", refused, "");

	// The prediction scale: the three velocities and nothing else.
	Stream scaled;
	scaled.settings.predictionScale = 0.25;
	Pose moving;
	moving.position[0] = 0.5;
	moving.velocity[1] = 2.0;
	moving.angularVelocity[2] = -4.0;
	moving.angularAcceleration[0] = 8.0;
	scaled.New(moving);
	const Pose out = scaled.New(moving);
	const bool onlyVelocities = out.position[0] == 0.5 && out.rotation[0] == 1.0 && out.velocity[1] == 0.5 &&
		out.angularVelocity[2] == -1.0 && out.angularAcceleration[0] == 2.0 && !ChangesNothing(scaled.settings);
	check("pose filter: the prediction scale changes the velocities and nothing else", onlyVelocities, "");
}

void SmoothingScenarios(Check check)
{
	char detail[240];
	const double light = ShakeLeft(Smooth(2.0, 320.0)), medium = ShakeLeft(Smooth(1.0, 160.0)),
		strong = ShakeLeft(Smooth(0.5, 80.0));
	snprintf(detail, sizeof detail, "shake left: light %.3f, medium %.3f, strong %.3f", light, medium, strong);
	check("pose filter: a device at rest shakes less, and less again with a lower cutoff",
		light < 0.6 && medium < 0.4 && strong < medium && medium < light, detail);

	// The trail is v / (2 pi (cutoff + gain v)), under 1 / (2 pi gain) at every speed.
	bool within = true;
	std::string trails;
	for (const double speed : { 0.01, 0.05, 0.3, 1.0, 4.0 })
	{
		Stream stream;
		stream.settings = Smooth(1.0, 160.0);
		const double trail = Trail(stream, speed);
		const double cutoff = 1.0 + 160.0 * speed, most = speed / (2.0 * Pi * cutoff);
		// What a low-pass step leaves behind a ramp, exactly.
		const double expected = speed * Tick / std::expm1(2.0 * Pi * cutoff * Tick);
		within = within && trail <= most && std::fabs(trail - expected) <= 0.01 * expected + 1.0e-12 &&
			trail < 1.0 / (2.0 * Pi * 160.0);
		snprintf(detail, sizeof detail, "%s%.2f m/s: %.4f mm, at most %.3f", trails.empty() ? "" : ", ", speed,
			1000.0 * trail, 1000.0 * most);
		trails += detail;
	}
	check("pose filter: at a steady speed the smoothed position trails by no more than 1 / (2 pi gain)", within,
		trails.c_str());

	// A steady turn about z.
	bool turnWithin = true;
	std::string turns;
	for (const double rate : { 0.1, 1.0, 6.0 })
	{
		Stream stream;
		stream.settings = Smooth(1.0, 160.0);
		Pose pose;
		pose.angularVelocity[2] = rate;
		double behind = 0.0;
		for (int i = 0; i < 1500; ++i)
		{
			AboutZ(rate * Tick * i, pose.rotation);
			const Pose out = stream.New(pose);
			behind = Angle(out.rotation, pose.rotation);
		}
		const double cutoff = 1.0 + stream.settings.turnGain * rate, most = rate / (2.0 * Pi * cutoff);
		const double expected = rate * Tick / std::expm1(2.0 * Pi * cutoff * Tick);
		// An angle this near zero is known to about 1e-7 rad.
		turnWithin = turnWithin && behind <= most + 1.0e-7 && std::fabs(behind - expected) <= 0.01 * expected + 1.0e-7;
		snprintf(detail, sizeof detail, "%s%.1f rad/s: %.5f deg, at most %.4f", turns.empty() ? "" : ", ", rate,
			behind * 180.0 / Pi, most * 180.0 / Pi);
		turns += detail;
	}
	check("pose filter: a steady turn trails by no more than 1 / (2 pi turn gain)", turnWithin, turns.c_str());

	// A driver that reports no velocity: the speed worked out from the
	// positions opens the filter, late and less away.
	Stream silent;
	silent.settings = Smooth(1.0, 160.0);
	silent.velocities = false;
	const double silentTrail = Trail(silent, 1.0);
	snprintf(detail, sizeof detail, "trail at 1 m/s with no reported velocity: %.3f mm", 1000.0 * silentTrail);
	check("pose filter: a driver that reports no velocity is followed by the speed worked out from its positions",
		silentTrail > 0.0 && silentTrail < 0.0015, detail);

	// A pose time that never moves: after MaxSamePoseSeconds the capture time
	// is the clock, and the device is followed as closely as any other.
	Stream stuck;
	stuck.settings = Smooth(1.0, 160.0);
	stuck.settings.releaseCorrections = true;
	stuck.poseTimeStands = true;
	Pose pose;
	pose.velocity[0] = 1.0;
	double worst = 0.0, last = 0.0;
	for (int i = 0; i < 1500; ++i)
	{
		pose.position[0] = Tick * i;
		const Pose out = stuck.New(pose);
		last = pose.position[0] - out.position[0];
		if (last > worst)
			worst = last;
	}
	snprintf(detail, sizeof detail, "worst %.2f mm behind, %.3f mm at the end", 1000.0 * worst, 1000.0 * last);
	check("pose filter: a stream whose pose time never moves is still followed",
		stuck.state.captureIsPoseTime && worst < 1.0 * (MaxSamePoseSeconds + 2.0 * Tick) + 0.002 && last < 0.0015, detail);
}

void ReleaseScenarios(Check check)
{
	char detail[240];
	Stream stream;
	stream.settings = Release(0.1);
	Pose pose;
	pose.position[0] = 2.0;
	for (int i = 0; i < 50; ++i)
		stream.New(pose);
	// The driver corrects the pose by 5 mm with no new pose time.
	Pose corrected = pose;
	corrected.position[1] += 0.005;
	const Pose atCorrection = stream.Again(corrected);
	const double stepOut = Distance(atCorrection.position, pose.position);
	double afterTenth = 0.0, afterSecond = 0.0;
	for (int i = 1; i <= 300; ++i)
	{
		const Pose out = stream.New(corrected);
		if (i == 25)
			afterTenth = Distance(out.position, corrected.position);
		afterSecond = Distance(out.position, corrected.position);
	}
	// Then the device itself moves 5 mm from one pose time to the next.
	Pose moved = corrected;
	moved.position[2] += 0.005;
	const Pose atMove = stream.New(moved);
	snprintf(detail, sizeof detail, "at the correction %.4f mm, %.3f mm left after 0.1 s, %.5f mm after 1.2 s; a movement "
		"is %.5f mm behind", 1000.0 * stepOut, 1000.0 * afterTenth, 1000.0 * afterSecond,
		1000.0 * Distance(atMove.position, moved.position));
	check("pose filter: a correction is let out slowly and a movement passes at once",
		stepOut < 0.0002 && afterTenth > 0.0012 && afterTenth < 0.0025 && afterSecond < 1.0e-6 &&
			Distance(atMove.position, moved.position) < 1.0e-6, detail);

	// The same for a turn: one degree about z.
	Stream turning;
	turning.settings = Release(0.1);
	Pose facing;
	for (int i = 0; i < 50; ++i)
		turning.New(facing);
	Pose turned = facing;
	AboutZ(Pi / 180.0, turned.rotation);
	const Pose atTurn = turning.Again(turned);
	const double turnStep = Angle(atTurn.rotation, facing.rotation);
	double turnTenth = 0.0, turnSecond = 0.0;
	for (int i = 1; i <= 300; ++i)
	{
		const Pose out = turning.New(turned);
		if (i == 25)
			turnTenth = Angle(out.rotation, turned.rotation);
		turnSecond = Angle(out.rotation, turned.rotation);
	}
	snprintf(detail, sizeof detail, "at the correction %.5f deg, %.4f deg left after 0.1 s, %.6f deg after 1.2 s",
		turnStep * 180.0 / Pi, turnTenth * 180.0 / Pi, turnSecond * 180.0 / Pi);
	check("pose filter: a turn correction is let out slowly",
		turnStep < 1.0e-9 && turnTenth > 0.25 * Pi / 180.0 && turnTenth < 0.5 * Pi / 180.0 &&
			turnSecond < 1.0e-6, detail);

	// More than the most that is held passes at once.
	Stream large;
	large.settings = Release(0.1);
	Pose start;
	for (int i = 0; i < 50; ++i)
		large.New(start);
	Pose away = start;
	away.position[0] = 0.08;
	AboutZ(0.3, away.rotation);
	const Pose atLarge = large.Again(away);
	const double held = Distance(atLarge.position, away.position), heldTurn = Angle(atLarge.rotation, away.rotation);
	snprintf(detail, sizeof detail, "held %.2f mm of 80 and %.4f rad of 0.3", 1000.0 * held, heldTurn);
	check("pose filter: more than the most that is held passes at once",
		held > 0.9 * MaxHeldMeters && held <= MaxHeldMeters * 1.0001 && heldTurn > 0.9 * MaxHeldRadians &&
			heldTurn <= MaxHeldRadians * 1.0001, detail);
}

void RestartScenarios(Check check)
{
	char detail[240];
	const auto settle = [](Stream &stream)
	{
		Pose pose;
		pose.velocity[0] = 0.01;
		for (int i = 0; i < 100; ++i)
		{
			pose.position[0] = 0.01 * Tick * i;
			stream.New(pose);
		}
	};
	// A pose that cannot be used goes out as it came, and the next good one is
	// a first pose again: it is not smoothed toward the old place.
	const double nan = std::numeric_limits<double>::quiet_NaN();
	int passed = 0, restarted = 0;
	for (int kind = 0; kind < 5; ++kind)
	{
		Stream stream;
		stream.settings = Smooth(0.5, 80.0);
		stream.settings.releaseCorrections = true;
		settle(stream);
		Pose bad;
		bad.position[0] = 5.0;
		bool usable = true;
		double capture = stream.clock + 0.01;
		if (kind == 0)
			bad.position[1] = nan;
		else if (kind == 1)
			bad.rotation[0] = 0.0;
		else if (kind == 2)
			bad.velocity[2] = std::numeric_limits<double>::infinity();
		else if (kind == 3)
			usable = false;
		else
			capture = nan;
		Pose out = bad;
		Apply(stream.state, stream.settings, capture, usable, out);
		passed += SameBits(out, bad) && !stream.state.primed;
		Pose good;
		good.position[0] = 5.0;
		const Pose first = stream.New(good);
		restarted += first.position[0] == 5.0;
	}
	snprintf(detail, sizeof detail, "%d of 5 passed untouched, %d of 5 started again", passed, restarted);
	check("pose filter: a pose that cannot be used passes untouched and starts the filter again",
		passed == 5 && restarted == 5, detail);

	// A silence, a capture clock that goes back, and a pose time that jumps.
	int fresh = 0;
	for (int kind = 0; kind < 3; ++kind)
	{
		Stream stream;
		stream.settings = Smooth(0.5, 80.0);
		settle(stream);
		Pose jumped;
		jumped.position[0] = 3.0;
		Pose out = jumped;
		if (kind == 0)
			out = stream.New(jumped, RestartSeconds + 0.05);
		else if (kind == 1)
		{
			out.timeOffset = -0.006;
			Apply(stream.state, stream.settings, stream.clock - 1.0, true, out);
		}
		else
		{
			out.timeOffset = 1.0;
			Apply(stream.state, stream.settings, stream.clock + 0.01, true, out);
		}
		fresh += out.position[0] == 3.0 && stream.state.primed;
	}
	snprintf(detail, sizeof detail, "%d of 3 started again at the new pose", fresh);
	check("pose filter: a silence, a clock that goes back and a jump of the pose time start the filter again",
		fresh == 3, detail);

	// The rotation goes out with unit length and the sign it came with.
	Stream signs;
	signs.settings = Smooth(1.0, 160.0);
	signs.settings.releaseCorrections = true;
	bool kept = true;
	Pose pose;
	pose.angularVelocity[2] = 1.0;
	for (int i = 0; i < 600 && kept; ++i)
	{
		AboutZ(Tick * i, pose.rotation);
		const double sign = (i / 37) % 2 ? -1.0 : 1.0;
		for (int k = 0; k < 4; ++k)
			pose.rotation[k] *= sign * 1.02;        // the driver's is a little long
		const Pose out = i % 2 ? signs.Again(pose) : signs.New(pose);
		const double length = std::sqrt(out.rotation[0] * out.rotation[0] + out.rotation[1] * out.rotation[1] +
			out.rotation[2] * out.rotation[2] + out.rotation[3] * out.rotation[3]);
		const double dot = out.rotation[0] * pose.rotation[0] + out.rotation[1] * pose.rotation[1] +
			out.rotation[2] * pose.rotation[2] + out.rotation[3] * pose.rotation[3];
		// The first pose of a stream goes out as it came.
		kept = i == 0 || (std::fabs(length - 1.0) < 1.0e-9 && dot > 0.0);
	}
	check("pose filter: a filtered rotation has unit length and the sign it came with", kept, "");

	// Streams of every kind at once, good and bad samples mixed: what goes out
	// is finite, and a device that then stays in one place gets its walk.
	bool finite = true, arrives = true;
	Noise noise{ 2024u };
	for (int trial = 0; trial < 40; ++trial)
	{
		Stream stream;
		stream.settings.smooth = noise.Next() > -0.5;
		stream.settings.releaseCorrections = noise.Next() > -0.5;
		stream.settings.minCutoffHz = 0.05 + 4.0 * (noise.Next() + 1.0);
		stream.settings.turnMinCutoffHz = 0.05 + 4.0 * (noise.Next() + 1.0);
		stream.settings.speedGain = 300.0 * (noise.Next() + 1.0);
		stream.settings.turnGain = 300.0 * (noise.Next() + 1.0);
		stream.settings.releaseSeconds = 0.005 + 0.4 * (noise.Next() + 1.0);
		stream.velocities = noise.Next() > 0.0;
		stream.poseTimeStands = noise.Next() > 0.5;
		arrives = arrives && Usable(stream.settings);
		Pose walk;
		for (int i = 0; i < 600; ++i)
		{
			for (int axis = 0; axis < 3; ++axis)
			{
				walk.position[axis] += 0.02 * noise.Next();
				walk.velocity[axis] = 5.0 * noise.Next();
				walk.angularVelocity[axis] = 10.0 * noise.Next();
			}
			for (int k = 0; k < 4; ++k)
				walk.rotation[k] = noise.Next();
			walk.rotation[0] += 0.01;
			Pose out;
			const double pick = noise.Next();
			if (pick > 0.9)
			{
				out = walk;
				out.position[0] = pick > 0.95 ? std::numeric_limits<double>::quiet_NaN() : walk.position[0];
				Apply(stream.state, stream.settings, stream.clock, pick > 0.95, out);
				continue;
			}
			out = pick > 0.2 ? stream.New(walk, Tick * (1.0 + 4.0 * (noise.Next() + 1.0))) : stream.Again(walk);
			for (int axis = 0; axis < 3; ++axis)
				finite = finite && std::isfinite(out.position[axis]);
			for (int k = 0; k < 4; ++k)
				finite = finite && std::isfinite(out.rotation[k]);
		}
		Pose rest = walk;
		AboutZ(0.3, rest.rotation);
		for (int axis = 0; axis < 3; ++axis)
			rest.velocity[axis] = rest.angularVelocity[axis] = 0.0;
		Pose out;
		for (int i = 0; i < 20000; ++i)
			out = stream.New(rest);
		arrives = arrives && Distance(out.position, rest.position) < 1.0e-6 && Angle(out.rotation, rest.rotation) < 1.0e-6;
	}
	check("pose filter: mixed streams give finite poses, and a device that stays in one place gets its pose",
		finite && arrives, finite ? (arrives ? "" : "a pose did not arrive") : "a value was not finite");
}

// The filters inside the driver: the settings arrive in a runtime state, and
// each device's poses pass through its filter on the way to the runtime.
void DriverScenarios(Check check)
{
	char detail[240];
	const auto still = []
	{
		vr::DriverPose_t pose{};
		pose.poseIsValid = true;
		pose.deviceIsConnected = true;
		pose.result = vr::TrackingResult_Running_OK;
		pose.qRotation.w = pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = 1.0;
		return pose;
	};
	// Device 3 is smoothed, device 4 is not, device 5 has its velocities cut.
	auto provider = std::make_unique<ServerTrackedDeviceProvider>();
	protocol::SetRuntimeState runtime;
	runtime.filterMask = uint64_t{ 1 } << 3;
	runtime.filter.smooth = 1;
	runtime.filter.speedGain = runtime.filter.turnGain = 0.0;
	runtime.predictionCut[5] = 0.75;
	protocol::RejectReason reason = protocol::RejectReason::StaleSession;
	bool accepted = provider->TrySetRuntimeState(runtime, &reason) && reason == protocol::RejectReason::None;
	double atStep[6] = {}, velocityAtStep[6] = {};
	for (int i = 0; i < 60; ++i)
	{
		provider->SetPoseTimeForTest(Tick * i);
		for (uint32_t id = 3; id <= 5; ++id)
		{
			vr::DriverPose_t pose = still();
			pose.vecPosition[0] = i >= 10 ? 0.1 : 0.0;
			pose.vecVelocity[1] = 2.0;
			provider->HandleDevicePoseUpdated(id, pose);
			if (i == 10)
			{
				atStep[id] = pose.vecPosition[0];
				velocityAtStep[id] = pose.vecVelocity[1];
			}
		}
	}
	// One step of a 1 Hz low-pass filter over 4 ms.
	const double expected = 0.1 * -std::expm1(-2.0 * Pi * 1.0 * Tick);
	snprintf(detail, sizeof detail, "at the step: device 3 at %.5f m (a filter step is %.5f), device 4 at %.3f, device 5 "
		"at %.3f with velocity %.2f of 2", atStep[3], expected, atStep[4], atStep[5], velocityAtStep[5]);
	check("pose filter: the driver filters the devices in the mask, and cuts the velocities it is told to",
		accepted && std::fabs(atStep[3] - expected) < 1.0e-9 && atStep[4] == 0.1 && atStep[5] == 0.1 &&
			velocityAtStep[3] == 2.0 && velocityAtStep[4] == 2.0 && velocityAtStep[5] == 0.5, detail);

	// Settings outside the limits refuse the whole state, and the filters stay
	// as they were.
	int refused = 0;
	for (int kind = 0; kind < 6; ++kind)
	{
		protocol::SetRuntimeState bad = runtime;
		bad.filterMask = ~uint64_t{ 0 };
		if (kind == 0)
			bad.filter.smooth = 2;
		else if (kind == 1)
			bad.filter.releaseCorrections = 7;
		else if (kind == 2)
			bad.filter.minCutoffHz = 0.0;
		else if (kind == 3)
			bad.filter.turnGain = std::numeric_limits<double>::quiet_NaN();
		else if (kind == 4)
			bad.filter.releaseSeconds = 30.0;
		else
			bad.predictionCut[63] = 1.5;
		reason = protocol::RejectReason::None;
		questcal::driverinput::PoseFilterState out;
		refused += !provider->TrySetRuntimeState(bad, &reason) && reason == protocol::RejectReason::InvalidValues &&
			!questcal::driverinput::ValidatePoseFilters(bad, out) && out.filterMask == 0;
	}
	const auto stepAgain = [&](uint32_t id)
	{
		// A silence starts the filter again; then a step shows whether it is on.
		provider->SetPoseTimeForTest(10.0);
		vr::DriverPose_t pose = still();
		provider->HandleDevicePoseUpdated(id, pose);
		provider->SetPoseTimeForTest(10.0 + Tick);
		pose = still();
		pose.vecPosition[0] = 0.1;
		provider->HandleDevicePoseUpdated(id, pose);
		return pose.vecPosition[0];
	};
	const double keptOn = stepAgain(3), keptOff = stepAgain(4);
	snprintf(detail, sizeof detail, "%d of 6 refused; after them device 3 steps to %.5f m and device 4 to %.3f", refused,
		keptOn, keptOff);
	check("pose filter: the driver refuses a state whose filter settings are outside the limits, and keeps its filters",
		refused == 6 && std::fabs(keptOn - expected) < 1.0e-9 && keptOff == 0.1, detail);

	// With the calibration on the same device: the filter works on the pose as
	// its driver made it, and the calibration then moves the result.
	auto both = std::make_unique<ServerTrackedDeviceProvider>();
	protocol::SetRuntimeState calibrated = runtime;
	calibrated.enabledMask = uint64_t{ 1 } << 3;
	calibrated.transform.generation = 1;
	calibrated.transform.translation.v[0] = 0.3;
	accepted = both->TrySetRuntimeState(calibrated);
	double worldAtStep = 0.0, worldAtEnd = 0.0;
	for (int i = 0; i < 2000; ++i)
	{
		both->SetPoseTimeForTest(Tick * i);
		vr::DriverPose_t pose = still();
		pose.vecPosition[0] = i >= 10 ? 0.1 : 0.0;
		both->HandleDevicePoseUpdated(3, pose);
		worldAtEnd = pose.vecWorldFromDriverTranslation[0] + pose.vecPosition[0];
		if (i == 10)
			worldAtStep = worldAtEnd;
	}
	snprintf(detail, sizeof detail, "at the step %.5f m (0.3 + %.5f), at rest %.9f m", worldAtStep, expected, worldAtEnd);
	check("pose filter: the calibration moves the filtered pose",
		accepted && std::fabs(worldAtStep - (0.3 + expected)) < 1.0e-9 && std::fabs(worldAtEnd - 0.4) < 1.0e-9, detail);

	// A pose another driver changed on its way to the pose hook, with the pose
	// before it at hand: a device that is post-processed here gets every part of
	// its own motion back and keeps the rest of what that driver made of it.
	auto behind = std::make_unique<ServerTrackedDeviceProvider>();
	accepted = behind->TrySetRuntimeState(runtime);
	vr::DriverPose_t own = still();
	own.poseTimeOffset = -0.011;
	own.qRotation.w = 0.8;
	own.qRotation.y = 0.6;
	for (int i = 0; i < 3; ++i)
	{
		own.vecPosition[i] = 0.2 + i;
		own.vecVelocity[i] = 2.0 + i;
		own.vecAcceleration[i] = 3.0 + i;
		own.vecAngularVelocity[i] = 4.0 + i;
		own.vecAngularAcceleration[i] = 8.0 + i;
	}
	const auto changedByOther = [&]
	{
		vr::DriverPose_t pose = own;
		pose.poseTimeOffset = 0.0;
		pose.qRotation = still().qRotation;
		for (int i = 0; i < 3; ++i)
		{
			pose.vecPosition[i] += 0.5;
			pose.vecVelocity[i] = pose.vecAcceleration[i] = 0.0;
			pose.vecAngularVelocity[i] = pose.vecAngularAcceleration[i] = 0.0;
		}
		pose.deviceIsConnected = false;
		pose.vecWorldFromDriverTranslation[2] = 7.0;
		return pose;
	};
	behind->SetPoseTimeForTest(1.0);
	// Device 5 has its velocities cut to a quarter; device 4 has no filter.
	vr::DriverPose_t cut = changedByOther(), plain = changedByOther(), same = own;
	behind->HandleDevicePoseUpdated(5, cut, {}, false, &own);
	const uint32_t afterCut = behind->OwnMotionRestored();
	behind->HandleDevicePoseUpdated(4, plain, {}, false, &own);
	const uint32_t afterPlain = behind->OwnMotionRestored();
	// A pose that driver left as it was is not counted.
	behind->SetPoseTimeForTest(1.0 + Tick);
	behind->HandleDevicePoseUpdated(5, same, {}, false, &own);
	const uint32_t afterSame = behind->OwnMotionRestored();
	bool ownMotion = cut.poseTimeOffset == own.poseTimeOffset && cut.qRotation.w == 0.8 && cut.qRotation.y == 0.6;
	bool otherKept = plain.poseTimeOffset == 0.0 && plain.qRotation.w == 1.0;
	for (int i = 0; i < 3; ++i)
	{
		ownMotion = ownMotion && cut.vecPosition[i] == own.vecPosition[i] &&
			cut.vecVelocity[i] == own.vecVelocity[i] * 0.25 && cut.vecAcceleration[i] == own.vecAcceleration[i] &&
			cut.vecAngularVelocity[i] == own.vecAngularVelocity[i] * 0.25 &&
			cut.vecAngularAcceleration[i] == own.vecAngularAcceleration[i] * 0.25;
		otherKept = otherKept && plain.vecPosition[i] == own.vecPosition[i] + 0.5 && plain.vecVelocity[i] == 0.0;
	}
	const bool restKept = !cut.deviceIsConnected && cut.vecWorldFromDriverTranslation[2] == 7.0 && cut.poseIsValid;
	snprintf(detail, sizeof detail, "accepted %d; filtered device: own motion %d (x %.2f, velocity %.3f), the rest kept "
		"%d; device with no filter keeps the other driver's pose %d (x %.2f); counted %u, %u, %u; published %u",
		accepted, ownMotion, cut.vecPosition[0], cut.vecVelocity[0], restKept, otherKept, plain.vecPosition[0],
		afterCut, afterPlain, afterSame, behind->PublishedForTest());
	check("pose filter: a filtered device gets its own motion back from a pose another driver changed",
		accepted && ownMotion && restKept && otherKept && afterCut == 1 && afterPlain == 1 && afterSame == 1 &&
			behind->PublishedForTest() == 0, detail);
}

} // namespace

void RunPoseFilterScenarios(Check check)
{
	SettingsScenarios(check);
	SmoothingScenarios(check);
	ReleaseScenarios(check);
	RestartScenarios(check);
	DriverScenarios(check);
}
