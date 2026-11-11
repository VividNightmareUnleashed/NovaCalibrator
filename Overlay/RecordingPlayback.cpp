#include "RecordingPlayback.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace questcal::recording
{

namespace
{

// How far apart the kept record positions are, in recorded seconds: a jump
// reads at most this much more than the trail it has to rebuild.
constexpr double PointSpacingSeconds = 0.25;
// A stretch longer than this is a count the file made up: the sample that
// claims it keeps the time of the one before.
constexpr double MaxStretchSeconds = 7.0 * 24.0 * 3600.0;
// What one Frame call reads at most, about a minute of nine devices.
constexpr int FrameRecordBudget = 200000;
// Bounds on what an untrusted file can make this keep.
constexpr size_t MaxKeptRecords = 100000;
constexpr size_t MaxLosses = 4096;
constexpr size_t MaxMarks = 4096;
constexpr size_t MaxStretches = 4096;
constexpr size_t MaxDevices = 256;
constexpr size_t MaxSystems = 16;
constexpr double TrailStepSeconds = 1.0 / 90.0;
constexpr uint64_t ProgressStepBytes = 8ull << 20;

bool Finite3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// q (w x y z) turned to unit length; false when it has no direction.
bool Unit(const double q[4], double out[4])
{
	const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (!std::isfinite(n) || n < 1e-9)
		return false;
	for (int i = 0; i < 4; ++i)
		out[i] = q[i] / n;
	return true;
}

// v turned by the unit quaternion q.
void Rotate(const double q[4], const double v[3], double out[3])
{
	const double w = q[0], x = q[1], y = q[2], z = q[3];
	// t = 2 * cross(q.xyz, v); out = v + w * t + cross(q.xyz, t)
	const double tx = 2.0 * (y * v[2] - z * v[1]);
	const double ty = 2.0 * (z * v[0] - x * v[2]);
	const double tz = 2.0 * (x * v[1] - y * v[0]);
	out[0] = v[0] + w * tx + (y * tz - z * ty);
	out[1] = v[1] + w * ty + (z * tx - x * tz);
	out[2] = v[2] + w * tz + (x * ty - y * tx);
}

// a times b, both w x y z.
void Multiply(const double a[4], const double b[4], double out[4])
{
	out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// Seconds from a stretch's first sample to this one, never negative and never
// more than a count could mean.
double SecondsInto(int64_t qpc, int64_t baseQpc, double frequency, double soFar)
{
	const double seconds = (static_cast<double>(qpc) - static_cast<double>(baseQpc)) / frequency;
	if (!(seconds > soFar) || seconds > MaxStretchSeconds)
		return soFar;
	return seconds;
}

} // namespace

struct Playback::Index
{
	std::atomic<bool> cancel{ false };
	std::atomic<bool> ready{ false };

	std::mutex mutex;   // guards `summary`
	Summary summary;

	// Written by the first pass and read only once `ready` is set.
	double frequency = 1.0;
	std::vector<DeviceInfo> headerDevices;
	struct Span
	{
		uint64_t firstOffset = 0;   // of the first record that belongs to it
		int64_t baseQpc = 0;        // of its first sample
		double start = 0.0;
		double seconds = 0.0;
	};
	std::vector<Span> spans;
	struct Point
	{
		double time = 0.0;
		uint64_t offset = 0;
	};
	std::vector<Point> points;
	std::vector<std::pair<uint64_t, DeviceInfo>> deviceRecords;
	std::vector<std::pair<uint64_t, Alignment>> alignments;
	std::vector<std::pair<uint64_t, Model>> models;
};

struct Playback::Cursor
{
	std::unique_ptr<std::istream> stream;
	std::unique_ptr<Reader> reader;
	size_t span = 0;
	double spanSeconds = 0.0;
	double time = 0.0;
	bool hasPending = false;
	Record pending;
	double pendingTime = 0.0;
	// The samples of the block that was read last, and the next one to give.
	std::vector<Sample> block;
	size_t blockAt = 0;

	struct Device
	{
		DeviceInfo info;
		bool any = false;
		bool tracked = false;
		double position[3] = { 0.0, 0.0, 0.0 };
		double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };
		std::string renderModel;
		double lastTrail = -1e18;
		std::vector<std::pair<double, std::array<float, 3>>> trail;
	};
	std::vector<Device> devices;
	std::vector<Alignment> alignments;

	Device *Find(uint32_t id, bool create)
	{
		for (Device &device : devices)
			if (device.info.deviceId == id)
				return &device;
		if (!create || devices.size() >= MaxDevices)
			return nullptr;
		devices.emplace_back();
		devices.back().info.deviceId = id;
		return &devices.back();
	}

	void Describe(const DeviceInfo &info)
	{
		if (Device *device = Find(info.deviceId, true))
			device->info = info;
	}

	void Shape(const Model &model)
	{
		if (Device *device = Find(model.deviceId, true))
			device->renderModel = model.renderModel;
	}

	void Align(const Alignment &alignment)
	{
		for (Alignment &known : alignments)
			if (known.trackingSystem == alignment.trackingSystem)
			{
				known = alignment;
				return;
			}
		if (alignments.size() < MaxSystems)
			alignments.push_back(alignment);
	}

	// Nothing joins the two sides of a gap, a boundary or a pause.
	void CutTrails()
	{
		for (Device &device : devices)
		{
			device.trail.clear();
			device.lastTrail = -1e18;
		}
	}

	void Apply(const Sample &sample, double at)
	{
		Device *device = Find(sample.deviceId, true);
		if (!device)
			return;
		double driver[4], world[3], own[4], turned[4];
		const bool usable = sample.poseValid && sample.connected && Finite3(sample.position) &&
			Finite3(sample.worldFromDriverTranslation) && Unit(sample.worldFromDriverRotation, driver);
		device->tracked = usable;
		if (!usable)
			return;
		Rotate(driver, sample.position, world);
		for (int i = 0; i < 3; ++i)
			world[i] += sample.worldFromDriverTranslation[i];
		// A rotation with no direction leaves the device upright.
		if (Unit(sample.rotation, own))
			Multiply(driver, own, turned);
		else
			std::copy(driver, driver + 4, turned);
		for (const Alignment &alignment : alignments)
		{
			double turn[4];
			if (!alignment.applied || alignment.trackingSystem != device->info.trackingSystem ||
				!Unit(alignment.rotation, turn) || !Finite3(alignment.translation) || !std::isfinite(alignment.scale))
				continue;
			const double scaled[3] = { world[0] * alignment.scale, world[1] * alignment.scale, world[2] * alignment.scale };
			Rotate(turn, scaled, world);
			for (int i = 0; i < 3; ++i)
				world[i] += alignment.translation[i];
			std::copy(turned, turned + 4, own);
			Multiply(turn, own, turned);
			break;
		}
		if (!Finite3(world))
		{
			device->tracked = false;
			return;
		}
		device->any = true;
		for (int i = 0; i < 3; ++i)
			device->position[i] = world[i];
		std::copy(turned, turned + 4, device->rotation);
		if (at - device->lastTrail >= TrailStepSeconds)
		{
			device->lastTrail = at;
			device->trail.push_back({ at, { static_cast<float>(world[0]), static_cast<float>(world[1]),
				static_cast<float>(world[2]) } });
		}
	}
};

Playback::Playback() = default;

Playback::~Playback()
{
	Close();
}

void Playback::Close()
{
	std::shared_ptr<Index> index;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		index.swap(index_);
	}
	if (index)
		index->cancel.store(true, std::memory_order_release);
	if (thread_.joinable())
		thread_.join();
	cursor_.reset();
	source_ = nullptr;
}

void Playback::Open(const Source &source, uint64_t bytes, bool wait)
{
	Close();
	auto index = std::make_shared<Index>();
	index->summary.state = State::Reading;
	index->summary.bytes = bytes;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		index_ = index;
	}
	source_ = source;
	if (wait)
		Read(index, source, bytes);
	else
		thread_ = std::thread([index, source, bytes] { Read(index, source, bytes); });
}

Playback::Summary Playback::GetSummary() const
{
	std::shared_ptr<Index> index;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		index = index_;
	}
	if (!index)
		return Summary();
	std::lock_guard<std::mutex> lock(index->mutex);
	return index->summary;
}

// The first pass: every record once, front to back.
void Playback::Read(const std::shared_ptr<Index> &shared, const Source &source, uint64_t bytes)
{
	Index &index = *shared;
	Summary summary;
	summary.state = State::Reading;
	summary.bytes = bytes;
	const auto publish = [&]
	{
		std::lock_guard<std::mutex> lock(index.mutex);
		index.summary = summary;
	};
	const auto fail = [&](const std::string &why)
	{
		summary = Summary();
		summary.state = State::Failed;
		summary.bytes = bytes;
		summary.error = why;
		publish();
	};

	// An exception that leaves a thread's entry ends the program, and the
	// vectors here grow with the file.
	try
	{
		std::unique_ptr<std::istream> in = source ? source() : nullptr;
		if (!in || !*in)
			return fail("the recording cannot be opened");
		Reader reader(*in);
		Header header;
		if (!reader.ReadHeader(header))
			return fail(reader.Error());
		index.frequency = static_cast<double>(header.qpcFrequency);
		index.headerDevices = header.devices;
		summary.startUnixMicroseconds = header.startUnixMicroseconds;

		// The stretch the next sample opens, and the one that is open.
		uint64_t nextFirstOffset = reader.BytesRead();
		Stretch next;
		next.startUnixMicroseconds = header.startUnixMicroseconds;
		bool open = false;
		double total = 0.0, lastPoint = 0.0;
		bool cleanEnd = false;
		std::set<uint32_t> seen;
		uint64_t published = 0;
		// Each device as the file last described it, for the marks.
		std::vector<DeviceInfo> described = header.devices;

		const auto close = [&]
		{
			if (!open)
				return;
			open = false;
			total = index.spans.back().start + index.spans.back().seconds;
			summary.stretches.back().seconds = index.spans.back().seconds;
		};

		Record record;
		uint64_t offset = reader.BytesRead();
		// Whether a reading can start at the record: in a compact file only
		// where its blocks reach back to nothing.
		bool fresh = reader.Fresh();
		Reader::Next state = reader.Read(record);
		bool tooMany = false;
		const auto sample = [&](const Sample &one)
		{
			if (!open)
			{
				open = true;
				Index::Span span;
				span.firstOffset = nextFirstOffset;
				span.baseQpc = one.qpc;
				span.start = total;
				index.spans.push_back(span);
				next.start = total;
				summary.stretches.push_back(next);
				index.points.push_back({ total, nextFirstOffset });
				lastPoint = total;
			}
			Index::Span &span = index.spans.back();
			span.seconds = SecondsInto(one.qpc, span.baseQpc, index.frequency, span.seconds);
			const double at = span.start + span.seconds;
			if (fresh && at - lastPoint >= PointSpacingSeconds)
			{
				index.points.push_back({ at, offset });
				lastPoint = at;
			}
			++summary.samples;
			if (seen.size() < MaxDevices)
				seen.insert(one.deviceId);
		};
		for (; state == Reader::Next::Record && !tooMany;
			offset = reader.BytesRead(), fresh = reader.Fresh(), state = reader.Read(record))
		{
			if (index.cancel.load(std::memory_order_acquire))
				return;
			cleanEnd = false;
			// More stretches than any recorder writes: what was read so far plays,
			// and the rest is left out.
			const bool holdsSamples = record.kind == KindSample || (record.kind == KindSamples && !record.samples.empty());
			if (holdsSamples && !open && index.spans.size() >= MaxStretches)
			{
				tooMany = true;
				break;
			}
			switch (record.kind)
			{
			case KindSample:
				sample(record.sample);
				break;
			case KindSamples:
				for (const Sample &one : record.samples)
					sample(one);
				break;
			case KindGap:
				if (record.gap.lost > 0)
				{
					summary.lost += (std::min)(record.gap.lost, UINT64_MAX - summary.lost);
					const double at = open ? index.spans.back().start + index.spans.back().seconds : total;
					if (summary.losses.size() < MaxLosses)
						summary.losses.push_back({ at, record.gap.lost });
				}
				break;
			case KindDevice:
			{
				if (index.deviceRecords.size() < MaxKeptRecords)
					index.deviceRecords.emplace_back(offset, record.device);
				bool known = false;
				for (DeviceInfo &device : described)
					if (device.deviceId == record.device.deviceId)
					{
						device = record.device;
						known = true;
					}
				if (!known && described.size() < MaxDevices)
					described.push_back(record.device);
				break;
			}
			case KindMark:
			{
				if (summary.marks.size() >= MaxMarks)
					break;
				Mark mark;
				// On the open stretch's clock, and never outside what was read of it.
				mark.time = total;
				if (open)
				{
					const Index::Span &span = index.spans.back();
					const double into = (static_cast<double>(record.mark.qpc) - static_cast<double>(span.baseQpc)) /
						index.frequency;
					mark.time = span.start + (into > 0.0 ? (std::min)(into, span.seconds) : 0.0);
				}
				mark.unixMicroseconds = record.mark.unixMicroseconds;
				mark.name = record.mark.name;
				for (const DeviceInfo &device : described)
					if (record.mark.deviceId != NoDevice && device.deviceId == record.mark.deviceId)
					{
						mark.hasDevice = true;
						mark.device = device;
					}
				summary.marks.push_back(std::move(mark));
				break;
			}
			case KindAlignment:
				if (index.alignments.size() < MaxKeptRecords)
					index.alignments.emplace_back(offset, record.alignment);
				break;
			case KindModel:
				if (index.models.size() < MaxKeptRecords)
					index.models.emplace_back(offset, record.model);
				break;
			case KindPause:
			case KindEnd:
				close();
				cleanEnd = true;
				break;
			case KindResume:
				close();
				nextFirstOffset = offset;
				next = Stretch();
				next.startUnixMicroseconds = record.resume.unixMicroseconds;
				next.trigger = record.resume.trigger;
				break;
			default:
				break;
			}
			if (reader.BytesRead() - published >= ProgressStepBytes)
			{
				published = reader.BytesRead();
				summary.progress = bytes > 0 ? (std::min)(1.0, static_cast<double>(published) / static_cast<double>(bytes)) : 0.0;
				summary.seconds = open ? index.spans.back().start + index.spans.back().seconds : total;
				// The timeline's parts are handed over whole at the end.
				Summary partial;
				partial.state = State::Reading;
				partial.bytes = bytes;
				partial.progress = summary.progress;
				partial.seconds = summary.seconds;
				std::lock_guard<std::mutex> lock(index.mutex);
				index.summary = partial;
			}
		}
		close();
		summary.seconds = total;
		summary.devices = seen.size();
		summary.cutShort = tooMany || state != Reader::Next::EndOfFile || !cleanEnd;
		if (tooMany)
			summary.error = "the recording has more stretches than a playback keeps";
		else if (state == Reader::Next::Invalid)
			summary.error = reader.Error();
		summary.progress = 1.0;
		summary.state = State::Ready;
		index.ready.store(true, std::memory_order_release);
		publish();
	}
	catch (const std::exception &e)
	{
		fail(e.what());
	}
}

// Puts the cursor on the last kept position at or before `time`, knowing what
// the file said of devices and alignments up to there.
void Playback::Seek(const Index &index, double time)
{
	if (!cursor_)
		cursor_ = std::make_unique<Cursor>();
	Cursor &cursor = *cursor_;
	if (!cursor.stream)
	{
		cursor.stream = source_ ? source_() : nullptr;
		if (cursor.stream)
		{
			// The header first: it says whether the samples are in blocks, and
			// what the blocks' numbers are steps of.
			cursor.reader = std::make_unique<Reader>(*cursor.stream);
			Header header;
			if (!cursor.reader->ReadHeader(header))
				cursor.reader.reset();
		}
	}
	cursor.devices.clear();
	cursor.alignments.clear();
	cursor.hasPending = false;
	cursor.block.clear();
	cursor.blockAt = 0;
	cursor.span = 0;
	cursor.spanSeconds = 0.0;
	cursor.time = 0.0;
	if (!cursor.reader || index.points.empty())
		return;

	auto after = std::upper_bound(index.points.begin(), index.points.end(), time,
		[](double t, const Index::Point &point) { return t < point.time; });
	const Index::Point &point = after == index.points.begin() ? index.points.front() : *(after - 1);
	for (const DeviceInfo &device : index.headerDevices)
		cursor.Describe(device);
	for (const auto &entry : index.deviceRecords)
		if (entry.first < point.offset)
			cursor.Describe(entry.second);
	for (const auto &entry : index.models)
		if (entry.first < point.offset)
			cursor.Shape(entry.second);
	for (const auto &entry : index.alignments)
		if (entry.first < point.offset)
			cursor.Align(entry.second);
	while (cursor.span + 1 < index.spans.size() && point.offset >= index.spans[cursor.span + 1].firstOffset)
		++cursor.span;
	cursor.spanSeconds = (std::max)(0.0, point.time - index.spans[cursor.span].start);
	cursor.time = point.time;
	if (!cursor.reader->SeekToRecord(point.offset))
		cursor.reader.reset();
}

bool Playback::Frame(double time, std::vector<DevicePose> &devices)
{
	devices.clear();
	std::shared_ptr<Index> shared;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		shared = index_;
	}
	if (!shared || !shared->ready.load(std::memory_order_acquire))
		return false;
	const Index &index = *shared;
	if (index.spans.empty())
		return true;
	const double total = index.spans.back().start + index.spans.back().seconds;
	time = std::isfinite(time) ? (std::min)((std::max)(time, 0.0), total) : 0.0;

	// Backwards, or far ahead: start again a trail's length before the moment.
	if (!cursor_ || time < cursor_->time || time > cursor_->time + 5.0)
		Seek(index, (std::max)(0.0, time - TrailSeconds));
	Cursor &cursor = *cursor_;
	if (!cursor.reader)
		return true;

	for (int budget = FrameRecordBudget; budget > 0; --budget)
	{
		if (!cursor.hasPending)
		{
			if (cursor.blockAt < cursor.block.size())
			{
				// The next sample of the block in hand, as a record of its own.
				cursor.pending.kind = KindSample;
				cursor.pending.sample = cursor.block[cursor.blockAt++];
			}
			else
			{
				const uint64_t offset = cursor.reader->BytesRead();
				if (cursor.reader->Read(cursor.pending) != Reader::Next::Record)
					break;
				while (cursor.span + 1 < index.spans.size() && offset >= index.spans[cursor.span + 1].firstOffset)
				{
					++cursor.span;
					cursor.spanSeconds = 0.0;
					cursor.CutTrails();
				}
				if (cursor.pending.kind == KindSamples && !cursor.pending.samples.empty())
				{
					cursor.block.swap(cursor.pending.samples);
					cursor.blockAt = 0;
					continue;
				}
			}
			const Index::Span &span = index.spans[cursor.span];
			if (cursor.pending.kind == KindSample)
				cursor.spanSeconds = SecondsInto(cursor.pending.sample.qpc, span.baseQpc, index.frequency, cursor.spanSeconds);
			cursor.pendingTime = span.start + cursor.spanSeconds;
			cursor.hasPending = true;
		}
		if (cursor.pending.kind == KindSample && cursor.pendingTime > time)
			break;
		cursor.hasPending = false;
		switch (cursor.pending.kind)
		{
		case KindSample:
			cursor.Apply(cursor.pending.sample, cursor.pendingTime);
			cursor.time = cursor.pendingTime;
			break;
		case KindDevice:
			cursor.Describe(cursor.pending.device);
			break;
		case KindAlignment:
			cursor.Align(cursor.pending.alignment);
			break;
		case KindModel:
			cursor.Shape(cursor.pending.model);
			break;
		case KindGap:
		case KindBoundary:
		case KindPause:
		case KindResume:
		case KindEnd:
			cursor.CutTrails();
			break;
		default:
			break;
		}
	}

	const double trailStart = cursor.time - TrailSeconds;
	for (Cursor::Device &device : cursor.devices)
	{
		if (!device.any)
			continue;
		size_t old = 0;
		while (old < device.trail.size() && device.trail[old].first < trailStart)
			++old;
		device.trail.erase(device.trail.begin(), device.trail.begin() + static_cast<std::ptrdiff_t>(old));
		DevicePose pose;
		pose.info = device.info;
		pose.tracked = device.tracked;
		for (int i = 0; i < 3; ++i)
			pose.position[i] = device.position[i];
		std::copy(device.rotation, device.rotation + 4, pose.rotation);
		pose.renderModel = device.renderModel;
		pose.trail.reserve(device.trail.size());
		for (const auto &entry : device.trail)
			pose.trail.push_back(entry.second);
		devices.push_back(std::move(pose));
	}
	return true;
}

} // namespace questcal::recording
