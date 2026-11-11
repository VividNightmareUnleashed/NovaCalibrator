// Writes a pose recording again in the other mode, and says how far two
// recordings are apart (docs/recording-format.md).
//
//   cl /std:c++17 /EHsc /W4 /O2 /I ..\Overlay novapose-convert.cpp
//
//   novapose-convert IN OUT
//       IN as a compact recording (format version 2).
//   novapose-convert IN OUT --velocity-scale 100000 --angular-scale 100000
//       The same with finer velocities: steps in one meter (radian) a second.
//       The default is 10000, a step of 100 micrometers a second.
//   novapose-convert IN OUT --raw
//       IN as a raw recording (format version 1): every sample whole, with
//       the values a reader of IN gets.
//   novapose-convert --compare A B
//       What differs between the samples of A and of B, as the largest
//       distance for each kind of number. Exit code 1 when the two do not
//       hold the same records and samples of the same devices at the same
//       counts.
//
// Like tools/novapose-listing.cpp it needs the format header and nothing else
// of Nova Calibrator. tools/check-recording-readers.ps1 compiles it and runs
// it on the reference recordings.
#include "PoseRecordingFormat.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace questcal::recording;

namespace
{

int Usage()
{
	std::fprintf(stderr,
		"Usage: novapose-convert IN OUT [--raw] [--velocity-scale N] [--angular-scale N]\n"
		"       novapose-convert --compare A B\n");
	return 2;
}

// A file read through a large buffer: a recording can be gigabytes.
struct Input
{
	std::vector<char> buffer = std::vector<char>(1u << 20);
	std::ifstream stream;

	explicit Input(const char *path)
	{
		stream.rdbuf()->pubsetbuf(buffer.data(), static_cast<std::streamsize>(buffer.size()));
		stream.open(path, std::ios::in | std::ios::binary);
	}
};

const char *HowItEnded(Reader::Next next)
{
	return next == Reader::Next::EndOfFile ? "complete" : next == Reader::Next::Truncated ? "cut short inside a record"
		: "not a recording from some record on";
}

int Convert(const char *from, const char *to, bool raw, uint32_t velocityScale, uint32_t angularScale)
{
	Input in(from);
	if (!in.stream.is_open())
	{
		std::fprintf(stderr, "%s could not be opened.\n", from);
		return 1;
	}
	Reader reader(in.stream);
	Header header;
	if (!reader.ReadHeader(header))
	{
		std::fprintf(stderr, "%s is not a recording this program can use: %s.\n", from, reader.Error().c_str());
		return 1;
	}
	const bool wasCompact = header.version == FormatVersionCompact;
	header.version = raw ? FormatVersion : FormatVersionCompact;
	header.mode = raw ? ModeRaw : ModeCompact;
	header.velocityScale = raw ? 0 : velocityScale;
	header.angularScale = raw ? 0 : angularScale;
	const compact::Parameters parameters{ header.qpcFrequency, velocityScale, angularScale };
	if (!raw && !compact::Valid(parameters))
	{
		std::fprintf(stderr, "A compact recording has a clock rate of %lld to %lld counts a second and velocity scales "
			"of %u to %u; these are %lld, %u and %u.\n", static_cast<long long>(compact::MinClockRate),
			static_cast<long long>(compact::MaxClockRate), compact::MinVelocityScale, compact::MaxVelocityScale,
			static_cast<long long>(header.qpcFrequency), velocityScale, angularScale);
		return 1;
	}
	std::ofstream out(to, std::ios::out | std::ios::binary | std::ios::trunc);
	if (!out.is_open())
	{
		std::fprintf(stderr, "%s could not be written.\n", to);
		return 1;
	}

	std::string bytes;
	AppendHeader(bytes, header);
	CompactWriter writer(parameters);
	unsigned long long samples = 0, written = 0, dropped = 0;
	const auto sample = [&](const Sample &one)
	{
		if (raw)
			AppendSample(bytes, one);
		else
			writer.Append(bytes, one);
		++samples;
	};
	Record record;
	Reader::Next next = reader.Read(record);
	for (; next == Reader::Next::Record; next = reader.Read(record))
	{
		if (record.kind == KindSample)
			sample(record.sample);
		else if (record.kind == KindSamples && !record.samples.empty())
			for (const Sample &one : record.samples)
				sample(one);
		else if (record.kind == KindSync && wasCompact)
			;   // the writer puts its own
		else if (record.kind == KindSamples || record.kind == KindSync)
			// A raw file's record of a kind that means something else in a
			// compact file cannot be carried over.
			++dropped;
		else
		{
			writer.Flush(bytes);
			AppendRecord(bytes, record);
			if (record.kind == KindResume)
				writer.Restart();
		}
		if (bytes.size() >= (1u << 20))
		{
			out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
			written += bytes.size();
			bytes.clear();
		}
	}
	writer.Flush(bytes);
	out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
	written += bytes.size();
	out.close();
	if (!out)
	{
		std::fprintf(stderr, "%s could not be written.\n", to);
		return 1;
	}
	std::printf("%s: %llu samples, %llu bytes read (%s); %s: %llu bytes, %s", from, samples,
		static_cast<unsigned long long>(reader.BytesRead()), HowItEnded(next), to, written, raw ? "raw" : "compact");
	if (samples > 0)
		std::printf(", %.3f bytes a sample", static_cast<double>(written) / static_cast<double>(samples));
	std::printf("\n");
	if (dropped > 0)
		std::fprintf(stderr, "%llu record(s) of kind 11 or 12 were left out.\n", dropped);
	if (next == Reader::Next::Invalid)
		std::fprintf(stderr, "%s stops being a recording part way: %s.\n", from, reader.Error().c_str());
	return next == Reader::Next::Invalid ? 1 : 0;
}

// One recording's samples and other records, one at a time.
struct Walk
{
	Input in;
	Reader reader;
	Header header;
	Record record;
	size_t inBlock = 0;
	Reader::Next next = Reader::Next::Record;
	bool opened = false;

	explicit Walk(const char *path) : in(path), reader(in.stream)
	{
		opened = in.stream.is_open() && reader.ReadHeader(header);
	}

	// The next sample (true, in `sample`) or the next record that is neither a
	// sample nor a sync (false, in `record`; `ended` at the end of the file).
	bool Next(Sample &sample, bool &ended)
	{
		ended = false;
		for (;;)
		{
			if (inBlock < record.samples.size())
			{
				sample = record.samples[inBlock++];
				return true;
			}
			next = reader.Read(record);
			inBlock = 0;
			if (next != Reader::Next::Record)
			{
				record.samples.clear();
				ended = true;
				return false;
			}
			if (record.kind == KindSample)
			{
				sample = record.sample;
				return true;
			}
			if (record.kind == KindSamples && !record.samples.empty())
				continue;
			if (record.kind == KindSync && header.version == FormatVersionCompact)
				continue;
			return false;
		}
	}
};

double Length(const double q[4])
{
	return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

// The angle between two rotations, in radians; 0 when one has no direction.
double Angle(const double a[4], const double b[4])
{
	const double la = Length(a), lb = Length(b);
	if (!(la > 0.0) || !(lb > 0.0))
		return 0.0;
	// The vector part of conj(a) * b, and its scalar part.
	const double w = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
	const double x = a[0] * b[1] - a[1] * b[0] - a[2] * b[3] + a[3] * b[2];
	const double y = a[0] * b[2] + a[1] * b[3] - a[2] * b[0] - a[3] * b[1];
	const double z = a[0] * b[3] - a[1] * b[2] + a[2] * b[1] - a[3] * b[0];
	return 2.0 * std::atan2(std::sqrt(x * x + y * y + z * z), std::fabs(w));
}

bool Single(const Sample &sample)
{
	double pose[compact::PoseNumbers];
	compact::PoseOf(sample, pose);
	uint32_t bits = 0;
	for (double value : pose)
		if (!compact::SingleBits(value, bits))
			return false;
	return true;
}

int Compare(const char *first, const char *second)
{
	Walk a(first), b(second);
	for (const Walk *walk : { &a, &b })
		if (!walk->opened)
		{
			std::fprintf(stderr, "%s is not a recording this program can use.\n", walk == &a ? first : second);
			return 1;
		}
	unsigned long long samples = 0, exact = 0, single = 0, singleExact = 0, records = 0;
	double offset = 0.0, position = 0.0, rotationNumber = 0.0, rotationAngle = 0.0, velocity = 0.0, angular = 0.0;
	std::string differs;
	if (a.header.qpcFrequency != b.header.qpcFrequency || a.header.startQpc != b.header.startQpc ||
		a.header.startUnixMicroseconds != b.header.startUnixMicroseconds || a.header.writer != b.header.writer ||
		a.header.devices != b.header.devices)
		differs = "the headers name another clock, start, writer or devices";
	while (differs.empty())
	{
		Sample one, two;
		bool endedA = false, endedB = false;
		const bool isSampleA = a.Next(one, endedA), isSampleB = b.Next(two, endedB);
		if (endedA || endedB)
		{
			if (endedA != endedB || a.next != b.next)
				differs = "one file ends before the other, or in another way";
			break;
		}
		if (isSampleA != isSampleB)
		{
			differs = "a sample in one file is another record in the other, after sample " + std::to_string(samples);
			break;
		}
		if (!isSampleA)
		{
			std::string bytesA, bytesB;
			AppendRecord(bytesA, a.record);
			AppendRecord(bytesB, b.record);
			if (bytesA != bytesB)
				differs = "a record of kind " + std::to_string(a.record.kind) + " differs, after sample " +
					std::to_string(samples);
			++records;
			continue;
		}
		if (one.qpc != two.qpc || one.deviceId != two.deviceId || one.trackingResult != two.trackingResult ||
			one.poseValid != two.poseValid || one.connected != two.connected ||
			std::memcmp(one.worldFromDriverRotation, two.worldFromDriverRotation, sizeof one.worldFromDriverRotation) != 0 ||
			std::memcmp(one.worldFromDriverTranslation, two.worldFromDriverTranslation,
				sizeof one.worldFromDriverTranslation) != 0)
		{
			differs = "sample " + std::to_string(samples) + " has another count, device, state or world-from-driver transform";
			break;
		}
		std::string wholeA, wholeB;
		AppendSample(wholeA, one);
		AppendSample(wholeB, two);
		const bool same = wholeA == wholeB;
		exact += same ? 1 : 0;
		if (Single(one))
		{
			++single;
			singleExact += same ? 1 : 0;
		}
		const auto furthest = [](double &most, double x, double y)
		{
			const double apart = std::fabs(x - y);
			// Not a number on one side only is as far apart as can be.
			if (apart > most || (apart != apart && std::memcmp(&x, &y, sizeof x) != 0))
				most = apart != apart ? HUGE_VAL : apart;
		};
		furthest(offset, one.poseTimeOffset, two.poseTimeOffset);
		for (int i = 0; i < 4; ++i)
			furthest(rotationNumber, one.rotation[i], two.rotation[i]);
		for (int i = 0; i < 3; ++i)
		{
			furthest(position, one.position[i], two.position[i]);
			furthest(velocity, one.velocity[i], two.velocity[i]);
			furthest(angular, one.angularVelocity[i], two.angularVelocity[i]);
		}
		const double angle = Angle(one.rotation, two.rotation);
		if (angle > rotationAngle)
			rotationAngle = angle;
		++samples;
	}
	std::printf("samples compared: %llu, other records: %llu\n", samples, records);
	std::printf("samples that are the same bit for bit: %llu\n", exact);
	std::printf("samples of single-precision numbers: %llu, the same bit for bit: %llu\n", single, singleExact);
	std::printf("largest distance, pose time offset: %.3e s\n", offset);
	std::printf("largest distance, position: %.3e m\n", position);
	std::printf("largest distance, a rotation number: %.3e\n", rotationNumber);
	std::printf("largest distance, rotation as an angle: %.3e rad\n", rotationAngle);
	std::printf("largest distance, velocity: %.3e m/s\n", velocity);
	std::printf("largest distance, angular velocity: %.3e rad/s\n", angular);
	if (!differs.empty())
	{
		std::printf("not the same recording: %s\n", differs.c_str());
		return 1;
	}
	std::printf("the same records and samples, in the same order\n");
	return 0;
}

} // namespace

int main(int argc, char **argv)
{
	if (argc == 4 && std::strcmp(argv[1], "--compare") == 0)
		return Compare(argv[2], argv[3]);
	if (argc < 3)
		return Usage();
	bool raw = false;
	uint32_t velocityScale = 10000, angularScale = 10000;
	for (int i = 3; i < argc; ++i)
	{
		const bool velocity = std::strcmp(argv[i], "--velocity-scale") == 0;
		if (std::strcmp(argv[i], "--raw") == 0)
			raw = true;
		else if ((velocity || std::strcmp(argv[i], "--angular-scale") == 0) && i + 1 < argc)
			(velocity ? velocityScale : angularScale) = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
		else
			return Usage();
	}
	return Convert(argv[1], argv[2], raw, velocityScale, angularScale);
}
