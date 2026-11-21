#pragma once

#include <openvr.h>
#include "../common/Protocol.h"
#include "../common/PoseStageTrace.h"
#include <windows.h>
#include <compressapi.h>
#include <intrin.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#pragma comment(lib, "Cabinet.lib")

namespace questcal::stagetrace
{
// Every capture's file name starts and ends with these.
inline constexpr wchar_t FilePrefix[] = L"Tracking-stages-";
inline constexpr wchar_t FileSuffix[] = L".novatrace";
// A capture answers one question and nothing else in the overlay removes it:
// the size cap and "Delete recording" are the recording's. So the folder
// keeps this many, the newest, and a capture that starts removes the rest.
inline constexpr size_t KeptCaptures = 5;

// Removes all but the `keep` newest captures of a folder, by the time each
// was last written. Only files with a capture's name are looked at.
inline void RemoveOldCaptures(const std::wstring &directory, size_t keep)
{
	std::vector<std::pair<uint64_t, std::wstring>> found;
	WIN32_FIND_DATAW entry;
	const HANDLE search = FindFirstFileW((directory + L"\\" + FilePrefix + L"*" + FileSuffix).c_str(), &entry);
	if (search == INVALID_HANDLE_VALUE)
		return;
	do
	{
		if ((entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
			found.push_back({ (uint64_t{ entry.ftLastWriteTime.dwHighDateTime } << 32) | entry.ftLastWriteTime.dwLowDateTime,
				entry.cFileName });
	} while (FindNextFileW(search, &entry));
	FindClose(search);
	std::sort(found.begin(), found.end());
	for (size_t i = 0; i + keep < found.size(); ++i)
		DeleteFileW((directory + L"\\" + found[i].second).c_str());
}

// A single lossless stream. A submitted sample can use the exact input from
// its callback as the delta base. Runtime matrices keep the SDK's float bits.
class Writer
{
	using Words = std::array<uint64_t, 22>;
	FILE *file;
	COMPRESSOR_HANDLE compressor = nullptr;
	std::vector<uint8_t> bytes;
	std::array<uint8_t, 32768> compressed{};
	Words driver[6][64]{};
	Words runtime[64]{};
	Words standing{};
	std::array<uint32_t, 64> inputSequence;
	int64_t lastQpc;

	template<typename T> static uint64_t Bits(const T &value)
	{
		uint64_t word = 0;
		static_assert(sizeof(value) <= sizeof(word));
		std::memcpy(&word, &value, sizeof(value));
		return word;
	}
	template<typename T> void Fixed(T value)
	{
		const auto p = reinterpret_cast<const uint8_t *>(&value);
		bytes.insert(bytes.end(), p, p + sizeof(value));
	}
	void Number(uint64_t n)
	{
		while (n >= 128)
		{
			bytes.push_back(static_cast<uint8_t>(n | 128));
			n >>= 7;
		}
		bytes.push_back(static_cast<uint8_t>(n));
	}
	void Time(int64_t qpc)
	{
		const uint64_t delta = static_cast<uint64_t>(qpc) - static_cast<uint64_t>(lastQpc);
		Number((delta << 1) ^ (0ull - (delta >> 63)));
		lastQpc = qpc;
	}
	void Changes(const Words &words, const Words &base)
	{
		uint32_t mask = 0;
		for (size_t i = 0; i < words.size(); ++i)
			if (words[i] != base[i])
				mask |= 1u << i;
		Number(mask);
		while (mask)
		{
			unsigned long i;
			_BitScanForward(&i, mask);
			Number(words[i] ^ base[i]);
			mask &= mask - 1;
		}
	}
	void Block(size_t n)
	{
		SIZE_T size = 0;
		const bool useZip = compressor && ResetCompressor(compressor) &&
			Compress(compressor, bytes.data(), n, compressed.data(), n, &size) && size < n;
		const uint16_t lengths[] = { static_cast<uint16_t>(n), static_cast<uint16_t>(useZip ? size : n) };
		const auto stored = lengths[1];
		if (fwrite(lengths, sizeof(lengths), 1, file) != 1 ||
			fwrite(useZip ? compressed.data() : bytes.data(), 1, stored, file) != stored)
			good = false;
		written += sizeof(lengths) + stored;
		bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(n));
	}
	static Words Matrix(const vr::HmdMatrix34_t &m)
	{
		Words words{};
		size_t i = 0;
		for (const auto &row : m.m)
			for (float v : row)
				words[i++] = Bits(v);
		return words;
	}
public:
	bool good = true;
	uint64_t written = 0;
	Writer(FILE *out, int64_t hz, int64_t start, uint64_t unixUs) : file(out), lastQpc(start)
	{
		inputSequence.fill(UINT32_MAX);
		bytes.reserve(65536);
		CreateCompressor(COMPRESS_ALGORITHM_MSZIP | COMPRESS_RAW, nullptr, &compressor);
		const uint64_t header[] = { static_cast<uint64_t>(hz), static_cast<uint64_t>(start), unixUs };
		good = fwrite("NOVASTG1", 1, 8, file) == 8 && fwrite(header, sizeof(header), 1, file) == 1;
		written = 32;
	}
	~Writer() { if (compressor) CloseCompressor(compressor); }
	Writer(const Writer &) = delete;
	Writer &operator=(const Writer &) = delete;
	void Flush(bool final = false)
	{
		while (good && (bytes.size() >= compressed.size() || (final && !bytes.empty())))
			Block((std::min)(bytes.size(), compressed.size()));
	}
	void Driver(const protocol::DevicePoseSample &s)
	{
		const uint32_t k = static_cast<uint32_t>(Kind(s.deviceId)), id = Device(s.deviceId), seq = Sequence(s.deviceId);
		if (k >= 6)
			return;
		const bool paired = k == 1 && inputSequence[id] == seq;
		bytes.push_back(static_cast<uint8_t>(k + 1 + (paired ? 128 : 0)));
		bytes.push_back(static_cast<uint8_t>(id));
		Number(seq);
		Time(s.sampleTimeQpc);
		Words w{};
		w[0] = (static_cast<uint64_t>(s.trackingResult) << 2) | (s.deviceIsConnected ? 2 : 0) | (s.poseIsValid ? 1 : 0);
		w[1] = Bits(s.poseTimeOffset);
		size_t i = 2;
		const auto quat = [&](const vr::HmdQuaternion_t &q)
		{
			for (double v : { q.w, q.x, q.y, q.z }) w[i++] = Bits(v);
		};
		quat(s.worldFromDriverRotation);
		for (double v : s.worldFromDriverTranslation) w[i++] = Bits(v);
		quat(s.rotation);
		for (const auto p : { s.position, s.velocity, s.angularVelocity })
			for (int j = 0; j < 3; ++j) w[i++] = Bits(p[j]);
		Changes(w, driver[paired ? 0 : k][id]);
		driver[k][id] = w;
		if (k == 0) inputSequence[id] = seq;
		Flush();
	}
	void Runtime(int64_t begin, int64_t end, uint64_t mask, const vr::TrackedDevicePose_t *poses)
	{
		bytes.push_back(7);
		Time(begin);
		Number(static_cast<uint64_t>(end - begin));
		Number(mask);
		for (size_t id = 0; id < 64; ++id)
		{
			if (!(mask & (uint64_t{1} << id))) continue;
			const auto &p = poses[id];
			auto w = Matrix(p.mDeviceToAbsoluteTracking);
			for (int i = 0; i < 3; ++i)
			{
				w[12 + i] = Bits(p.vVelocity.v[i]);
				w[15 + i] = Bits(p.vAngularVelocity.v[i]);
			}
			w[18] = (static_cast<uint64_t>(p.eTrackingResult) << 2) | (p.bDeviceIsConnected ? 2 : 0) | (p.bPoseIsValid ? 1 : 0);
			Changes(w, runtime[id]);
			runtime[id] = w;
		}
		Flush();
	}
	void Gap(int64_t qpc, uint64_t count)
	{
		bytes.push_back(8); Time(qpc); Number(count);
		inputSequence.fill(UINT32_MAX);
	}
	void Describe(uint32_t id, uint32_t deviceClass, int32_t role, const std::string &controllerType)
	{
		bytes.push_back(9); bytes.push_back(static_cast<uint8_t>(id)); Number(deviceClass);
		Fixed(role);
		const auto size = (std::min)(controllerType.size(), size_t{255});
		bytes.push_back(static_cast<uint8_t>(size));
		bytes.insert(bytes.end(), controllerType.data(), controllerType.data() + size);
	}
	void Origin(int64_t qpc, const vr::HmdMatrix34_t &matrix, bool first)
	{
		const auto w = Matrix(matrix);
		if (first || w != standing)
		{
			bytes.push_back(10); Time(qpc); Changes(w, standing);
			standing = w;
		}
	}
	void End(int64_t qpc, uint8_t reason)
	{
		bytes.push_back(11); Time(qpc); bytes.push_back(reason); Flush(true);
	}
};
}
