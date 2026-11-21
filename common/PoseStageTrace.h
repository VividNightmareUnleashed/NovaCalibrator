#pragma once

#include <cstdint>

namespace questcal::stagetrace
{
// A private instance of the existing MPMC ring. No diagnostic row can reach
// a solver consumer. Only the recorder holds the request event between polls.
inline constexpr char Request[] = "Local\\NovaCalibrator.StageTrace.v1.Request";
inline constexpr char Ring[] = "Local\\NovaCalibrator.StageTrace.v1.layout5";
enum class Stage : uint32_t { Input, Submitted, Resent, InputHead, SubmittedHead, ResentHead };

// OpenVR has 64 slots. The counter is per device, wraps modulo 2^23,
// and advances per driver callback, including revisions at the same pose time.
inline uint32_t Tag(uint32_t device, Stage stage, uint64_t sequence)
{
	return (static_cast<uint32_t>(sequence) << 9) | (static_cast<uint32_t>(stage) << 6) | device;
}
inline uint32_t Device(uint32_t tag) { return tag & 63u; }
inline Stage Kind(uint32_t tag) { return static_cast<Stage>((tag >> 6) & 7u); }
inline uint32_t Sequence(uint32_t tag) { return tag >> 9; }
}
