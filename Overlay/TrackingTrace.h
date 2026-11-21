#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace vr { class IVRSystem; }
namespace questcal::stagetrace
{
enum class Result : unsigned char { Complete, Stopped, DriverMissing, DriverEnded, SizeLimit, Failed, Working, Idle };

class Capture
{
public:
	~Capture() { Close(); }
	void Start(vr::IVRSystem *system, const std::wstring &directory);
	void Stop();
	void Close();
	Result Status() const { return result.load(std::memory_order_acquire); }
private:
	std::thread worker;
	std::atomic<Result> result{ Result::Idle };
	std::atomic<int64_t> stopAt{ 0 };
	Result Run(vr::IVRSystem &system, const std::wstring &directory);
};
}
