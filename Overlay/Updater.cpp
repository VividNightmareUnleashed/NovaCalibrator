#include "stdafx.h"
#include "Updater.h"
#include "UpdateSignature.h"
#include "UpdateSigningKey.h"
#include "../common/Version.h"

#include <bcrypt.h>
#include <winhttp.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "winhttp.lib")

namespace questcal
{
namespace update
{
namespace
{

class InternetHandle
{
public:
	InternetHandle() = default;
	explicit InternetHandle(HINTERNET value) : value(value) {}
	~InternetHandle() { if (value) WinHttpCloseHandle(value); }
	InternetHandle(const InternetHandle &) = delete;
	InternetHandle &operator=(const InternetHandle &) = delete;
	InternetHandle(InternetHandle &&other) noexcept : value(other.value)
	{
		other.value = nullptr;
	}
	InternetHandle &operator=(InternetHandle &&other) noexcept
	{
		if (this != &other)
		{
			if (value) WinHttpCloseHandle(value);
			value = other.value;
			other.value = nullptr;
		}
		return *this;
	}
	operator HINTERNET() const { return value; }
	explicit operator bool() const { return value != nullptr; }

private:
	HINTERNET value = nullptr;
};

struct HttpRequest
{
	InternetHandle connection;
	InternetHandle request;
	operator HINTERNET() const { return request; }
};

std::runtime_error NetworkError(const char *operation)
{
	return std::runtime_error(std::string(operation) + " failed (Windows error " +
		std::to_string(GetLastError()) + ").");
}

// Every input is ASCII built from a validated release tag or digest.
std::wstring Utf8ToWide(const std::string &text)
{
	if (text.empty()) return std::wstring();
	const int chars = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
		text.data(), static_cast<int>(text.size()), nullptr, 0);
	std::wstring wide(static_cast<size_t>(chars), L'\0');
	MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
		static_cast<int>(text.size()), &wide[0], chars);
	return wide;
}

std::filesystem::path LocalUpdateRoot()
{
	DWORD chars = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);
	if (chars <= 1)
		throw std::runtime_error("Windows did not provide a local app-data folder.");
	std::wstring value(static_cast<size_t>(chars), L'\0');
	DWORD written = GetEnvironmentVariableW(L"LOCALAPPDATA", &value[0], chars);
	if (written == 0 || written >= chars)
		throw std::runtime_error("Windows did not provide a local app-data folder.");
	value.resize(written);
	return std::filesystem::path(value) / L"NovaCalibrator" / L"updates";
}

InternetHandle OpenSession()
{
	InternetHandle session(WinHttpOpen(L"NovaCalibrator/" QUESTCAL_VERSION_STRING,
		WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
		WINHTTP_NO_PROXY_BYPASS, 0));
	if (!session) throw NetworkError("Opening the update connection");
	if (!WinHttpSetTimeouts(session, 5000, 5000, 5000, 10000))
		throw NetworkError("Setting update timeouts");
	return session;
}

HttpRequest OpenGet(HINTERNET session, const wchar_t *host,
	INTERNET_PORT port, const wchar_t *path)
{
	HttpRequest result;
	result.connection = InternetHandle(WinHttpConnect(session, host, port, 0));
	if (!result.connection) throw NetworkError("Connecting to GitHub");
	result.request = InternetHandle(WinHttpOpenRequest(result.connection, L"GET", path, nullptr,
		WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE));
	if (!result.request) throw NetworkError("Creating the GitHub request");
	DWORD redirectPolicy = WINHTTP_OPTION_REDIRECT_POLICY_DISALLOW_HTTPS_TO_HTTP;
	if (!WinHttpSetOption(result.request, WINHTTP_OPTION_REDIRECT_POLICY,
		&redirectPolicy, sizeof redirectPolicy))
		throw NetworkError("Securing GitHub redirects");
	return result;
}

void SendGet(HINTERNET request, const wchar_t *headers = WINHTTP_NO_ADDITIONAL_HEADERS)
{
	const DWORD headerLength = headers == WINHTTP_NO_ADDITIONAL_HEADERS ? 0 : -1L;
	if (!WinHttpSendRequest(request, headers, headerLength,
		WINHTTP_NO_REQUEST_DATA, 0, 0, 0) || !WinHttpReceiveResponse(request, nullptr))
		throw NetworkError("Requesting the update");

	DWORD status = 0, bytes = sizeof status;
	if (!WinHttpQueryHeaders(request,
		WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
		WINHTTP_HEADER_NAME_BY_INDEX, &status, &bytes, WINHTTP_NO_HEADER_INDEX))
		throw NetworkError("Reading the update response");
	if (status != 200)
		throw std::runtime_error("GitHub returned HTTP " + std::to_string(status) + ".");
}

// `tooLarge` is the error once the body passes `maxBytes`.
std::string ReadResponse(HINTERNET request, size_t maxBytes,
	const std::function<bool()> &cancelled, const char *tooLarge)
{
	std::string body;
	std::array<char, 16384> buffer;
	for (;;)
	{
		if (cancelled()) throw std::runtime_error("Update check cancelled.");
		DWORD read = 0;
		if (!WinHttpReadData(request, buffer.data(),
			static_cast<DWORD>(buffer.size()), &read))
			throw NetworkError("Reading the update response");
		if (read == 0) break;
		if (body.size() + read > maxBytes)
			throw std::runtime_error(tooLarge);
		body.append(buffer.data(), read);
	}
	return body;
}

std::string FetchReleaseFeed(HINTERNET session,
	const std::function<bool()> &cancelled)
{
	HttpRequest request = OpenGet(session, L"api.github.com",
		INTERNET_DEFAULT_HTTPS_PORT,
		L"/repos/VividNightmareUnleashed/NovaCalibrator/releases?per_page=20");
	SendGet(request,
		L"Accept: application/vnd.github+json\r\n"
		L"X-GitHub-Api-Version: 2022-11-28\r\n");
	return ReadResponse(request, 1024 * 1024, cancelled,
		"GitHub returned an unexpectedly large release list.");
}

struct DownloadTarget
{
	std::wstring host;
	std::wstring path;
	INTERNET_PORT port = INTERNET_DEFAULT_HTTPS_PORT;
};

DownloadTarget ParseDownloadTarget(const std::string &url)
{
	std::wstring wide = Utf8ToWide(url);
	URL_COMPONENTS parts{};
	parts.dwStructSize = sizeof parts;
	parts.dwHostNameLength = static_cast<DWORD>(-1);
	parts.dwUrlPathLength = static_cast<DWORD>(-1);
	parts.dwExtraInfoLength = static_cast<DWORD>(-1);
	// SelectReleaseCandidate pinned the URL to https://github.com/.
	if (!WinHttpCrackUrl(wide.c_str(), static_cast<DWORD>(wide.size()), 0, &parts))
		throw std::runtime_error("The release package URL is invalid.");

	DownloadTarget target;
	target.host.assign(parts.lpszHostName, parts.dwHostNameLength);
	target.path.assign(parts.lpszUrlPath, parts.dwUrlPathLength);
	if (parts.dwExtraInfoLength)
		target.path.append(parts.lpszExtraInfo, parts.dwExtraInfoLength);
	target.port = parts.nPort;
	return target;
}

// SHA-256 through Windows CNG, fed in pieces. Any failure, from opening the
// provider on, makes Final false.
class Sha256
{
public:
	Sha256()
	{
		if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM,
			nullptr, 0) < 0)
		{
			algorithm = nullptr;
			return;
		}
		DWORD objectBytes = 0, resultBytes = 0;
		if (BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH,
			reinterpret_cast<PUCHAR>(&objectBytes), sizeof objectBytes,
			&resultBytes, 0) < 0 || objectBytes == 0)
			return;
		object.resize(objectBytes);
		if (BCryptCreateHash(algorithm, &hash, object.data(), objectBytes,
			nullptr, 0, 0) < 0)
			hash = nullptr;
	}
	~Sha256()
	{
		if (hash) BCryptDestroyHash(hash);
		if (algorithm) BCryptCloseAlgorithmProvider(algorithm, 0);
	}
	Sha256(const Sha256 &) = delete;
	Sha256 &operator=(const Sha256 &) = delete;

	void Update(const void *data, ULONG size)
	{
		healthy = healthy && hash && BCryptHashData(hash,
			static_cast<PUCHAR>(const_cast<void *>(data)), size, 0) >= 0;
	}
	bool Final(std::array<unsigned char, 32> &digest)
	{
		return healthy && hash && BCryptFinishHash(hash, digest.data(),
			static_cast<ULONG>(digest.size()), 0) >= 0;
	}

private:
	BCRYPT_ALG_HANDLE algorithm = nullptr;
	BCRYPT_HASH_HANDLE hash = nullptr;
	std::vector<unsigned char> object;
	bool healthy = true;
};

bool HashFile(const std::filesystem::path &path,
	std::array<unsigned char, 32> &digest)
{
	std::ifstream input(path, std::ios::binary);
	if (!input) return false;
	Sha256 hash;
	std::array<char, 65536> buffer;
	while (input)
	{
		input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
		const std::streamsize count = input.gcount();
		if (count > 0)
			hash.Update(buffer.data(), static_cast<ULONG>(count));
	}
	return input.eof() && hash.Final(digest);
}

class FileHandle
{
public:
	explicit FileHandle(HANDLE value) : value(value) {}
	~FileHandle() { if (value != INVALID_HANDLE_VALUE) CloseHandle(value); }
	FileHandle(const FileHandle &) = delete;
	FileHandle &operator=(const FileHandle &) = delete;
	operator HANDLE() const { return value; }

private:
	HANDLE value;
};

// Reads the package once, through a handle that lets nothing write or replace
// it meanwhile, and checks its size, its SHA-256 against the release's digest
// and its signature against `key`. Empty when it passes, otherwise why not.
// The SHA-256 the update helper checks again is then the digest of signed
// bytes.
std::string VerifyPackage(const std::filesystem::path &path,
	const ReleaseCandidate &release, const SigningPublicKey &key)
{
	PackageSignature signature;
	if (!ParsePackageSignature(release.signature, signature))
		return "The release signature is malformed.";
	FileHandle file(CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
		OPEN_EXISTING, FILE_FLAG_SEQUENTIAL_SCAN, nullptr));
	if (file == INVALID_HANDLE_VALUE)
		return "The update package could not be opened.";
	LARGE_INTEGER size{};
	if (!GetFileSizeEx(file, &size) || static_cast<uint64_t>(size.QuadPart) != release.size)
		return "The update package is not the size GitHub reported.";

	Sha256 sha256;
	Blake2b512 blake2b;
	std::vector<char> buffer(65536);
	uint64_t total = 0;
	for (;;)
	{
		DWORD read = 0;
		if (!ReadFile(file, buffer.data(), static_cast<DWORD>(buffer.size()), &read, nullptr))
			return "The update package could not be read.";
		if (read == 0) break;
		total += read;
		sha256.Update(buffer.data(), read);
		blake2b.Update(buffer.data(), read);
	}
	std::array<unsigned char, 32> digest{};
	if (total != release.size || !sha256.Final(digest))
		return "The update package could not be read.";
	if (digest != release.digestBytes)
		return "The downloaded package failed SHA-256 verification.";
	if (!VerifyPackageSignature(key, signature, blake2b.Final(), release.packageName))
		return "The downloaded package's signature does not verify against the release key.";
	return std::string();
}

void DownloadPackage(HINTERNET session, const ReleaseCandidate &release,
	const std::filesystem::path &partPath,
	const std::function<bool()> &cancelled,
	const std::function<void(uint64_t)> &progress)
{
	const DownloadTarget target = ParseDownloadTarget(release.downloadUrl);
	HttpRequest request = OpenGet(session, target.host.c_str(),
		target.port, target.path.c_str());
	SendGet(request);

	std::ofstream output(partPath, std::ios::binary | std::ios::trunc);
	if (!output)
		throw std::runtime_error("The update package could not be saved.");
	std::array<char, 65536> buffer;
	uint64_t total = 0;
	for (;;)
	{
		if (cancelled()) throw std::runtime_error("Update download cancelled.");
		DWORD read = 0;
		if (!WinHttpReadData(request, buffer.data(),
			static_cast<DWORD>(buffer.size()), &read))
			throw NetworkError("Downloading the update package");
		if (read == 0) break;
		if (total + read > release.size)
			throw std::runtime_error("The update package is larger than GitHub reported.");
		output.write(buffer.data(), read);
		if (!output)
			throw std::runtime_error("The update package could not be saved.");
		total += read;
		progress(total);
	}
	output.close();
	if (total != release.size)
		throw std::runtime_error("The update package download was incomplete.");
}

std::string DownloadSignature(HINTERNET session, const ReleaseCandidate &release,
	const std::function<bool()> &cancelled)
{
	const DownloadTarget target = ParseDownloadTarget(release.signatureUrl);
	HttpRequest request = OpenGet(session, target.host.c_str(),
		target.port, target.path.c_str());
	SendGet(request);
	std::string signature = ReadResponse(request, static_cast<size_t>(release.signatureSize),
		cancelled, "The release signature is larger than GitHub reported.");
	if (signature.size() != release.signatureSize)
		throw std::runtime_error("The release signature download was incomplete.");
	return signature;
}

// The key every update must be signed with, or an error for a build that has
// none.
SigningPublicKey ReleaseKey()
{
	SigningPublicKey key;
	if (!ParseSigningPublicKey(ReleaseSigningPublicKey, key))
		throw std::runtime_error("This build has no release signing key, so it cannot verify updates.");
	return key;
}

const char *UpdateHelperScript()
{
	return R"PS1(param(
    [Parameter(Mandatory=$true)][int]$ParentPid,
    [Parameter(Mandatory=$true)][string]$ZipPath,
    [Parameter(Mandatory=$true)][string]$ExpectedSha256
)
$ErrorActionPreference = 'Stop'
function Expand-VerifiedUpdate([string]$Path, [string]$ExpectedHash, [string]$Destination) {
    if (-not [IO.Path]::IsPathRooted($Path) -or -not [IO.Path]::IsPathRooted($Destination)) { throw 'Update paths must be absolute.' }
    foreach ($checkedPath in @($Path, $Destination)) {
        $ancestor = [IO.Path]::GetFullPath($checkedPath)
        while ($ancestor) {
            if (Test-Path -LiteralPath $ancestor) {
                if ((Get-Item -LiteralPath $ancestor -Force -ErrorAction Stop).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Update paths must not traverse reparse points.' }
            }
            $ancestor = [IO.Path]::GetDirectoryName($ancestor)
        }
    }
    # A read-only handle pins the bytes through hashing and extraction. On
    # Windows FileShare.Read excludes concurrent writes and replacement.
    $package = [IO.FileStream]::new($Path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read)
    try {
        if ($package.Length -gt 64MB) { throw 'The update archive exceeds its size budget.' }
        $actual = (Get-FileHash -InputStream $package -Algorithm SHA256).Hash
        if ($actual -ine $ExpectedHash) { throw 'The downloaded package no longer matches its verified SHA-256 digest.' }
        $package.Position = 0
        # ZipArchive's own assembly: Windows PowerShell, which runs this, does
        # not load it with System.IO.Compression.FileSystem.
        Add-Type -AssemblyName System.IO.Compression
        $archive = [IO.Compression.ZipArchive]::new($package, [IO.Compression.ZipArchiveMode]::Read, $true)
        try {
            if ($archive.Entries.Count -gt 4096) { throw 'The update archive contains too many entries.' }
            $root = [IO.Path]::GetFullPath($Destination + [IO.Path]::DirectorySeparatorChar)
            $names = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
            $total = 0L
            foreach ($entry in $archive.Entries) {
                $name = $entry.FullName.Replace('\','/')
                $parts = $name.TrimEnd('/').Split('/')
                if ($name.StartsWith('/') -or $name -match '[:\x00-\x1f<>"|?*]' -or
                    (($entry.ExternalAttributes -shr 16) -band 0xf000) -eq 0xa000 -or
                    ($parts | Where-Object { -not $_ -or $_ -eq '.' -or $_ -eq '..' -or $_ -match '[. ]$|^(?i:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\.|$)' })) {
                    throw 'The update package contains an unsafe path.'
                }
                $target = [IO.Path]::GetFullPath((Join-Path $Destination $name))
                if (-not $target.StartsWith($root, [StringComparison]::OrdinalIgnoreCase) -or -not $names.Add($target)) {
                    throw 'The update package contains an escaping or duplicate path.'
                }
                if ($entry.Length -gt 64MB) { throw 'An update archive entry exceeds its size budget.' }
                $total += $entry.Length
                if ($total -gt 256MB) { throw 'The expanded update exceeds its size budget.' }
            }
            # A fresh unpredictable directory prevents reuse of old contents.
            # The staging tree must remain exclusively owned until launch.
            New-Item -ItemType Directory -Path $Destination -ErrorAction Stop | Out-Null
            foreach ($entry in $archive.Entries) {
                $name = $entry.FullName.Replace('\','/')
                $target = Join-Path $Destination $name
                if ($name.EndsWith('/')) { New-Item -ItemType Directory -Path $target -Force | Out-Null; continue }
                New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($target)) -Force | Out-Null
                $entryStream = $entry.Open()
                try {
                    $output = [IO.FileStream]::new($target, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
                    try {
                        $buffer = [byte[]]::new(65536)
                        $written = 0L
                        while (($read = $entryStream.Read($buffer, 0, $buffer.Length)) -gt 0) {
                            $written += $read
                            if ($written -gt $entry.Length -or $written -gt 64MB) { throw 'An expanded entry exceeds its declared size.' }
                            $output.Write($buffer, 0, $read)
                        }
                        if ($written -ne $entry.Length) { throw 'An expanded entry is incomplete.' }
                    } finally { $output.Dispose() }
                } finally { $entryStream.Dispose() }
            }
        } finally { $archive.Dispose() }
    } finally { $package.Dispose() }
}
try {
    Wait-Process -Id $ParentPid -ErrorAction SilentlyContinue
    $updateDir = Split-Path -Parent $ZipPath
    $extractDir = Join-Path $updateDir ('package-' + [Guid]::NewGuid().ToString('N'))
    Expand-VerifiedUpdate $ZipPath $ExpectedSha256 $extractDir

    $installers = @(Get-ChildItem -LiteralPath $extractDir -Filter Install.ps1 -File -Recurse)
    if ($installers.Count -ne 1) { throw 'The update package does not contain exactly one installer.' }
    $packageDir = $installers[0].Directory.FullName
    foreach ($required in @('app\NovaCalibrator.exe', 'driver\01novacalibrator\bin\win64\driver_01novacalibrator.dll', 'Uninstall.ps1')) {
        if (-not (Test-Path -LiteralPath (Join-Path $packageDir $required))) {
            throw "The update package is incomplete: $required is missing."
        }
    }

    Write-Host 'Nova Calibrator is ready to update.' -ForegroundColor Cyan
    Write-Host 'Close Steam completely, including its system-tray icon. Installation starts when Steam has stopped.'
    while (Get-Process -Name 'steam','vrserver','vrmonitor','vrcompositor' -ErrorAction SilentlyContinue) {
        Start-Sleep -Seconds 2
    }

    $arguments = '-NoProfile -ExecutionPolicy Bypass -File "' + $installers[0].FullName + '"'
    $installer = Start-Process -FilePath 'powershell.exe' -Verb RunAs -ArgumentList $arguments -WorkingDirectory $packageDir -Wait -PassThru
    if ($installer.ExitCode -ne 0) { throw "The installer exited with code $($installer.ExitCode)." }
    exit 0
} catch {
    Write-Host ''
    Write-Host "Update failed: $_" -ForegroundColor Red
    Read-Host 'Press Enter to close'
    exit 1
}
)PS1";
}

std::wstring SystemPowerShell()
{
	std::vector<wchar_t> path(MAX_PATH);
	UINT written = GetSystemDirectoryW(path.data(), static_cast<UINT>(path.size()));
	if (written == 0 || written >= path.size())
		throw std::runtime_error("Windows PowerShell could not be located.");
	std::wstring executable(path.data(), written);
	executable += L"\\WindowsPowerShell\\v1.0\\powershell.exe";
	if (!std::filesystem::is_regular_file(executable))
		throw std::runtime_error("Windows PowerShell could not be located.");
	return executable;
}

// No escaping needed: Windows paths cannot contain quotes, and every other
// argument is hexadecimal or numeric.
std::wstring QuoteArgument(const std::wstring &value)
{
	return L"\"" + value + L"\"";
}

} // namespace

Updater AppUpdater;

bool HashFileSha256(const std::filesystem::path &path, std::array<unsigned char, 32> &digest)
{
	return HashFile(path, digest);
}

#ifdef QUESTCAL_UPDATER_TEST_SEAM
std::string VerifyPackageForTest(const std::filesystem::path &path,
	const ReleaseCandidate &release, const SigningPublicKey &key)
{
	return VerifyPackage(path, release, key);
}
#endif

Updater::~Updater()
{
	Shutdown();
}

void Updater::SetLogSink(LogSink sink)
{
	std::lock_guard<std::mutex> lock(mutex);
	logSink = std::move(sink);
}

void Updater::Log(const std::string &message) const
{
	try
	{
		LogSink sink;
		{
			std::lock_guard<std::mutex> lock(mutex);
			sink = logSink;
		}
		if (sink)
			sink(message);
	}
	catch (...)
	{
		// Diagnostics must never change updater behavior.
	}
}

void Updater::SetEnabled(bool value)
{
	bool start = false;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (stopping || enabled == value)
			return;
		enabled = value;
		++revision;
		readyRelease = ReleaseCandidate();
		readyPackagePath.clear();
		snapshot = Snapshot();
		snapshot.state = enabled ? State::Idle : State::Disabled;
		start = enabled;
	}
	Log(value ? "automatic updates enabled" : "automatic updates disabled");
	if (start)
		CheckNow();
}

bool Updater::CheckNow()
{
	std::thread finished;
	uint64_t checkRevision = 0;
	std::string prereleaseBuild;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (!enabled || stopping || running)
			return false;
		// A prerelease belongs to the hand-installed lane, so the stable feed is
		// never asked. Say which build this is and leave the move to the tester.
		if (IsPrerelease(build))
		{
			prereleaseBuild = VersionString(build);
			snapshot = Snapshot();
			snapshot.state = State::Prerelease;
			snapshot.version = prereleaseBuild;
			snapshot.message = "Nova Calibrator " + prereleaseBuild +
				" is a prerelease. Automatic updates follow stable releases only, "
				"so this build will not update itself. Install a stable release by "
				"hand to rejoin them.";
		}
		if (worker.joinable())
			finished = std::move(worker);
		if (prereleaseBuild.empty())
		{
			running = true;
			checkRevision = ++revision;
			snapshot = Snapshot();
			snapshot.state = State::Checking;
			snapshot.message = "Checking GitHub for updates";
		}
	}
	if (finished.joinable())
		finished.join();
	if (!prereleaseBuild.empty())
	{
		Log("check skipped: " + prereleaseBuild +
			" is a prerelease and is not on the stable lane");
		return false;
	}
	Log("check started (current " + VersionString(build) + ")");
	try
	{
		worker = std::thread(&Updater::RunChecks, this, checkRevision);
	}
	catch (const std::exception &failure)
	{
		bool current = false;
		{
			std::lock_guard<std::mutex> lock(mutex);
			running = false;
			current = IsCurrentLocked(checkRevision);
			if (current)
			{
				snapshot.state = State::Failed;
				snapshot.message = "The update check could not start.";
			}
		}
		if (current)
			Log("check could not start: " + std::string(failure.what()));
		return false;
	}
	return true;
}

Snapshot Updater::GetSnapshot() const
{
	std::lock_guard<std::mutex> lock(mutex);
	return snapshot;
}

bool Updater::IsCurrent(uint64_t checkRevision) const
{
	std::lock_guard<std::mutex> lock(mutex);
	return IsCurrentLocked(checkRevision);
}

bool Updater::Publish(uint64_t checkRevision, State state,
	const std::string &message, const std::string &version,
	uint64_t downloaded, uint64_t total)
{
	std::lock_guard<std::mutex> lock(mutex);
	if (!IsCurrentLocked(checkRevision))
		return false;
	snapshot.state = state;
	snapshot.message = message;
	snapshot.version = version;
	snapshot.downloadedBytes = downloaded;
	snapshot.totalBytes = total;
	return true;
}

void Updater::RunChecks(uint64_t checkRevision)
{
	for (;;)
	{
		RunCheck(checkRevision);
		std::lock_guard<std::mutex> lock(mutex);
		if (!enabled || stopping || revision == checkRevision)
		{
			running = false;
			return;
		}
		// Re-enabling while the previous network request is finishing queues a
		// fresh check on this worker; its cancelled result cannot publish.
		checkRevision = revision;
		snapshot = Snapshot();
		snapshot.state = State::Checking;
		snapshot.message = "Checking GitHub for updates";
	}
}

void Updater::RunCheck(uint64_t checkRevision)
{
	try
	{
		auto cancelled = [&]() { return !IsCurrent(checkRevision); };
		InternetHandle session;
		std::string feed;
#ifdef QUESTCAL_UPDATER_TEST_SEAM
		if (fetchForTest)
			feed = fetchForTest();
		else
#endif
		{
			session = OpenSession();
			feed = FetchReleaseFeed(session, cancelled);
		}
		ReleaseCandidate release;
		bool available = false;
		std::string policyError;
		const Version current = build;
		if (!SelectReleaseCandidate(feed, current, release, available, policyError))
			throw std::runtime_error(policyError);
		if (!available)
		{
			const std::string version = VersionString(current);
			if (Publish(checkRevision, State::UpToDate,
				"Nova Calibrator is up to date", version))
				Log("check completed: current " + version +
					"; no newer stable release");
		}
		else
		{
			const std::string version = VersionString(release.version);
			const SigningPublicKey key = ReleaseKey();
			if (!Publish(checkRevision, State::Downloading,
				"Downloading Nova Calibrator " + version, version, 0, release.size))
				throw std::runtime_error("Update check cancelled.");
			Log("stable release " + version + " found (" +
				std::to_string(release.size) + " bytes)");

			// The signature is small, so a release this build will not take is
			// refused before the package is downloaded.
			release.signature = DownloadSignature(session, release, cancelled);
			PackageSignature signature;
			if (!ParsePackageSignature(release.signature, signature))
				throw std::runtime_error("The release signature is malformed.");
			if (signature.keyId != key.keyId)
				throw std::runtime_error("The release is signed by a key this build does not trust.");

			const std::filesystem::path directory = LocalUpdateRoot() /
				Utf8ToWide(version);
			std::error_code directoryError;
			std::filesystem::create_directories(directory, directoryError);
			if (directoryError)
				throw std::runtime_error("The update folder could not be created (system error " +
					std::to_string(directoryError.value()) + ").");
			const std::filesystem::path package = directory /
				Utf8ToWide(release.packageName);
			bool downloaded = false;
			if (!VerifyPackage(package, release, key).empty())
			{
				const std::filesystem::path part = package.wstring() + L".part";
				std::error_code ignored;
				std::filesystem::remove(part, ignored);
				if (IsCurrent(checkRevision))
					Log("download started for " + version);
				DownloadPackage(session, release, part, cancelled,
					[&](uint64_t bytes) {
						Publish(checkRevision, State::Downloading,
							"Downloading Nova Calibrator " + version,
							version, bytes, release.size);
					});
				const std::string failure = VerifyPackage(part, release, key);
				if (!failure.empty())
				{
					std::filesystem::remove(part, ignored);
					throw std::runtime_error(failure);
				}
				std::filesystem::remove(package, ignored);
				std::error_code renameError;
				std::filesystem::rename(part, package, renameError);
				if (renameError)
				{
					std::filesystem::remove(part, ignored);
					throw std::runtime_error("The verified update package could not be saved (system error " +
						std::to_string(renameError.value()) + ").");
				}
				downloaded = true;
			}
			{
				std::lock_guard<std::mutex> lock(mutex);
				if (IsCurrentLocked(checkRevision))
				{
					readyRelease = release;
					readyPackagePath = package.wstring();
				}
			}
			if (Publish(checkRevision, State::Ready,
				"Nova Calibrator " + version + " is ready to install",
				version, release.size, release.size))
			{
				Log(std::string(downloaded ? "downloaded package " : "cached package ") +
					version + "; size, SHA-256 and signature verified; ready to install");
			}
		}
	}
	catch (const std::exception &error)
	{
		if (Publish(checkRevision, State::Failed, error.what()))
			Log("check failed: " + std::string(error.what()));
	}
}

bool Updater::LaunchInstaller(std::string &error)
{
	ReleaseCandidate release;
	std::filesystem::path package;
	uint64_t offeredRevision = 0;
	{
		std::lock_guard<std::mutex> lock(mutex);
		// Ready is only published after readyPackagePath is set for the same revision.
		if (!enabled || stopping || snapshot.state != State::Ready)
		{
			error = "No verified update is ready to install.";
		}
		else
		{
			release = readyRelease;
			package = readyPackagePath;
			offeredRevision = revision;
		}
	}
	if (package.empty())
	{
		Log("installer handoff rejected: " + error);
		return false;
	}

	try
	{
		// The signature is the copy downloaded with the check, held in memory;
		// the package beside it is in a folder the user can write to.
		const std::string failure = VerifyPackage(package, release, ReleaseKey());
		if (!failure.empty())
		{
			Log("installer handoff refused for " + VersionString(release.version) + ": " + failure);
			throw std::runtime_error("The downloaded update no longer passes verification.");
		}

		const std::filesystem::path script = package.parent_path() / L"ApplyUpdate.ps1";
		std::ofstream output(script, std::ios::binary | std::ios::trunc);
		if (!output)
			throw std::runtime_error("The update helper could not be created.");
		output << UpdateHelperScript();
		output.close();
		if (!output)
			throw std::runtime_error("The update helper could not be created.");

		const std::wstring powershell = SystemPowerShell();
		const std::wstring digestHex = Utf8ToWide(release.digest.substr(7));
		std::wstring command = QuoteArgument(powershell) +
			L" -NoProfile -ExecutionPolicy Bypass -File " + QuoteArgument(script.wstring()) +
			L" -ParentPid " + std::to_wstring(GetCurrentProcessId()) +
			L" -ZipPath " + QuoteArgument(package.wstring()) +
			L" -ExpectedSha256 " + QuoteArgument(digestHex);
		std::vector<wchar_t> commandLine(command.begin(), command.end());
		commandLine.push_back(L'\0');
		STARTUPINFOW startup{};
		startup.cb = sizeof startup;
		PROCESS_INFORMATION process{};
		{
			// File verification/helper creation can take time. Serialize the
			// final revision check and process creation with disable/shutdown.
			std::lock_guard<std::mutex> lock(mutex);
			if (snapshot.state != State::Ready || !IsCurrentLocked(offeredRevision))
				throw std::runtime_error("The update was cancelled before installer handoff.");
			if (!CreateProcessW(powershell.c_str(), commandLine.data(), nullptr,
				nullptr, FALSE, CREATE_NEW_CONSOLE, nullptr,
				package.parent_path().c_str(), &startup, &process))
				throw NetworkError("Starting the update helper");
			// A second click cannot launch the same ready handoff twice.
			snapshot.state = State::Idle;
			readyPackagePath.clear();
			readyRelease = ReleaseCandidate();
		}
		CloseHandle(process.hThread);
		CloseHandle(process.hProcess);
		Log("installer handoff started for " + VersionString(release.version) +
			" after SHA-256 and signature re-verification");
		return true;
	}
	catch (const std::exception &failure)
	{
		error = failure.what();
		Log("installer handoff failed for " + VersionString(release.version) +
			": " + error);
		return false;
	}
}

void Updater::Shutdown()
{
	std::thread active;
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (stopping && !worker.joinable())
			return;
		stopping = true;
		enabled = false;
		++revision;
		if (worker.joinable())
			active = std::move(worker);
		snapshot.state = State::Disabled;
	}
	if (active.joinable())
		active.join();
	std::lock_guard<std::mutex> lock(mutex);
	running = false;
}

} // namespace update
} // namespace questcal
