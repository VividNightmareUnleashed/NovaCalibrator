#include "../Overlay/UpdateSignature.h"
#include "../Overlay/UpdateSigningKey.h"
#include "../Overlay/Updater.h"
#include "SignatureFixture.h"

#include <windows.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

// The release key's signature on update packages (Overlay/UpdateSignature.h),
// against real minisign output from a throwaway key (SignatureFixture.h).
namespace
{
using Check = void (*)(const char *, bool, const char *);
using namespace questcal::update;

std::array<unsigned char, 64> Blake2b(const std::string &data)
{
	Blake2b512 hash;
	hash.Update(data.data(), data.size());
	return hash.Final();
}

std::string Base64(const std::string &bytes)
{
	static const char digits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::string text;
	for (size_t i = 0; i < bytes.size(); i += 3)
	{
		uint32_t group = static_cast<uint32_t>(static_cast<unsigned char>(bytes[i])) << 16;
		if (i + 1 < bytes.size()) group |= static_cast<uint32_t>(static_cast<unsigned char>(bytes[i + 1])) << 8;
		if (i + 2 < bytes.size()) group |= static_cast<unsigned char>(bytes[i + 2]);
		text += digits[(group >> 18) & 63];
		text += digits[(group >> 12) & 63];
		text += i + 1 < bytes.size() ? digits[(group >> 6) & 63] : '=';
		text += i + 2 < bytes.size() ? digits[group & 63] : '=';
	}
	return text;
}

std::string Replace(std::string text, const std::string &from, const std::string &to)
{
	const size_t at = text.find(from);
	if (at != std::string::npos)
		text.replace(at, from.size(), to);
	return text;
}

// The fixture's line `index` (0-based), without its line ending.
std::string Line(const std::string &signature, int index)
{
	size_t start = 0;
	for (int i = 0; i < index; ++i)
		start = signature.find('\n', start) + 1;
	return signature.substr(start, signature.find('\r', start) - start);
}

bool Verifies(const std::string &signatureText, const std::string &file = questcalfixture::SignedFile,
	const std::string &comment = questcalfixture::SignedComment,
	const std::string &keyText = questcalfixture::SigningKey)
{
	SigningPublicKey key;
	PackageSignature signature;
	return ParseSigningPublicKey(keyText, key) && ParsePackageSignature(signatureText, signature) &&
		VerifyPackageSignature(key, signature, Blake2b(file), comment);
}

void Base64Scenario(Check check)
{
	std::string bytes;
	const bool canonical = DecodeBase64("QQ==", bytes) && bytes == "A" &&
		DecodeBase64("QUI=", bytes) && bytes == "AB" &&
		DecodeBase64("QUJD", bytes) && bytes == "ABC" &&
		DecodeBase64(Base64(std::string("\x00\xff\x10", 3)), bytes) && bytes == std::string("\x00\xff\x10", 3);
	// Unused bits set, padding missing, misplaced or overlong, and bytes
	// outside the alphabet.
	const bool refused = !DecodeBase64("QR==", bytes) && !DecodeBase64("QUJ=", bytes) &&
		!DecodeBase64("QQ=", bytes) && !DecodeBase64("Q===", bytes) && !DecodeBase64("QQ=A", bytes) &&
		!DecodeBase64("Q=Q=", bytes) && !DecodeBase64("====", bytes) && !DecodeBase64("", bytes) &&
		!DecodeBase64("QU-D", bytes) && !DecodeBase64("QUJD\n", bytes) && !DecodeBase64("QUJDQQ==QUJD", bytes);
	check("update signature: base64 has one spelling per byte string", canonical && refused, "");
}

void Blake2bScenario(Check check)
{
	// RFC 7693, appendix A.
	static const unsigned char abc[64] = {
		0xba, 0x80, 0xa5, 0x3f, 0x98, 0x1c, 0x4d, 0x0d, 0x6a, 0x27, 0x97, 0xb6, 0x9f, 0x12, 0xf6, 0xe9,
		0x4c, 0x21, 0x2f, 0x14, 0x68, 0x5a, 0xc4, 0xb7, 0x4b, 0x12, 0xbb, 0x6f, 0xdb, 0xff, 0xa2, 0xd1,
		0x7d, 0x87, 0xc5, 0x39, 0x2a, 0xab, 0x79, 0x2d, 0xc2, 0x52, 0xd5, 0xde, 0x45, 0x33, 0xcc, 0x95,
		0x18, 0xd3, 0x8a, 0xa8, 0xdb, 0xf1, 0x92, 0x5a, 0xb9, 0x23, 0x86, 0xed, 0xd4, 0x00, 0x99, 0x23 };
	const std::array<unsigned char, 64> whole = Blake2b("abc");
	Blake2b512 pieces;
	const std::string file = questcalfixture::SignedFile;
	for (size_t i = 0; i < file.size(); i += 5)
		pieces.Update(file.data() + i, (std::min)(size_t(5), file.size() - i));
	check("update signature: BLAKE2b-512 matches RFC 7693, fed whole or in pieces",
		std::memcmp(whole.data(), abc, 64) == 0 && pieces.Final() == Blake2b(file), "");
}

void GenuineScenario(Check check)
{
	// minisign writes CRLF on Windows and LF on Linux; the last newline is
	// optional.
	const std::string crlf = questcalfixture::Signature;
	std::string lf = crlf;
	for (size_t at; (at = lf.find('\r')) != std::string::npos;)
		lf.erase(at, 1);
	check("update signature: the key's signature verifies, with LF or CRLF lines",
		Verifies(crlf) && Verifies(lf) && Verifies(lf.substr(0, lf.size() - 1)) &&
		Verifies(Replace(crlf, "signature from the test key", "anything at all")), "");
}

void TamperScenario(Check check)
{
	std::string file = questcalfixture::SignedFile;
	file[3] ^= 1;
	const std::string genuine = questcalfixture::Signature;
	// A comment changed on the line, against its own signature.
	const std::string comment = Replace(genuine, "trusted comment: QuestCalibrator-9.9.9.zip",
		"trusted comment: QuestCalibrator-9.9.10.zip");
	// One bit of the file signature changed.
	std::string signatureBytes;
	DecodeBase64(Line(genuine, 1), signatureBytes);
	signatureBytes[40] ^= 0x10;
	const std::string flipped = Replace(genuine, Line(genuine, 1), Base64(signatureBytes));
	check("update signature: a changed file, signature or trusted comment is refused",
		!Verifies(genuine, file) && !Verifies(comment, questcalfixture::SignedFile, "QuestCalibrator-9.9.10.zip") &&
		!Verifies(flipped), "");
	// The genuine signature of 9.9.9 offered for a newer package name.
	check("update signature: the trusted comment must name the package",
		!Verifies(genuine, questcalfixture::SignedFile, "QuestCalibrator-9.9.10.zip") &&
		!Verifies(genuine, questcalfixture::SignedFile, ""), "");
}

void OtherKeyScenario(Check check)
{
	const std::string other = questcalfixture::OtherSignature;
	SigningPublicKey key;
	ParseSigningPublicKey(questcalfixture::SigningKey, key);
	// The other key's signature, relabelled with the test key's id.
	std::string bytes;
	DecodeBase64(Line(other, 1), bytes);
	std::memcpy(&bytes[2], key.keyId.data(), key.keyId.size());
	const std::string relabelled = Replace(other, Line(other, 1), Base64(bytes));
	check("update signature: another key's signature is refused, relabelled or not",
		Verifies(other, questcalfixture::SignedFile, questcalfixture::SignedComment,
			questcalfixture::OtherSigningKey) &&
		!Verifies(other) && !Verifies(relabelled), "");
}

void MalformedScenario(Check check)
{
	const std::string genuine = questcalfixture::Signature;
	PackageSignature signature;
	SigningPublicKey key;
	const bool refused = !ParsePackageSignature(questcalfixture::LegacySignature, signature) &&
		!ParsePackageSignature(genuine + "\r\n", signature) &&
		!ParsePackageSignature(genuine.substr(0, genuine.find("\r\ntrusted comment:") + 2), signature) &&
		!ParsePackageSignature(Replace(genuine, "untrusted comment: ", "untrusted: "), signature) &&
		!ParsePackageSignature(Replace(genuine, "trusted comment: Quest", "trusted comment:Quest"), signature) &&
		!ParsePackageSignature(Replace(genuine, "the test key", "the\rtest key"), signature) &&
		!ParsePackageSignature(Replace(genuine, "RUQc", "RUQ"), signature) &&
		!ParsePackageSignature(Replace(genuine, "the test key", std::string(4096, 'k')), signature) &&
		!ParsePackageSignature("", signature) &&
		!ParseSigningPublicKey("", key) &&
		!ParseSigningPublicKey(Line(genuine, 1), key) &&
		!ParseSigningPublicKey(std::string(questcalfixture::SigningKey) + "=", key);
	check("update signature: the legacy algorithm and malformed files are refused", refused, "");
	SigningPublicKey builtIn;
	check("update signature: the built-in release key, once set, parses",
		ReleaseSigningPublicKey[0] == '\0' || ParseSigningPublicKey(ReleaseSigningPublicKey, builtIn),
		ReleaseSigningPublicKey);
}

void PackageScenario(Check check)
{
	wchar_t folder[MAX_PATH] = {};
	GetTempPathW(MAX_PATH, folder);
	const std::filesystem::path path = std::filesystem::path(folder) /
		(L"questcal-signature-" + std::to_wstring(GetCurrentProcessId()) + L".zip");
	auto write = [&](const std::string &bytes) {
		std::ofstream(path, std::ios::binary | std::ios::trunc).write(bytes.data(),
			static_cast<std::streamsize>(bytes.size()));
	};
	const std::string file = questcalfixture::SignedFile;
	write(file);

	SigningPublicKey key;
	ParseSigningPublicKey(questcalfixture::SigningKey, key);
	ReleaseCandidate release;
	release.packageName = questcalfixture::SignedComment;
	release.size = file.size();
	release.signature = questcalfixture::Signature;
	HashFileSha256(path, release.digestBytes);

	const bool genuine = VerifyPackageForTest(path, release, key).empty();
	ReleaseCandidate wrongSize = release;
	++wrongSize.size;
	ReleaseCandidate wrongDigest = release;
	wrongDigest.digestBytes[0] ^= 1;
	ReleaseCandidate renamed = release;
	renamed.packageName = "QuestCalibrator-9.9.10.zip";
	ReleaseCandidate unsignedRelease = release;
	unsignedRelease.signature.clear();
	const bool refusedMetadata = !VerifyPackageForTest(path, wrongSize, key).empty() &&
		!VerifyPackageForTest(path, wrongDigest, key).empty() &&
		!VerifyPackageForTest(path, renamed, key).empty() &&
		!VerifyPackageForTest(path, unsignedRelease, key).empty();
	SigningPublicKey other;
	ParseSigningPublicKey(questcalfixture::OtherSigningKey, other);
	const bool refusedKey = !VerifyPackageForTest(path, release, other).empty();

	// Same size, one byte changed: the SHA-256 the release names fails first.
	std::string changed = file;
	changed[0] ^= 1;
	write(changed);
	const std::string changedResult = VerifyPackageForTest(path, release, key);
	std::filesystem::remove(path);
	const bool refusedMissing = !VerifyPackageForTest(path, release, key).empty();
	check("update signature: a package must match its size, SHA-256 and signature at once",
		genuine && refusedMetadata && refusedKey && !changedResult.empty() && refusedMissing,
		changedResult.c_str());
}

} // namespace

void RunUpdateSignatureScenarios(void (*check)(const char *, bool, const char *))
{
	Base64Scenario(check);
	Blake2bScenario(check);
	GenuineScenario(check);
	TamperScenario(check);
	OtherKeyScenario(check);
	MalformedScenario(check);
	PackageScenario(check);
}
