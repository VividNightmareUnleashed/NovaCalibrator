#include "UpdateSignature.h"
#include "UpdatePolicy.h"

#include "../lib/monocypher/monocypher-ed25519.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace questcal
{
namespace update
{
namespace
{

int Base64Value(char c)
{
	if (c >= 'A' && c <= 'Z') return c - 'A';
	if (c >= 'a' && c <= 'z') return c - 'a' + 26;
	if (c >= '0' && c <= '9') return c - '0' + 52;
	if (c == '+') return 62;
	if (c == '/') return 63;
	return -1;
}

bool DecodeExactly(const std::string &text, size_t size, std::string &bytes)
{
	return DecodeBase64(text, bytes) && bytes.size() == size;
}

template<size_t N>
void CopyBytes(const std::string &bytes, size_t offset, std::array<unsigned char, N> &out)
{
	std::memcpy(out.data(), bytes.data() + offset, N);
}

// `line` with its prefix removed, if it has that prefix.
bool StripPrefix(const std::string &line, const char *prefix, std::string &rest)
{
	const size_t length = std::strlen(prefix);
	if (line.compare(0, length, prefix) != 0)
		return false;
	rest = line.substr(length);
	return true;
}

} // namespace

bool DecodeBase64(const std::string &text, std::string &bytes)
{
	bytes.clear();
	if (text.empty() || text.size() % 4 != 0)
		return false;
	size_t padding = 0;
	if (text[text.size() - 1] == '=')
		padding = text[text.size() - 2] == '=' ? 2 : 1;
	for (size_t group = 0; group < text.size(); group += 4)
	{
		const bool last = group + 4 == text.size();
		const size_t digits = last ? 4 - padding : 4;
		uint32_t value = 0;
		for (size_t i = 0; i < 4; ++i)
		{
			const char c = text[group + i];
			int digit = 0;
			if (i < digits)
			{
				digit = Base64Value(c);
				if (digit < 0)
					return false;
			}
			else if (c != '=')
			{
				return false;
			}
			value = (value << 6) | static_cast<uint32_t>(digit);
		}
		// The bits a padded group leaves unused must be zero, or two texts
		// would decode to the same bytes.
		if ((digits == 3 && (value & 0xff) != 0) || (digits == 2 && (value & 0xffff) != 0))
			return false;
		bytes.push_back(static_cast<char>(value >> 16));
		if (digits >= 3)
			bytes.push_back(static_cast<char>((value >> 8) & 0xff));
		if (digits == 4)
			bytes.push_back(static_cast<char>(value & 0xff));
	}
	return true;
}

bool ParseSigningPublicKey(const std::string &encoded, SigningPublicKey &key)
{
	// "Ed", the key id, then the Ed25519 public key.
	std::string bytes;
	if (!DecodeExactly(encoded, 2 + 8 + 32, bytes) || bytes.compare(0, 2, "Ed") != 0)
		return false;
	CopyBytes(bytes, 2, key.keyId);
	CopyBytes(bytes, 10, key.key);
	return true;
}

bool ParsePackageSignature(const std::string &text, PackageSignature &signature)
{
	if (text.size() > MaxSignatureBytes)
		return false;
	std::vector<std::string> lines;
	size_t start = 0;
	while (start < text.size())
	{
		size_t end = text.find('\n', start);
		const size_t next = end == std::string::npos ? text.size() : end + 1;
		if (end == std::string::npos)
			end = text.size();
		if (end > start && text[end - 1] == '\r')
			--end;
		lines.push_back(text.substr(start, end - start));
		start = next;
	}
	if (lines.size() != 4)
		return false;
	for (const std::string &line : lines)
		if (line.find_first_of(std::string("\r\0", 2)) != std::string::npos)
			return false;

	std::string untrusted, bytes, comment;
	if (!StripPrefix(lines[0], "untrusted comment: ", untrusted) ||
		!StripPrefix(lines[2], "trusted comment: ", comment))
		return false;
	// "ED" is the prehashed algorithm; the legacy "Ed" signs the whole file,
	// which minisign no longer writes by default.
	if (!DecodeExactly(lines[1], 2 + 8 + 64, bytes) || bytes.compare(0, 2, "ED") != 0)
		return false;
	PackageSignature parsed;
	CopyBytes(bytes, 2, parsed.keyId);
	CopyBytes(bytes, 10, parsed.signature);
	if (!DecodeExactly(lines[3], 64, bytes))
		return false;
	CopyBytes(bytes, 0, parsed.commentSignature);
	parsed.trustedComment = comment;
	signature = std::move(parsed);
	return true;
}

bool VerifyPackageSignature(const SigningPublicKey &key, const PackageSignature &signature,
	const std::array<unsigned char, 64> &fileHash, const std::string &expectedComment)
{
	if (signature.keyId != key.keyId || signature.trustedComment != expectedComment)
		return false;
	if (crypto_ed25519_check(signature.signature.data(), key.key.data(),
		fileHash.data(), fileHash.size()) != 0)
		return false;
	// minisign signs the file signature followed by the trusted comment.
	std::vector<uint8_t> signedComment(signature.signature.begin(), signature.signature.end());
	signedComment.insert(signedComment.end(), signature.trustedComment.begin(),
		signature.trustedComment.end());
	return crypto_ed25519_check(signature.commentSignature.data(), key.key.data(),
		signedComment.data(), signedComment.size()) == 0;
}

} // namespace update
} // namespace questcal
