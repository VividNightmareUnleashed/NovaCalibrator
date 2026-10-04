#pragma once

// The release key's signature on an update package. Releases carry a minisign
// signature (https://jedisct1.github.io/minisign/) beside the package, made by
// the release workflow with the key whose public half is built into the app
// (UpdateSigningKey.h). Only minisign's default algorithm is accepted: Ed25519
// over the package's BLAKE2b-512 hash. The signed trusted comment must be the
// package's canonical name, so a release cannot carry the genuine signature of
// an older package under a newer name.

#include "../lib/monocypher/monocypher.h"

#include <array>
#include <cstddef>
#include <string>

namespace questcal
{
namespace update
{

struct SigningPublicKey
{
	std::array<unsigned char, 8> keyId{};
	std::array<unsigned char, 32> key{};
};

struct PackageSignature
{
	std::array<unsigned char, 8> keyId{};
	std::array<unsigned char, 64> signature{};
	std::string trustedComment;
	std::array<unsigned char, 64> commentSignature{};
};

// Standard base64 with padding. Each byte string has one accepted spelling:
// padding only at the end and unused low bits zero.
bool DecodeBase64(const std::string &text, std::string &bytes);

// The key line of a minisign public key file, its second line.
bool ParseSigningPublicKey(const std::string &encoded, SigningPublicKey &key);

// A whole .minisig file: four lines, ending in LF or CRLF as minisign writes
// them on Linux or Windows, at most MaxSignatureBytes (UpdatePolicy.h).
bool ParsePackageSignature(const std::string &text, PackageSignature &signature);

// True when `signature` is `key`'s signature of a file whose BLAKE2b-512 hash
// is `fileHash`, with the trusted comment `expectedComment`, also signed.
bool VerifyPackageSignature(const SigningPublicKey &key, const PackageSignature &signature,
	const std::array<unsigned char, 64> &fileHash, const std::string &expectedComment);

// BLAKE2b-512, fed in pieces.
class Blake2b512
{
public:
	Blake2b512() { crypto_blake2b_init(&context, 64); }
	void Update(const void *data, size_t size)
	{
		crypto_blake2b_update(&context, static_cast<const uint8_t *>(data), size);
	}
	std::array<unsigned char, 64> Final()
	{
		std::array<unsigned char, 64> hash{};
		crypto_blake2b_final(&context, hash.data());
		return hash;
	}

private:
	crypto_blake2b_ctx context;
};

} // namespace update
} // namespace questcal
