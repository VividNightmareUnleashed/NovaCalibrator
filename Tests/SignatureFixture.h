#pragma once

// A throwaway minisign key, not the release key, and signatures minisign 0.12
// made with it on Windows (CRLF lines), for the signature tests and the
// signature fuzz target. Its secret half was not kept.

namespace questcalfixture
{

// The test key's public key line, and an unrelated key's.
inline constexpr char SigningKey[] = "RWQcKCkkrqWZLoqvqW+6r3jZxlxXBQ22TLKtKjG3ZGG6hlBV3Ql/JPzH";
inline constexpr char OtherSigningKey[] = "RWTWjLFPPqcoMs0K9RsmAnOiGCCFwvPp2KEFDBm670m6ObOiXwTEUMBg";

// The file signed, and the trusted comment every signature here carries.
inline constexpr char SignedFile[] = "QuestCalibrator signature fixture\n";
inline constexpr char SignedComment[] = "QuestCalibrator-9.9.9.zip";

// minisign -S -t QuestCalibrator-9.9.9.zip: the prehashed algorithm.
inline constexpr char Signature[] =
	"untrusted comment: signature from the test key\r\nRUQcKCkkrqWZLvK1RjMTcNvD2wJ++/E5yqdxxY02ib3OWWnjVa7hOFdPBwVAoISTRALazKx9zKlsGzkyvB3rZAUkA6+KD6HXVAE=\r\ntrusted comment: QuestCalibrator-9.9.9.zip\r\n1HinLrZ89ZpMeX6ZwWNFRmdWeya4YFEsFvWD422EoI1jmqCX5ocN4aIFhDO51pFFNFmzftUBmlff6A6T27MtBg==\r\n";
// The same with -l, the legacy algorithm that signs the whole file.
inline constexpr char LegacySignature[] =
	"untrusted comment: signature from minisign secret key\r\nRWQcKCkkrqWZLqiVLWJl2OQ4F0NGTe+OMnFVc/cYtIuU9VlBESPmohI3l1ESU6CILB2Up7VrV1CsiwbTxLHgiX167suftssrPAQ=\r\ntrusted comment: QuestCalibrator-9.9.9.zip\r\nHYrEUD5mhQRVl5DU6Q5F4otX8J3P05rG3Bj2I/UbQ/wLmiIgYvjBQSOGzVQVzIckhZlJEHxWCZgAx1hmwBeHDQ==\r\n";
// The other key's signature of the same file.
inline constexpr char OtherSignature[] =
	"untrusted comment: signature from minisign secret key\r\nRUTWjLFPPqcoMtqtvGeVjVyVssOHhntibs1Is+wAO5n9O+/S+ymXnVFFe2w037R/eNCGWrRKako4yJc+J2oWAZp3RMfMR1rLZg0=\r\ntrusted comment: QuestCalibrator-9.9.9.zip\r\nVly+khxAGjemQkUQJWTZJgnBSWLNEZwVS6/2yqFHhIBUxr5YkgFVtAoZsu3vEzr+N3ezec6/VQHAZdPmUu9xAA==\r\n";

} // namespace questcalfixture
