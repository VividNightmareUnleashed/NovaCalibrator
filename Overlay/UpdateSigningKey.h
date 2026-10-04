#pragma once

namespace questcal
{
namespace update
{

// The public half of the release signing key: the second line of its minisign
// .pub file. The updater installs only packages this key signed, and the
// release workflow signs with the secret half (docs/releasing.md, Signing).
// While it is empty this build verifies nothing, so it takes no update, and
// install\package-release.ps1 refuses to package a stable release.
inline constexpr char ReleaseSigningPublicKey[] = "RWROz0EyJk7+y7g6PuWXpFhPJVXMbDOleI2Xi0ENupoRI1v6qd7SVF9i";

} // namespace update
} // namespace questcal
