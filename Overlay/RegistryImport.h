#pragma once

#include <windows.h>

namespace questcal
{

#ifdef QUESTCAL_REGISTRY_IMPORT_TEST_SEAM
// When set, runs instead of RegDeleteTreeA on the staging key, so a test can
// leave there a leftover the import cannot remove.
inline LSTATUS (*DeleteStagingForTest)(HKEY root, const char *staging) = nullptr;
#endif

// Copies the registry tree at `from` to `to`, both under `root`, unless `to`
// exists already. The copy is made under `staging` and renamed to `to` (whose
// last component is `toLeaf`) only once it is complete, so `to` never exists
// half copied: a start that dies part-way leaves only `staging`, which the
// next call deletes before it copies again. `staging` must share `to`'s parent.
//
// ERROR_SUCCESS: copied. ERROR_ALREADY_EXISTS: `to` was there, and nothing was
// done. ERROR_FILE_NOT_FOUND: there is no `from` to read. Any other value is
// the error that stopped the import, which leaves no `to`, and no `staging`
// unless it was a `staging` left from before that could not be deleted.
inline LSTATUS ImportRegistryTree(HKEY root, const char *from, const char *to,
	const char *staging, const wchar_t *toLeaf)
{
	HKEY existing = nullptr;
	if (RegOpenKeyExA(root, to, 0, KEY_READ, &existing) == ERROR_SUCCESS)
	{
		RegCloseKey(existing);
		return ERROR_ALREADY_EXISTS;
	}
	HKEY source = nullptr;
	if (RegOpenKeyExA(root, from, 0, KEY_READ, &source) != ERROR_SUCCESS)
		return ERROR_FILE_NOT_FOUND;
	// What an earlier import left when it died before the rename. One that
	// stays must not be copied over: RegCopyTreeA keeps what the destination
	// holds, so the new key would have what the old key has since lost.
	LSTATUS result =
#ifdef QUESTCAL_REGISTRY_IMPORT_TEST_SEAM
		DeleteStagingForTest ? DeleteStagingForTest(root, staging) :
#endif
		RegDeleteTreeA(root, staging);
	if (result != ERROR_SUCCESS && result != ERROR_FILE_NOT_FOUND)
	{
		RegCloseKey(source);
		return result;
	}
	HKEY copy = nullptr;
	result = RegCreateKeyExA(root, staging, 0, nullptr, 0, KEY_ALL_ACCESS, nullptr, &copy, nullptr);
	if (result == ERROR_SUCCESS)
	{
		result = RegCopyTreeA(source, nullptr, copy);
		if (result == ERROR_SUCCESS)
			result = RegRenameKey(copy, nullptr, toLeaf);
		RegCloseKey(copy);
		if (result != ERROR_SUCCESS)
			RegDeleteTreeA(root, staging);
	}
	RegCloseKey(source);
	return result;
}

} // namespace questcal
