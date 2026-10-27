#pragma once

#include <windows.h>

namespace questcal
{

// Copies the registry tree at `from` to `to`, both under `root`, unless `to`
// exists already. The copy is made under `staging` and renamed to `to` (whose
// last component is `toLeaf`) only once it is complete, so `to` never exists
// half copied: a start that dies part-way leaves only `staging`, which the
// next call deletes before it copies again. `staging` must share `to`'s parent.
//
// ERROR_SUCCESS: copied. ERROR_ALREADY_EXISTS: `to` was there, and nothing was
// done. ERROR_FILE_NOT_FOUND: there is no `from` to read. Any other value is
// the error that stopped the copy, which leaves neither `to` nor `staging`.
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
	// What an earlier import left when it died before the rename.
	RegDeleteTreeA(root, staging);
	HKEY copy = nullptr;
	LSTATUS result = RegCreateKeyExA(root, staging, 0, nullptr, 0, KEY_ALL_ACCESS, nullptr, &copy, nullptr);
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
