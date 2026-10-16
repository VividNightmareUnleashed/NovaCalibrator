#pragma once

#include <cstdint>
#include <cstring>

namespace questcal::hookchain
{

// Where a call into a hooked function really goes first. A MinHook user
// patches the function's entry with a jump to its relay, which jumps on to its
// detour; a second user loaded later patches the same entry again, and its
// detour then runs ahead of the first one's. Following only jumps from the
// address a caller reaches tells whether `detour` is the first code to run, or
// whose code is.
struct Walk
{
	// The first address that is not a jump this walk follows: `detour` when it
	// is reached, else the code that runs first.
	uintptr_t end = 0;
	bool reachedDetour = false;
	// `end`, or the pointer the jump at `end` reads its target from, could not
	// be read.
	bool unreadable = false;
	uint32_t hops = 0;
};

// Enough for MinHook's own chain (entry, relay) behind a hot-patch jump and an
// incremental-link thunk, with room for other hookers' relays.
constexpr uint32_t MaxHops = 8;

// The longest jump decoded: mov r11, imm64; jmp r11.
constexpr size_t MaxJumpBytes = 13;

// `read(address, out, size)` copies `size` bytes at `address`, or returns false
// when they are not readable; nothing else is ever dereferenced. Decoded: jmp
// rel32 (E9), jmp rel8 (EB), jmp [rip+disp32] (FF 25), and mov rax/r11, imm64
// followed by jmp rax/r11, the forms x64 hooking libraries write. Stops at
// `detour` before decoding it, so a thunk behind the detour is not followed.
template <class Read>
Walk Follow(uintptr_t start, uintptr_t detour, Read &&read)
{
	Walk walk;
	uintptr_t at = start;
	for (;;)
	{
		walk.end = at;
		if (at == detour)
		{
			walk.reachedDetour = true;
			return walk;
		}
		if (walk.hops >= MaxHops)
			return walk;

		uint8_t code[MaxJumpBytes] = {};
		// A jump near the end of a readable region is still decoded: read what
		// the shortest form needs first, then the rest only when its opcode
		// asks for more.
		if (!read(at, code, 2))
		{
			walk.unreadable = true;
			return walk;
		}
		uintptr_t next = 0;
		if (code[0] == 0xEB)
		{
			next = at + 2 + static_cast<intptr_t>(static_cast<int8_t>(code[1]));
		}
		else if (code[0] == 0xE9)
		{
			if (!read(at, code, 5))
				return walk;
			int32_t rel;
			std::memcpy(&rel, code + 1, sizeof rel);
			next = at + 5 + static_cast<intptr_t>(rel);
		}
		else if (code[0] == 0xFF && code[1] == 0x25)
		{
			if (!read(at, code, 6))
				return walk;
			int32_t disp;
			std::memcpy(&disp, code + 2, sizeof disp);
			uint64_t target = 0;
			if (!read(at + 6 + static_cast<intptr_t>(disp), &target, sizeof target))
			{
				walk.unreadable = true;
				return walk;
			}
			next = static_cast<uintptr_t>(target);
		}
		else if ((code[0] == 0x48 && code[1] == 0xB8) || (code[0] == 0x49 && code[1] == 0xBB))
		{
			const bool r11 = code[0] == 0x49;
			if (!read(at, code, r11 ? 13 : 12))
				return walk;
			const bool jumps = r11
				? code[10] == 0x41 && code[11] == 0xFF && code[12] == 0xE3
				: code[10] == 0xFF && code[11] == 0xE0;
			if (!jumps)
				return walk;
			uint64_t target;
			std::memcpy(&target, code + 2, sizeof target);
			next = static_cast<uintptr_t>(target);
		}
		else
		{
			return walk;
		}
		++walk.hops;
		at = next;
	}
}

} // namespace questcal::hookchain
