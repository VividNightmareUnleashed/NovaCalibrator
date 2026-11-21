#pragma once

// The one nesting guard every picojson parse of untrusted text goes through:
// the registry records and the GitHub release feed alike.

#include "RecordBounds.h"

#include <stdexcept>
#include <string>

namespace questcal
{

// picojson recurses with no depth limit, and a stack overflow is an SEH
// exception no catch sees: a deeply nested registry value would crash at
// startup with no log, and a deeply nested release feed would crash the update
// check. Reject that shape before parsing so it takes the caller's ordinary
// refusal path. The records nest at most three levels and the release feed
// about five; brackets inside strings do not count.
inline void RejectExcessiveJsonNesting(const std::string &text)
{
	int depth = 0;
	bool inString = false;
	bool escaped = false;
	for (char c : text)
	{
		if (inString)
		{
			if (escaped)
				escaped = false;
			else if (c == '\\')
				escaped = true;
			else if (c == '"')
				inString = false;
			continue;
		}

		if (c == '"')
			inString = true;
		else if (c == '[' || c == '{')
		{
			if (!IsValidJsonDepth(++depth))
				throw std::runtime_error("record nesting is too deep");
		}
		else if (c == ']' || c == '}')
			--depth;
	}
}

// One UTF-8 sequence at `i`: its length and code point, or 0 for a byte that
// starts none (a lone continuation byte, an overlong form, a surrogate, a
// value past U+10FFFF, a sequence the text ends inside).
inline int DecodeUtf8(const std::string &text, size_t i, unsigned &codePoint)
{
	const unsigned char lead = static_cast<unsigned char>(text[i]);
	if (lead < 0x80)
	{
		codePoint = lead;
		return 1;
	}
	const int length = lead >= 0xf0 && lead <= 0xf4 ? 4 : lead >= 0xe0 && lead < 0xf0 ? 3 :
		lead >= 0xc2 && lead < 0xe0 ? 2 : 0;
	if (length == 0 || i + static_cast<size_t>(length) > text.size())
		return 0;
	codePoint = length == 4 ? lead & 0x07u : length == 3 ? lead & 0x0fu : lead & 0x1fu;
	for (int k = 1; k < length; ++k)
	{
		const unsigned char next = static_cast<unsigned char>(text[i + static_cast<size_t>(k)]);
		if ((next & 0xc0) != 0x80)
			return 0;
		codePoint = (codePoint << 6) | (next & 0x3fu);
	}
	const unsigned smallest = length == 4 ? 0x10000u : length == 3 ? 0x800u : 0x80u;
	if (codePoint < smallest || codePoint > 0x10ffffu || (codePoint >= 0xd800u && codePoint <= 0xdfffu))
		return 0;
	return length;
}

// A record's text with each byte that is not UTF-8 replaced by U+FFFD, so
// every string a parse accepts is UTF-8 and the next save, which writes
// escapes (EscapeNonAsciiJson), reads back as the same record.
inline std::string ReplaceInvalidUtf8(const std::string &text)
{
	std::string out;
	out.reserve(text.size());
	for (size_t i = 0; i < text.size();)
	{
		unsigned codePoint = 0;
		const int length = DecodeUtf8(text, i, codePoint);
		if (length == 0)
		{
			out += "\xef\xbf\xbd";
			++i;
			continue;
		}
		out.append(text, i, static_cast<size_t>(length));
		i += static_cast<size_t>(length);
	}
	return out;
}

// The records reach the registry through its ANSI entry points, which pass
// text through the system code page both ways. On a double-byte code page
// (Japanese, Chinese, Korean) the last byte of a UTF-8 name can read as a lead
// byte and take the quote after it, and the record no longer parses at the
// next start. Text with nothing above ASCII is the same in every code page, so
// a record is written with each code point above it as its \u escape, which
// picojson reads back as the same UTF-8. A serialized record has such bytes
// only inside its strings. A byte that is not UTF-8 becomes U+FFFD.
inline std::string EscapeNonAsciiJson(const std::string &json)
{
	static const char hex[] = "0123456789abcdef";
	std::string out;
	out.reserve(json.size());
	const auto escape = [&out](unsigned unit)
	{
		out += "\\u";
		for (int shift = 12; shift >= 0; shift -= 4)
			out += hex[(unit >> shift) & 0xf];
	};

	for (size_t i = 0; i < json.size();)
	{
		unsigned codePoint = 0;
		const int length = DecodeUtf8(json, i, codePoint);
		if (length == 1)
			out += json[i];
		else if (length == 0)
			escape(0xfffdu);
		else if (codePoint >= 0x10000u)
		{
			const unsigned offset = codePoint - 0x10000u;
			escape(0xd800u + (offset >> 10));
			escape(0xdc00u + (offset & 0x3ffu));
		}
		else
			escape(codePoint);
		i += static_cast<size_t>(length == 0 ? 1 : length);
	}
	return out;
}

}
