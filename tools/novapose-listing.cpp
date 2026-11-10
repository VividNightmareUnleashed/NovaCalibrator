// Prints the listing of a pose recording (docs/recording-format.md, "The
// listing").
//
//   cl /std:c++17 /EHsc /W4 /I ..\Overlay novapose-listing.cpp
//   novapose-listing FILE
//
// This is the whole of the C++ reader in a program of its own: it includes the
// format header and the listing header and nothing of Nova Calibrator, SteamVR
// or a UI library, which is what lets another program use them as they are.
// tools/check-recording-readers.ps1 compiles it with nothing else on its
// include path and compares its output with Tests/recordings/*.txt.
#include "PoseRecordingListing.h"

#include <cstdio>
#include <fstream>
#include <iostream>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		std::fprintf(stderr, "Usage: novapose-listing FILE\n");
		return 2;
	}
	std::ifstream in(argv[1], std::ios::in | std::ios::binary);
	if (!in.is_open())
	{
		std::fprintf(stderr, "%s could not be opened.\n", argv[1]);
		return 1;
	}
#ifdef _WIN32
	// The listing's lines end in a line feed alone, on every system.
	_setmode(_fileno(stdout), _O_BINARY);
#endif
	const std::string listing = questcal::recording::Listing(in);
	std::fwrite(listing.data(), 1, listing.size(), stdout);
	return 0;
}
