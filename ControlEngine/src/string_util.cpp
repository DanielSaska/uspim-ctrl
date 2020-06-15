#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include "string_util.h"

template<typename Out>
void split(std::string const& s, char delim, Out result) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		*(result++) = item;
	}
}

std::vector<std::string> split(std::string const& s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, std::back_inserter(elems));
	return elems;
}
