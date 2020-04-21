#include "algos.hpp"

int main(int argc, char ** argv) {
	std::vector<std::vector<int> > m = {
		std::vector<int>({0, 1, 0, 0}),
		std::vector<int>({0, 0, 1, 1}),
		std::vector<int>({1, 1, 0, 1}),
		std::vector<int>({1, 0, 0, 0}),
	};
	const int size = 4;
	Warshall(size, m);
	getchar();
	return 0;
}