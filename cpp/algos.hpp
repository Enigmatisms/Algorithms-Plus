#pragma once
#ifndef ALGO_HPP
#define ALGO_HPP

#include <iostream>
#include <cmath>
#include <vector>

static int fastPowerMod(int base, int expo, int M) {
	int A = (expo & 1) == 1 ? base % M : 1, K = base % M;
	while (expo) {
		K = (K * K) % M;							//K����ʽ����
		A = (A * ((expo & 1) == 1 ? K : 1)) % M;	//A����ʽ����
		expo >>= 1;
	}
	return A;
}

///@brief Warshall �㷨�Ļع�
static void Warshall(const int size, std::vector<std::vector<int> > map) {
	printf("Algorithm started.Input:\n");
	for (int i = 0; i < size; ++i) {
		printf("[");
		for (int j = 0; j < size - 1; ++j) {
			printf("%d, ", map[i][j]);
		}
		printf("%d]\n", map[i][size - 1]);
	}
	printf("\n");
	for (int k = 0; k < size; ++k) {
		for (int i = 0; i < size; ++i) {
			for (int j = 0; j < size; ++j) {
				map[i][j] = map[i][j] | (map[i][k] & map[k][j]);
			}
		}
	}
	printf("Algorithm completed. Result:\n");
	for (int i = 0; i < size; ++i) {
		printf("[");
		for (int j = 0; j < size - 1; ++j) {
			printf("%d, ", map[i][j]);
		}
		printf("%d]\n", map[i][size - 1]);
	}
}



#endif //ALGO_HPP
