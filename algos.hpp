#pragma once
#include <iostream>
#include <cmath>
#include <vector>

int fastPowerMod(int base, int expo, int M) {
	int A = (expo & 1) == 1 ? base % M : 1, K = base % M;
	while (expo) {
		K = (K * K) % M;							//K����ʽ����
		A = (A * ((expo & 1) == 1 ? K : 1)) % M;	//A����ʽ����
		expo >>= 1;
	}
	return A;
}
