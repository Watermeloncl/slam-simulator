#include <random>

#include "utilities.h"

int Utilities::GetRandomInt(int min, int max) {
    thread_local std::random_device rd;
    thread_local std::mt19937 gen(rd());

    std::uniform_int_distribution<int> distr(min, max);
    return distr(gen);
}