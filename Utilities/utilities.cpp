#include <random>

#include "utilities.h"
#include "..\config.h"

int Utilities::GetRandomInt(int min, int max) {
    thread_local std::random_device rd;
    thread_local std::mt19937 gen(rd());

    std::uniform_int_distribution<int> distr(min, max);
    return distr(gen);
}

double Utilities::AddRangeToNoise(double range, int tier) {
    thread_local std::random_device rd;
    thread_local std::mt19937 gen(rd());
    thread_local std::normal_distribution<double> range_accuracy(0.0, 1.0);

    return range + (range_accuracy(gen) * range * SENSOR_MODEL_ACCURACY[tier].first);
}