#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <random>

class Utilities {
public:
    static int GetRandomInt(int min, int max);
    static double GetRandomNoise(double mean, double sigma);
    static double GetFixedNoise(double value);
private:
    Utilities() = default;
};

#endif