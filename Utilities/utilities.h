#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <random>

class Utilities {
public:
    static int GetRandomInt(int min, int max);
    static double AddRangeToNoise(double range, int tier);

private:
    Utilities() = default;
};

#endif