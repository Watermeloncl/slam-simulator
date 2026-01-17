#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <random>

class Utilities {
public:
    static int GetRandomInt(int min, int max);

private:
    Utilities() = default;
};

#endif