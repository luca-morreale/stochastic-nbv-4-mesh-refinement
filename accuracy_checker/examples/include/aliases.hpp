#ifndef ACCURACY_ALIASES_H_
#define ACCURACY_ALIASES_H_

#include <chrono>
#include <locale>
#include <vector>
#include <unistd.h>

typedef std::chrono::milliseconds millis;

typedef std::vector<double> DoubleList;

inline
millis now()
{
    return std::chrono::duration_cast<millis>(std::chrono::system_clock::now().time_since_epoch());
}

#endif // ACCURACY_ALIASES_H_
