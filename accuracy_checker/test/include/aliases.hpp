#include <chrono>
#include <locale>
#include <unistd.h>

typedef std::chrono::milliseconds millis;

inline
millis now()
{
    return std::chrono::duration_cast<millis>(std::chrono::system_clock::now().time_since_epoch());
}


