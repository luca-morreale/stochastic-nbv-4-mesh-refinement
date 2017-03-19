
#include <meshac/utilities.hpp>

namespace meshac {
    
    IntArrayList combination(int N, int K)
    {
        IntArrayList combo;

        std::string bitmask(K, 1); // K leading 1's
        bitmask.resize(N, 0); // N-K trailing 0's

        #pragma omp parallel for
        for ( ;std::prev_permutation(bitmask.begin(), bitmask.end()); ) {
            IntList tmp;
            for (int i = 0; i < N; ++i) { // [0..N-1] integers
                if (bitmask[i]) {
                    tmp.push_back(i);
                }
            }

            #pragma omp critical
            combo.push_back(tmp);
        }

        return combo;
    }

} // namespace meshac