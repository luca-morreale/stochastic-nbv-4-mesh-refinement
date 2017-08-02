
#include <meshac/utilities.hpp>

namespace meshac {
    
    IntArrayList combination(int N, int K)
    {
        return combination(N, K, 0.0);
    }

    IntArrayList combination(int N, int K, double skipProbability)
    {
        return fixedSizeCombination(N, K, skipProbability, N);
    }

    IntArrayList fixedSizeCombination(int N, int K, double skipProbability, const int MAX_SIZE)
    {
        if (N < K) {
            return IntArrayList();
        }
        
        IntArrayList combo;

        std::string bitmask(K, 1); // K leading 1's
        bitmask.resize(N, 0); // N-K trailing 0's

        // should make it parallel but with sequential iteration on bitmask
        while (std::prev_permutation(bitmask.begin(), bitmask.end())) {
            if ((double)std::rand() < skipProbability * (double)RAND_MAX) {     // probabilistic selection
                continue;
            }

            IntList tmp;
            for (int i = 0; i < N; ++i) { // [0..N-1] integers
                if (bitmask[i]) {
                    tmp.push_back(i);
                }
            }

            combo.push_back(tmp);
            if(combo.size() > MAX_SIZE) {
                return combo;
            }
        }

        return combo;
    }

    // std::mt19937 = Mersenne Twister 19937 generator
    IntList FisherYatesShuffle(size_t reducedSize, size_t batchSize)
    {
        std::mt19937 gen(std::random_device{}());

        IntList permutation(reducedSize);
     
        for(size_t i = 0; i < batchSize; ++i) {
            std::uniform_int_distribution<> dis(0, i);
            size_t j = dis(gen);
            if(j < permutation.size()) {
                if(i < j) {
                    permutation[i] = permutation[j];
                }
                permutation[j] = i;
            }
        }
        return permutation;
    }

    void appendMatrixDiagonalToVector(EigMatrix &mat, DoubleList &list)
    {
        // vector<int> vec(mat.data(), mat.data() + mat.rows() * mat.cols())
        EigVector diagonal = mat.diagonal().transpose();
        auto it = diagonal.data();
        for (int i = 0; i < diagonal.size(); i++) {
            list.push_back(*(it+i));
        }
    }

    EigMatrix generateDiagonalMatrix(DoubleList &list)
    {
        EigVector tmp = EigVector::Map(list.data(), list.size());
        return tmp.asDiagonal();
    }

    EigMatrixList generateDiagonalMatrix(IntDoubleListMap &mapping)
    {
        EigMatrixList list;
        for (auto el : mapping) {
            list.push_back(generateDiagonalMatrix(el.second));
        }
        return list;
    }

    EigMatrix juxtaposeMatrixs(EigMatrixList &list)
    {
        int size = 0;
        for (auto el : list) {
            size += el.cols();
        }
        return juxtaposeMatrixs(list, size);
    }

    EigMatrix juxtaposeMatrixs(EigMatrixList &list, int size)
    {
        int position = 0;
        const int ROWS = list[0].rows();
        EigMatrix matrix(ROWS, size);
        
        for (auto el : list) {
            matrix.block(0, position, ROWS, el.cols()) = el;
            position += el.cols();
        }
        return matrix;
    }

    EigMatrixList juxtaposeMatrixs(IntEigMatrixListMap &mapping, EigMatrixList &matrixList)
    {
        EigMatrixList list;
        int i = 0;
        for (auto el : mapping) {
            EigMatrix mat = juxtaposeMatrixs(el.second, matrixList[i++].size());
            list.push_back(mat);
        }
        return list;
    }

    EigMatrix average(EigMatrixList &list)
    {
        if (list.size() == 0) {
            return EigZeros(1);
        }

        EigMatrix mat = std::accumulate(list.begin(), list.end(), EigZeros(list[0].rows(), list[0].cols()));
        return mat / list.size();
    }

    float rad2deg(float rad)
    {
        return rad * 180.0f / M_PI;
    }

    float deg2rad(float deg)
    {
        return deg * M_PI / 180.0f;
    }

} // namespace meshac
