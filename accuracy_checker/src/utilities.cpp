
#include <meshac/utilities.hpp>

namespace meshac {
    
    IntArrayList combination(int N, int K)
    {
        return combination(N, K, 0.0);
    }

    IntArrayList combination(int N, int K, double skipProbability)
    {
        if (N < K) {
            return IntArrayList();
        }
        
        IntArrayList combo;

        std::string bitmask(K, 1); // K leading 1's
        bitmask.resize(N, 0); // N-K trailing 0's

        // should make it parallel but with sequential iteration on bitmask
        while (std::prev_permutation(bitmask.begin(), bitmask.end())) {
            if ((double)std::rand() < skipProbability * (double)RAND_MAX) {
                continue;
            }

            IntList tmp;
            for (int i = 0; i < N; ++i) { // [0..N-1] integers
                if (bitmask[i]) {
                    tmp.push_back(i);
                }
            }

            combo.push_back(tmp);
        }

        return combo;
    }

    // should add check to have taken 1 sample for each point at least
    IntArrayList subsample(IntArrayList samples, int sampleSize)
    {
        IntArrayList subsamples;
        IntList indexs = FisherYatesShuffle(sampleSize, samples.size());
        
        for (int i : indexs) {
            subsamples.push_back(samples[i]);
        }

        return subsamples;
    }

    IntArrayList subsample(IntArrayList samples)
    {
        return subsample(samples, samples.size() / 4);
    }

    // std::mt19937 = Mersenne Twister 19937 generator
    std::vector<int> FisherYatesShuffle(size_t reducedSize, size_t batchSize)
    {
        std::mt19937 gen(std::random_device{}());

        std::vector<int> permutation(reducedSize);
     
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
        auto it = mat.diagonal().data();
        for (int i = 0; i < mat.diagonal().size(); i++) {
            list.push_back(*(it+i));
        }
    }

    EigMatrix generateDiagonalMatrix(DoubleList &list)
    {
        EigVector tmp = EigVector::Map(list.data(), list.size());
        return tmp.asDiagonal();
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

} // namespace meshac
