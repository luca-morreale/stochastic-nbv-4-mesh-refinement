#ifndef MESH_ACCURACY_UTILITIES_H
#define MESH_ACCURACY_UTILITIES_H

#include <algorithm>
#include <random>

#include <meshac/alias_definition.hpp>

namespace meshac {

    /*
     * Computes combinations.
     */
    IntArrayList combination(int N, int K);

    /*
     * Computes combination probabilistically dropping some combination.
     */
    IntArrayList combination(int N, int K, double skipProbability);

    /*
     * Computes a fixed number of combination.
     */
    IntArrayList fixedSizeCombination(int N, int K, double skipProbability, const int MAX_SIZE);

    /* 
     * Fisher–Yates shuffle
     * Generate a random permutation of elements.
     */
    IntList FisherYatesShuffle(size_t sampleSize, size_t batchSize);
    
    // methods to make std::vector interacts with eigen::matrix
    void appendMatrixDiagonalToVector(EigMatrix &mat, DoubleList &list);
    EigMatrix generateDiagonalMatrix(DoubleList &list);
    EigMatrix juxtaposeMatrixs(EigMatrixList &list, int size);
    EigMatrix juxtaposeMatrixs(EigMatrixList &list);

    /*
     * Subsample the data given.
     */
    template<typename T>
    std::vector<T> subsample(std::vector<T> samples, int sampleSize)
    {
        std::vector<T> subsamples;
        IntList indexs = FisherYatesShuffle(sampleSize, samples.size());
        for (int i : indexs) {
            subsamples.push_back(samples[i]);
        }

        return subsamples;
    }

    template<typename T>
    std::vector<T> subsample(std::vector<T> samples)
    {
        return subsample(samples, samples.size() / 4);
    }

    /*
     * Subsample the data given.
     */
    template<typename T>
    std::set<T> subsample(std::set<T> samples, int sampleSize)
    {
        if (samples.size() <= sampleSize) return samples;
        std::vector<T> convertedSet(samples.begin(), samples.end());
        std::vector<T> subsampleVector = subsample(convertedSet, sampleSize);
        
        std::set<T> subsamples;
        //std::cout << "inserting \n";
        for (int i = 0; i < subsampleVector.size(); i++) {
            subsamples.insert(subsampleVector[i]);
        }
        //std::set<T> subsamples(std::make_move_iterator(subsampleVector.begin()), std::make_move_iterator(subsampleVector.end()));
        //std::cout << "set size " << subsamples.size() << std::endl;
        return subsamples;
    }

    template<typename T>
    std::set<T> subsample(std::set<T> &samples)
    {
        return subsample(samples, samples.size() / 4);
    }

} // namespace meshac

#endif // MESH_ACCURACY_UTILITIES_H
