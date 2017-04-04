#ifndef MESH_ACCURACY_UTILITIES_H
#define MESH_ACCURACY_UTILITIES_H

#include <algorithm>
#include <random>

#include <meshac/alias_definition.hpp>

namespace meshac {


    IntArrayList combination(int N, int K);
    IntArrayList combination(int N, int K, double skipProbability);

    IntArrayList subsample(IntArrayList samples, int sampleSize);
    IntArrayList subsample(IntArrayList samples);   // extract a quarter of the samples

    // Fisher–Yates_shuffle
    std::vector<int> FisherYatesShuffle(size_t sampleSize, size_t batchSize);
    
    // methods to make std::vector interacts with eigen::matrix
    void appendMatrixDiagonalToVector(EigMatrix &mat, DoubleList &list);
    EigMatrix generateDiagonalMatrix(DoubleList &list);
    EigMatrix juxtaposeMatrixs(EigMatrixList &list, int size);
    EigMatrix juxtaposeMatrixs(EigMatrixList &list);

} // namespace meshac

#endif // MESH_ACCURACY_UTILITIES_H
