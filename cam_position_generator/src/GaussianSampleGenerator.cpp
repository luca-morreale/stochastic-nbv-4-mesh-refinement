#include <opview/GaussianSampleGenerator.hpp>


namespace opview {

    GaussianSampleGenerator::GaussianSampleGenerator()
    {
        randGen = gsl_rng_alloc(gsl_rng_mt19937); /*Define random number t*/
        gsl_rng_set(randGen, SEED); /*Initiate the random number generator with seed*/
        coordStd = 1.5;    // standard deviation of x, y, z
        orientStd = M_PI /3.0;  // standard deviation of roll, pitch, yaw
    }

    GaussianSampleGenerator::~GaussianSampleGenerator()
    {
        gsl_rng_free(randGen);
    }

    GLMVec3List GaussianSampleGenerator::extractUniformSamples(DoublePairList limits, size_t qt)
    {
        GLMVec3List pointList;
        double stepx = std::fabs(limits[0].second - limits[0].first) / qt;
        double stepy = std::fabs(limits[1].second - limits[1].first) / qt;
        double stepz = std::fabs(limits[2].second - limits[2].first) / qt;

        #pragma omp parallel for collapse(3)
        for (int x = 0; x < qt; x++) {
            for (int y = 0; y < qt; y++) {
                for (int z = 0; z < qt; z++) {
                    GLMVec3 point = GLMVec3(limits[0].first + x * stepx, limits[1].first + y * stepy, limits[2].first + z * stepz);
                    #pragma omp critical
                    pointList.push_back(point);
                }
            }
        }

        return pointList;
    }

    GLMVec3List GaussianSampleGenerator::getUniformSamples(DoublePairList limits, size_t qt)
    {
        return GaussianSampleGenerator::extractUniformSamples(limits, qt);
    }

    GLMVec3List GaussianSampleGenerator::getWeightedSamples(GLMVec3List &centers, DoubleList &weights, size_t qt)
    {
        IntList sampleQt = computeWeightedSampleQuantity(weights, qt);

        GLMVec3List points;
        for (int p = 0; p < centers.size(); p++) {
            GLMVec3List newpoints = getSamples(centers[p], sampleQt[p]);
            points.insert(points.end(), newpoints.begin(), newpoints.end());
        }
        return points;
    }

    EigVector5List GaussianSampleGenerator::getRandomSamples(EigVector5 &center, EigVector5 &vars, int qt)
    {
        EigVector5List points;
        for (int i = 0; i < qt; i++) {
            EigVector5 point = getRandomSample(center, vars);
            points.push_back(point);
        }
        return points;
    }

    EigVector5 GaussianSampleGenerator::getRandomSample(EigVector5 &center, EigVector5 &variances)
    {
        gsl_vector *mus     = gsl_vector_alloc(5);
        gsl_matrix *var     = gsl_matrix_alloc(5, 5);
        gsl_vector *results = gsl_vector_alloc(5);
        
        setupCoordinatesVarianceMatrix(var, variances);
        setupMusVector(mus, center);
        
        randomMultivariateSample(mus, var, results, 5);

        EigVector5 point;
        point << gsl_vector_get(results, 0), gsl_vector_get(results, 1), gsl_vector_get(results, 2), 
                        gsl_vector_get(results, 3), gsl_vector_get(results, 4);
        
        GSLVectorList vecs = {mus, results};
        GSLMatrixList mats = {var};
        GaussianSampleGenerator::freeDataStructure(vecs, mats);

        return point;
    }

    void GaussianSampleGenerator::setupCoordinatesVarianceMatrix(gsl_matrix *var, EigVector5 variances)
    {
        //set up variance matrix
        for (size_t i = 0; i < 5; i++) {
            for (size_t j = 0; j < 5; j++) {
                gsl_matrix_set(var, i, j, 0);
            }
        }

        for (size_t i = 0; i < 5; i++) {
            gsl_matrix_set(var, i, i, variances[i]);
        }
    }


    GLMVec3List GaussianSampleGenerator::getSamples(GLMVec3 &center, int qt)
    {
        GLMVec3List points;
        for (int i = 0; i < qt; i++) {
            GLMVec3 point = getSample(center);
            points.push_back(point);
        }
        return points;
    }

    GLMVec3 GaussianSampleGenerator::getSample(GLMVec3 &center)
    {
        gsl_vector *mus     = gsl_vector_alloc(COORDINATE_SIZE);
        gsl_matrix *var     = gsl_matrix_alloc(COORDINATE_SIZE, COORDINATE_SIZE);
        gsl_vector *results = gsl_vector_alloc(COORDINATE_SIZE);
        
        setupCoordinatesVarianceMatrix(var, COORDINATE_SIZE);
        setupMusVector(mus, center);
        
        randomMultivariateSample(mus, var, results, COORDINATE_SIZE);

        GLMVec3 point = GLMVec3(gsl_vector_get(results, 0), gsl_vector_get(results, 1), gsl_vector_get(results, 2));

        GSLVectorList vecs = {mus, results};
        GSLMatrixList mats = {var};
        GaussianSampleGenerator::freeDataStructure(vecs, mats);

        return point;
    }

    EigVector5List GaussianSampleGenerator::getWeightedSamples(EigVector5List &centers, DoubleList &weights, size_t qt)
    {
        IntList sampleQt = computeWeightedSampleQuantity(weights, qt);

        EigVector5List points;
        for (int p = 0; p < centers.size(); p++) {
            EigVector5List newpoints = getSamples(centers[p], sampleQt[p]);
            points.insert(points.end(), newpoints.begin(), newpoints.end());
        }
        return points;
    }

    EigVector5List GaussianSampleGenerator::getSamples(EigVector5 &center, int qt)
    {
        EigVector5List points;
        for (int i = 0; i < qt; i++) {
            EigVector5 point = getSample(center);
            points.push_back(point);
        }
        return points;
    }

    EigVector5 GaussianSampleGenerator::getSample(EigVector5 &center)
    {
        gsl_vector *mus     = gsl_vector_alloc(3);
        gsl_matrix *var     = gsl_matrix_alloc(3, 3);
        gsl_vector *results = gsl_vector_alloc(3);
        
        setupOrientationVarianceMatrix(var, 3);
        setupMusVector(mus, center);
        
        randomMultivariateSample(mus, var, results, 3);

        EigVector5 point;
        point << gsl_vector_get(results, 0), gsl_vector_get(results, 1), center(2), // z constant 
                    center(3), gsl_vector_get(results, 2);  // pitch constant

        GSLVectorList vecs = {mus, results};
        GSLMatrixList mats = {var};
        GaussianSampleGenerator::freeDataStructure(vecs, mats);

        return point;
    }

    void GaussianSampleGenerator::randomMultivariateSample(const gsl_vector *mus, const gsl_matrix *variances, gsl_vector *results, size_t size)
    {
        /* multivariate normal distribution random number generator */
        /*
         *  mus           vector of means of size n
         *  variances     variance matrix of dimension n x n
         *  results       output variable with a sigle random vector normal distribution generation
         */

        gsl_matrix *L = gsl_matrix_alloc(size, size);
     
        gsl_matrix_memcpy(L, variances);
        gsl_linalg_cholesky_decomp(L);
     
        for(int k = 0; k < size; k++) {
            gsl_vector_set(results, k, gsl_ran_ugaussian(randGen));
        }
     
        gsl_blas_dtrmv(CblasLower, CblasNoTrans, CblasNonUnit, L, results);
        //gsl_vector_add(results, mus);
     
        gsl_matrix_free(L);
    }

    // TODO change sigma depending on something? No but maybe set a different one
    void GaussianSampleGenerator::setupCoordinatesVarianceMatrix(gsl_matrix *var, size_t size)
    {
        //set up variance matrix
        for (size_t i = 0; i < size; i++) {
            for (size_t j = 0; j < size; j++) {
                gsl_matrix_set(var, i, j, 0);
            }
        }

        for (size_t i = 0; i < COORDINATE_SIZE; i++) {
            gsl_matrix_set(var, i, i, std::pow(coordStd, 2));
        }
    }

    void GaussianSampleGenerator::setupOrientationVarianceMatrix(gsl_matrix *var, size_t size)
    {
        setupCoordinatesVarianceMatrix(var, 3);

        gsl_matrix_set(var, 2, 2, std::pow(orientStd, 2));
    }

    void GaussianSampleGenerator::setupMusVector(gsl_vector *mus, GLMVec3 &center)
    {
        //set up the mean vector
        for (int i = 0; i < 3; i++) {
            gsl_vector_set(mus, i, center[i]);
        }
    }

    void GaussianSampleGenerator::setupMusVector(gsl_vector *mus, EigVector5 &center)
    {
        //set up the mean vector
        gsl_vector_set(mus, 0, center[0]);
        gsl_vector_set(mus, 1, center[1]);
        gsl_vector_set(mus, 2, center[4]);
    }

    void GaussianSampleGenerator::freeDataStructure(GSLVectorList &vecs, GSLMatrixList &mats)
    {
        for (int i = 0; i < vecs.size(); i++) {
            gsl_vector_free(vecs[i]);
        }

        for (int i = 0; i < mats.size(); i++) {
            gsl_matrix_free(mats[i]);
        }
    }

    IntList GaussianSampleGenerator::computeWeightedSampleQuantity(DoubleList &weights, size_t qt)
    {
        // if there is a negative value translate all up and then compute the weights
        double minWeight = *std::min_element(std::begin(weights), std::end(weights));

        IntList sampleQt(weights.size());
        double sum = 0.0;

        for (int i = 0; i < weights.size(); i++) {
            weights[i] -= minWeight;
            sum += weights[i];
        }

        #pragma omp parallel for
        for (int i = 0; i < weights.size(); i++) {
            sampleQt[i] = floor((double)qt * weights[i] / sum);
        }

        return sampleQt;
    }

} // namespace opview
