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

    GLMVec3List GaussianSampleGenerator::getUniformSamples(DoublePairList limits, size_t qt)
    {
        GLMVec3List pointList;
        double stepx = std::fabs(limits[0].second - limits[0].first) / qt;
        double stepy = std::fabs(limits[1].second - limits[1].first) / qt;
        double stepz = std::fabs(limits[2].second - limits[2].first) / qt;

        #pragma omp parallel for
        for (int i = 0; i < qt; i++) {
            GLMVec3 point = GLMVec3(limits[0].first + i * stepx, limits[1].first + i * stepy, limits[2].first + i * stepz);
            #pragma omp critical
            pointList.push_back(point);
        }

        return pointList;
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
        freeDataStructure(vecs, mats);

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
        gsl_vector *mus     = gsl_vector_alloc(ORIENTATION_SIZE);
        gsl_matrix *var     = gsl_matrix_alloc(ORIENTATION_SIZE, ORIENTATION_SIZE);
        gsl_vector *results = gsl_vector_alloc(ORIENTATION_SIZE);
        
        setupOrientationVarianceMatrix(var, ORIENTATION_SIZE);
        setupMusVector(mus, center);
        
        randomMultivariateSample(mus, var, results, ORIENTATION_SIZE);

        EigVector5 point;
        point << gsl_vector_get(results, 0), gsl_vector_get(results, 1), gsl_vector_get(results, 2), 
                    gsl_vector_get(results, 3), gsl_vector_get(results, 4);

        GSLVectorList vecs = {mus, results};
        GSLMatrixList mats = {var};
        freeDataStructure(vecs, mats);

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

    // FIXME change sigma depending on something? No but maybe set a different one
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
        setupCoordinatesVarianceMatrix(var, size);

        for (size_t i = COORDINATE_SIZE; i < ORIENTATION_SIZE; i++) {
            gsl_matrix_set(var, i, i, std::pow(orientStd, 2));
        }
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
        for (int i = 0; i < 5; i++) {
            gsl_vector_set(mus, i, center[i]);
        }
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

        IntList sampleQt;
        double sum = 0.0;

        for (int i = 0; i < weights.size(); i++) {
            weights[i] -= minWeight;
            sum += weights[i];
        }

        for (int i = 0; i < weights.size(); i++) {
            sampleQt.push_back(ceil(qt * weights[i] / sum));
        }

        return sampleQt;
    }

} // namespace opview
