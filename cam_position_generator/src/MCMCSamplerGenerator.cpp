#include <opview/MCMCSamplerGenerator.hpp>


namespace opview {

    MCMCSamplerGenerator::MCMCSamplerGenerator()
    {
        randGen = gsl_rng_alloc(gsl_rng_mt19937); /*Define random number t*/
        gsl_rng_set(randGen, SEED); /*Initiate the random number generator with seed*/
        std = 1.0;    // standard deviation
    }

    MCMCSamplerGenerator::~MCMCSamplerGenerator()
    {
        gsl_rng_free(randGen);
    }

    GLMVec3List MCMCSamplerGenerator::getUniformSamples(DoublePairList limits, size_t qt)
    {
        GLMVec3List pointList;
        double stepx = (limits[0].second - limits[0].first) / qt;
        double stepy = (limits[1].second - limits[1].first) / qt;
        double stepz = (limits[2].second - limits[2].first) / qt;

        for (int i = 0; i < qt; i++) {
            pointList.push_back(GLMVec3(limits[0].first + i * stepx, limits[0].first + i * stepy, limits[0].first + i * stepz));
        }

        return pointList;
    }

    GLMVec3List MCMCSamplerGenerator::getWeightedSamples(GLMVec3List &centers, DoubleList &weights, size_t qt)
    {
        IntList sampleQt = computeWeightedSampleQuantity(weights, qt);

        GLMVec3List points;
        for (int p = 0; p < centers.size(); p++) {
            GLMVec3List newpoints = getSamples(centers[p], sampleQt[p]);
            points.insert(points.end(), newpoints.begin(), newpoints.end());
        }
        return points;
    }

    GLMVec3List MCMCSamplerGenerator::getSamples(GLMVec3 &center, int qt)
    {
        GLMVec3List points;
        for (int i = 0; i < qt; i++) {
            GLMVec3 point = getSample(center);
            points.push_back(point);
        }
        return points;
    }

    GLMVec3 MCMCSamplerGenerator::getSample(GLMVec3 &center)
    {
        gsl_vector *mus     = gsl_vector_alloc(SAMPLE_SIZE);
        gsl_matrix *var     = gsl_matrix_alloc(SAMPLE_SIZE, SAMPLE_SIZE);
        gsl_vector *results = gsl_vector_alloc(SAMPLE_SIZE);
        
        setupVarianceMatrix(var);
        setupMusVector(mus, center);
        
        randomMultivariateSample(mus, var, results);

        GLMVec3 point = GLMVec3(gsl_vector_get(results, 0), gsl_vector_get(results, 1), gsl_vector_get(results, 2));

        GSLVectorList vecs = {mus, results};
        GSLMatrixList mats = {var};
        freeDataStructure(vecs, mats);

        return point;
    }


    void MCMCSamplerGenerator::randomMultivariateSample(const gsl_vector *mus, const gsl_matrix *variances, gsl_vector *results)
    {
        /* multivariate normal distribution random number generator */
        /*
         *  mus           vector of means of size n
         *  variances     variance matrix of dimension n x n
         *  results       output variable with a sigle random vector normal distribution generation
         */

        gsl_matrix *L = gsl_matrix_alloc(SAMPLE_SIZE, SAMPLE_SIZE);
     
        gsl_matrix_memcpy(L, variances);
        gsl_linalg_cholesky_decomp(L);
     
        for(int k = 0; k < SAMPLE_SIZE; k++) {
            gsl_vector_set(results, k, gsl_ran_ugaussian(randGen));
        }
     
        gsl_blas_dtrmv(CblasLower, CblasNoTrans, CblasNonUnit, L, results);
        gsl_vector_add(results, mus);
     
        gsl_matrix_free(L);
    }

    void MCMCSamplerGenerator::setupVarianceMatrix(gsl_matrix *var) // FIXME change sigma depending on something? No but maybe set a different one
    {
        //set up variance matrix
        gsl_matrix_set(var, 0, 0, pow(std,2));
        gsl_matrix_set(var, 0, 1, 0);
        gsl_matrix_set(var, 0, 2, 0);
        gsl_matrix_set(var, 1, 1, pow(std,2));
        gsl_matrix_set(var, 1, 0, 0);
        gsl_matrix_set(var, 1, 2, 0);
        gsl_matrix_set(var, 2, 2, pow(std,2));
        gsl_matrix_set(var, 2, 0, 0);
        gsl_matrix_set(var, 2, 1, 0);
    }

    void MCMCSamplerGenerator::setupMusVector(gsl_vector *mus, GLMVec3 &center)
    {
        //set up the mean vector
        for (int i = 0; i < 3; i++) {
            gsl_vector_set(mus, i, center[i]);
        }
    }

    void MCMCSamplerGenerator::freeDataStructure(GSLVectorList &vecs, GSLMatrixList &mats)
    {
        for (int i = 0; i < vecs.size(); i++) {
            gsl_vector_free(vecs[i]);
        }

        for (int i = 0; i < mats.size(); i++) {
            gsl_matrix_free(mats[i]);
        }
    }

    IntList MCMCSamplerGenerator::computeWeightedSampleQuantity(DoubleList &weights, size_t qt)
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
