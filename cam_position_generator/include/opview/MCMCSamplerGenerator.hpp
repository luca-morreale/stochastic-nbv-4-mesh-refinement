#ifndef CAM_POSITION_GENERATOR_MCMC_SAMPLER_GENERATOR_H
#define CAM_POSITION_GENERATOR_MCMC_SAMPLER_GENERATOR_H

#include <math.h>
#include <time.h>

#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include <opview/type_definition.h>
#include <opview/alias_definition.h>


namespace opview {

    #define SAMPLE_SIZE 3

    class MCMCSamplerGenerator {
    public:
        MCMCSamplerGenerator();
        ~MCMCSamplerGenerator();

        virtual GLMVec3List getUniformSamples(DoublePairList limits, size_t qt=10);

        virtual GLMVec3List getWeightedSamples(GLMVec3List &centers, DoubleList &weights, size_t qt=100);
        virtual GLMVec3List getSamples(GLMVec3 &centers, int qt=100);
        virtual GLMVec3 getSample(GLMVec3 &center);
        

    protected:
        virtual void randomMultivariateSample(const gsl_vector *mus, const gsl_matrix *variances, gsl_vector *results);
        virtual void setupVarianceMatrix(gsl_matrix *var);
        virtual void setupMusVector(gsl_vector *mus, GLMVec3 &center);
        virtual void freeDataStructure(GSLVectorList &vecs, GSLMatrixList &mats);

    private:
        const gsl_rng *randGen;
        double std;
        const long SEED = time(NULL);


        IntList computeWeightedSampleQuantity(DoubleList &weights, size_t qt);


    };

    typedef MCMCSamplerGenerator* MCMCSamplerGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MCMC_SAMPLER_GENERATOR_H
