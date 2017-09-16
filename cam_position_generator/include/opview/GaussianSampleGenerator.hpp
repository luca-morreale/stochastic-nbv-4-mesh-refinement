#ifndef CAM_POSITION_GENERATOR_GAUSSIAN_SAMPLE_GENERATOR_H
#define CAM_POSITION_GENERATOR_GAUSSIAN_SAMPLE_GENERATOR_H

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

    #define COORDINATE_SIZE 3
    #define ORIENTATION_SIZE 5

    class GaussianSampleGenerator {
    public:
        GaussianSampleGenerator();
        ~GaussianSampleGenerator();

        static GLMVec3List extractUniformSamples(DoublePairList limits, size_t qt=10);
        virtual GLMVec3List getUniformSamples(DoublePairList limits, size_t qt=10);

        virtual GLMVec3List getWeightedSamples(GLMVec3List &centers, DoubleList &weights, size_t qt=100);
        virtual GLMVec3List getSamples(GLMVec3 &centers, int qt=100);
        virtual GLMVec3 getSample(GLMVec3 &center);

        virtual EigVector5List getWeightedSamples(EigVector5List &centers, DoubleList &weights, size_t qt=1000);
        virtual EigVector5List getSamples(EigVector5 &center, int qt=100);
        virtual EigVector5 getSample(EigVector5 &center);
        

    protected:
        virtual void randomMultivariateSample(const gsl_vector *mus, const gsl_matrix *variances, gsl_vector *results, size_t size);
        virtual void setupCoordinatesVarianceMatrix(gsl_matrix *var, size_t size);
        virtual void setupOrientationVarianceMatrix(gsl_matrix *var, size_t size);
        virtual void setupMusVector(gsl_vector *mus, GLMVec3 &center);
        virtual void setupMusVector(gsl_vector *mus, EigVector5 &center);
        virtual void freeDataStructure(GSLVectorList &vecs, GSLMatrixList &mats);

    private:
        const gsl_rng *randGen;
        double coordStd;
        double orientStd;
        const long SEED = time(NULL);


        IntList computeWeightedSampleQuantity(DoubleList &weights, size_t qt);


    };

    typedef GaussianSampleGenerator* GaussianSampleGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_GAUSSIAN_SAMPLE_GENERATOR_H
