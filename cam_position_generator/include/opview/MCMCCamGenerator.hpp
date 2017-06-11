#ifndef CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H

#include <glm/gtx/norm.hpp>

#include <opview/type_definition.h>
#include <opview/alias_definition.h>
#include <opview/MCMCSamplerGenerator.hpp>
#include <opview/OrientationHierarchicalGraphicalModel.hpp>
#include <opview/MultiBruteForceSolverGenerator.hpp>

namespace opview {

    #define OFFSPRING 0.1   // 10% of the total

    class MCMCCamGenerator : public OrientationHierarchicalGraphicalModel {
    public:
        MCMCCamGenerator(MultiBruteForceSolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, MCConfiguration &mcConfig, double goalAngle=55, double dispersion=5);
        ~MCMCCamGenerator();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

        virtual OrderedStates extractBestResults(AdderInferencePtr algorithm);

        VonMisesConfigurationPtr vonMisesConfiguration();
        void setVonMisesConfiguration(VonMisesConfiguration vonMisesConfig);

    protected:
            virtual void initLambdas();
            virtual DoubleIntMapList getPointMapping(GLMVec3List &points);
            
            virtual void generalStep(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector, OrderedStates &currentOptima, LambdaGLMPointsList &getPoints);
            virtual OrderedStates uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector);
            virtual OrderedStates resamplingMCStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedStates &currentOptima);
            
            virtual void fillObjectiveFunction(GMExplicitFunction &objFunction, GLMVec3 &centroid, GLMVec3 &normVector, GLMVec3List &points, DoubleIntMapList &mapping);
            virtual void computeDistributionForList(GMSparseFunctionList &modelFunctions, BoostObjFunctionList &evals, GLMVec3 &centroids, GLMVec3 &normVectors, GLMVec3List &points, DoubleIntMapList &mapping);
            virtual void fillConstraintFunction(GMSparseFunction &constraint, GLMVec3 &centroid, GLMVec3List &points, DoubleIntMapList &mapping);
            
            virtual GLMVec3List getCentersFromOptima(OrderedStates currentOptima);
            virtual DoubleList getWeightsFromOptima(OrderedStates currentOptima);

            LambdaGLMPointsList uniformPointGetter;
            LambdaGLMPointsList resamplingPointGetter;

    private:
        MCMCSamplerGeneratorPtr sampler;
        MCConfiguration mcConfig;


    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
