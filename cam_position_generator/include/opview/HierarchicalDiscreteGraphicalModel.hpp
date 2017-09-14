#ifndef CAM_POSITION_HIERARCHICAL_DISCRETE_GRAPHICAL_MODEL_H
#define CAM_POSITION_HIERARCHICAL_DISCRETE_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/norm.hpp>

#include <opview/BasicGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {
    
    #define ORIGINAL_SIDE_SIZE 1.0
    #define MIN_COORDINATE -1.0
    
    class HierarchicalDiscreteGraphicalModel : public BasicGraphicalModel {
    public:
        HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, HierarchicalDiscretizationConfiguration &config, 
                                                        GLMVec3List &cams, double goalAngle=45, double dispersion=8);
        ~HierarchicalDiscreteGraphicalModel();

        virtual LabelList estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector) override;

        virtual size_t setNumLabels(size_t labels);
        virtual size_t numLabels() override;

        size_t getDepth();
        void setDepth(size_t depth);

    protected:

        virtual LabelList extractResults(AdderInferencePtr algorithm) override;
        virtual VarIndexList getOptimaForDiscreteSpace(LabelList &currentOptima);

        virtual void reduceScale(LabelList &currentOptimal);
        virtual void resetPosition();

        virtual int getXScalingFactor(float currentOptima, float halfNextSize, float scale);
        virtual int getYScalingFactor(float currentOptima, float halfNextSize, float scale);

        GLMVec3 lowerBounds();
        GLMVec3 upperBounds();

    private:
        HierarchicalDiscretizationConfiguration config;

    };


} // namespace opview

#endif // CAM_POSITION_HIERARCHICAL_DISCRETE_GRAPHICAL_MODEL_H
