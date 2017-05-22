#ifndef CAM_POSITION_HIERARCHICAL_DISCRETE_GRAPHICAL_MODEL_H
#define CAM_POSITION_HIERARCHICAL_DISCRETE_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/norm.hpp>

#include <opview/type_definition.h>
#include <opview/BasicGraphicalModel.hpp>

namespace opview {


    
    #define DISCRETE_LABELS 20
    #define VARS 3
    #define ORIGINAL_SIDE_SIZE 2
    #define MIN_COORDINATE -1.0
    
    class HierarchicalDiscreteGraphicalModel : public BasicGraphicalModel {
    public:
        HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, size_t depth, GLMVec3List &cams, double goalAngle=45, double dispersion=8);
        ~HierarchicalDiscreteGraphicalModel();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector) override;

    protected:

        virtual void reduceScale(LabelList currentOptimal);

        void initShapes();

        virtual size_t numVariables() override;
        virtual size_t numLabels() override;

        void resetPosition();

    private:
        size_t depth;

        //double scale = 1.0;
        double minSquareX = MIN_COORDINATE;
        double minSquareY = MIN_COORDINATE;
        double minSquareZ = MIN_COORDINATE;

    };


} // namespace opview

#endif // CAM_POSITION_HIERARCHICAL_DISCRETE_GRAPHICAL_MODEL_H
