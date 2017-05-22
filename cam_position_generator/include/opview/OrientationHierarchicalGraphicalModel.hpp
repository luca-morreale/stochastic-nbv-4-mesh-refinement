#ifndef CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/norm.hpp>

#include <opview/type_definition.h>
#include <opview/HierarchicalDiscreteGraphicalModel.hpp>

namespace opview {

    #define orientationcycles(P0, Pn, YAW0, YAWn) for(int ptc = P0; ptc < Pn; ptc++) for(int yaw = YAW0; yaw < YAWn; yaw++)
    #define ORIENTATION_VARS 5
    
    class OrientationHierarchicalGraphicalModel : public HierarchicalDiscreteGraphicalModel {
    public:
        OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, size_t depth, size_t labels, GLMVec3List &cams, double goalAngle=45, double dispersion=8);
        ~OrientationHierarchicalGraphicalModel();

    protected:

        virtual void fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector) override;
        virtual LabelType computeObjectiveFunction(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

        virtual size_t numVariables() override;

        float deg2rad(float deg);
        float rad2deg(float rad);

    };


} // namespace opview

#endif // CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
