#include <opview/OrientationHierarchicalGraphicalModel.hpp>

namespace opview {

    OrientationHierarchicalGraphicalModel::OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, size_t depth, size_t labels,
                                                                        GLMVec3List &cams, double goalAngle, double dispersion)
                                                                        : HierarchicalDiscreteGraphicalModel(solver, depth, labels, cams, goalAngle, dispersion)
    { /*    */ }

    OrientationHierarchicalGraphicalModel::~OrientationHierarchicalGraphicalModel()
    { /*    */ }


    void OrientationHierarchicalGraphicalModel::fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
            orientationcycles(0, numLabels(), 0, numLabels()) { //for(int ptc = P0; ptc < Pn; ptc++) for(int yaw = YAW0; yaw < YAWn; yaw++)
                GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));
                EigVector5 pose;
                pose << scaledPos.x, scaledPos.y, scaledPos.z, deg2rad(ptc), deg2rad(yaw);
                
                LabelType val = computeObjectiveFunction(pose, centroid, normVector);

                #pragma omp critical
                vonMises(pose[0], pose[1], pose[2], pose[3], pose[4]) = val;
            }
        }
    }

    LabelType OrientationHierarchicalGraphicalModel::computeObjectiveFunction(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);
        return logVonMises(point, centroid, normalVector);
    }


    size_t OrientationHierarchicalGraphicalModel::numVariables()
    {
        return ORIENTATION_VARS;
    }

    float OrientationHierarchicalGraphicalModel::deg2rad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    float OrientationHierarchicalGraphicalModel::rad2deg(float rad)
    {
        return rad * 180.0 / M_PI;
    }
    //

} // namespace opview
