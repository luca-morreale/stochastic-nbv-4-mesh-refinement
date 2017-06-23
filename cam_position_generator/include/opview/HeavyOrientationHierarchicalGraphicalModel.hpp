#ifndef CAM_POSITION_GENERATOR_ORIENTATION_HEAVY_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_ORIENTATION_HEAVY_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <opview/OrientationHierarchicalGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {
    
    class HeavyOrientationHierarchicalGraphicalModel : public OrientationHierarchicalGraphicalModel {
    public:
        HeavyOrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                    CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                                    GLMVec3List &cams, size_t spaceBlock=8, size_t offsetBlock=0, double goalAngle=55, double dispersion=5);
        ~HeavyOrientationHierarchicalGraphicalModel();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);
    protected:
        virtual size_t numLabels() override;
        virtual void changeBlockSettings(size_t blockNumber);

    private:
        size_t spaceBlock;
        size_t offsetBlock;

        size_t popcount(size_t n);

        typedef OrientationHierarchicalGraphicalModel super;
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_ORIENTATION_HEAVY_HIERARCHICAL_GRAPHICAL_MODEL_H
