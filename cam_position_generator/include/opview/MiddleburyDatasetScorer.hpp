#ifndef CAM_POSITION_GENERATOR_MIDDLEBURY_DATASET_SCORER_H
#define CAM_POSITION_GENERATOR_MIDDLEBURY_DATASET_SCORER_H

#include <opview/alias_definition.h>
#include <opview/type_definition.h>
#include <opview/MiddleburyDatasetReader.hpp>
#include <opview/AutonomousStochasticMethod.hpp>

namespace opview {

    class MiddleburyDatasetScorer : public AutonomousStochasticMethod {
    public:
        MiddleburyDatasetScorer(std::string &cameraPoses, MeshConfiguration &meshConfig, 
                                                    size_t maxPoints, double goalAngle=45, double dispersion=5);

        ~MiddleburyDatasetScorer();
        DoubleList estimateBestCameraPosition();
        std::string estimateView();
        StringDoubleMap evaluateCameras();

        void removeView(std::string &view);

    protected:
        void setCameras();
        void setTree();

        double computeScoreForCam(Camera &cam);
        double computeScore(Camera &cam, GLMVec3 &centroid, GLMVec3 &normVector);
        double logVonMisesWrapper(GLMVec4 &pose, GLMVec3 &centroid, GLMVec3 &normVector, VonMisesConfiguration &vonMisesConfig);
        double visibilityDistribution(Camera &cam, GLMVec3 &centroid, GLMVec3 &normalVector, TreePtr tree);
        double imageProjectionDistribution(Camera &cam, GLMVec3 &centroid, GLMVec3 &normalVector, TreePtr tree);
        double computeBDConstraint(GLMVec4 &pose, GLMVec3 &centroid, GLMVec3List &cams);
        double computeBDConstraint(GLMVec3 &pose, GLMVec3 &centroid, GLMVec3 &cam);

        bool isMeaningfulPose(Camera &cam, GLMVec3 &centroid, TreePtr tree);
        bool isOppositeView(Camera &cam, GLMVec3 &centroid);
        bool isIntersecting(Camera &cam, GLMVec3 &centroid, TreePtr tree);
        bool isMathemathicalError(Segment_intersection &intersection, Point &point);
        bool isPointInsideImage(Camera &cam, GLMVec3 &centroid);
        GLMVec2 getProjectedPoint(Camera &cam, GLMVec3 &centroid);

    private:
        std::string meshFile;
        std::string cameraPoses;

        StringList views;
        CameraList cameras;

        FormulationPtr formulation;

        VonMisesConfiguration vonMisesConfig;

    };

    typedef MiddleburyDatasetScorer* MiddleburyDatasetScorerPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MIDDLEBURY_DATASET_SCORER_H
