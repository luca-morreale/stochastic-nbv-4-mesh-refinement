#ifndef EVALUATION_CAMERA_POSITION_MIDDLEBURY_DATASET_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_MIDDLEBURY_DATASET_EVALUATOR_H_

#include <regex>
#include <sstream>

#include <aliases.h>
#include <SystemEvaluator.hpp>
#include <MiddleburyDatasetReader.hpp>
#include <utilities.hpp>

namespace cameval {

    class MiddleburyDatasetEvaluator : public SystemEvaluator {
    public:
        MiddleburyDatasetEvaluator(std::string &groundTruthFilename, std::string &poseFilename, 
                            std::string &baseImageFolder, std::string &basicPoses, std::string &inputImagesFolder, std::string &reportFile);
        ~MiddleburyDatasetEvaluator();

        virtual void datasetEvaluation(int steps, std::string reconstructionExe, std::string accuracyExe);
    
    protected:
        virtual void readDatasetCameras();
        virtual Camera getCameraOfView(std::string &view);
        virtual std::string transformToLookat(std::string &fixedPosefile);
        virtual std::string readPose(std::string &poseFile);
        virtual Pose getPose(std::string &data);
        virtual std::string getImage(std::string &poseString);
        virtual void setBasicPoseCamera(std::string &basicPoses);

        virtual void readInitialPoses(std::string &basicPoses);

        virtual std::string computeStructure(std::string &jsonFile, int imgId);

        virtual std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename);

        //virtual void setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId);
        //virtual void setDefaultCameraPoses();

    private:
        std::string inputImagesFolder;
        std::string poseFilename;

        CameraList cameras;
        StringList views;
        CameraList datasetCameras;
        StringList datasetViews;

        typedef BasicEvaluator super;
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_MIDDLEBURY_DATASET_EVALUATOR_H_
