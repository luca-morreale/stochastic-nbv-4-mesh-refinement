#ifndef EVALUATION_CAMERA_POSITION_BASIC_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_BASIC_EVALUATOR_H_

#include <regex>

#include <boost/filesystem.hpp>

#include <aliases.h>
#include <FileHandler.hpp>
#include <InputReader.hpp>
#include <OpenMvgJsonHandler.hpp>
#include <OpenMvgSysCall.hpp>
#include <PoseReader.hpp>
#include <utilities.hpp>

namespace cameval {
    
    class BasicEvaluator {
    public:
        BasicEvaluator(std::string &groundTruthFilename, std::string &basicPoseFilename, std::string &baseImageFolder, 
                                                                    std::string &intrinsicParams, std::string &outputFile);
        ~BasicEvaluator();

        virtual double evaluatePose(std::string &imageName, std::string &basicFolder);
    
        std::string getOuputFile();
        std::string getGroundTruthFilename();
        PoseList getCameraPoses();
        OpenMvgJsonHandlerPtr getMvgJsonHandler();
        void setMvgJsonHandler(OpenMvgJsonHandlerPtr mvgJsonHandler);

        void appendToCameraPoses(Pose &camPose);

    protected:
        /**   Functions used during an evaluation step   **/
        virtual std::string getImage(std::string &imageName) = 0;
        virtual Pose getPose(std::string &imageName) = 0;
        virtual void moveImageIntoImagesFolder(std::string &filename);
        virtual size_t appendImageToJson(std::string &sfmFile, std::string &imageFile);
        virtual void setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId);
        virtual void setPositionOfCameras(std::string &sfmFile, Pose &pose, size_t imgId);
        virtual std::string generatePairFile(size_t uniqueId);
        virtual std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename);
        virtual double parseDistance(std::string &logFile);
        virtual void cleanFiles(StringList files);
        virtual std::string computeStructure(std::string &jsonFile, int imgId);

        /**   Set up functions used by constructor   **/
        void setOpenMVGData(std::string &baseImageFolder, std::string &intrinsicParams);
        void setCloudCompareLogRegex();
        void setBasicPoseCamera();

        /***   System call function   ***/
        int execute(std::string command);

        void setDefaultCameraPoses();

        std::string getImageFolder();
        

    private:
        std::string groundTruthFilename;
        std::string baseImageFolder;
        std::string intrinsicParams;
        std::string basicPoseFilename;
        std::string outputFile;

        std::regex distanceRegex;

        OpenMvgJsonHandlerPtr mvgJsonHandler;
        PoseList cameraPoses;
        size_t defaultCamNumber;

    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_BASIC_EVALUATOR_H_
