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
        BasicEvaluator(std::string &posesFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                                        std::string &baseImageFolder, std::string &intrinsicParams, std::string &outputFile);
        ~BasicEvaluator();

        virtual void evaluate();

    protected:
        virtual std::string initOpenMvg();
        virtual double evaluatePose(IntStringPair &entry, std::string &basicFolder);
        virtual void setDefaultCameraPoses();
        virtual std::string generatePairFile(size_t uniqueId);
        virtual size_t appendImageToJson(std::string &sfmFile, std::string &imageFile);
        virtual void setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId);
        virtual void setPositionOfCameras(std::string &sfmFile, Pose &pose, size_t imgId);
        virtual std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename);
        virtual double parseDistance(std::string &logFile);
        virtual void generateBasicPairFile();

        virtual void appendToPoses(IntStringPair &pair);


        virtual std::string getImage(IntStringPair &entry) = 0;
        virtual Pose getPose(std::string &data) = 0;

    private:
        std::string groundTruthFilename;
        std::string posesFilename;
        std::string baseImageFolder;
        std::string intrinsicParams;
        std::string basicPoseFilename;
        std::string outputFile;

        std::string filePrefix;
        std::string fileExtention;

        PoseList cameraPoses;
        QueueIntStringPair poses;
        StringList posesString;

        std::regex distanceRegex;

        OpenMvgJsonHandlerPtr mvgJsonHandler;
        size_t defaultCamNumber;

        int getIndexOfSmallestDistance(DoubleList &distances);
        void moveImageIntoImagesFolder(std::string &filename);
        void remapListToQueue(StringList &dataList);
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_BASIC_EVALUATOR_H_
