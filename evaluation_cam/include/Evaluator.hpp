#ifndef EVALUATION_CAMERA_POSITION_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_EVALUATOR_H_

#include <regex>

#include <boost/filesystem.hpp>

#include <aliases.h>
#include <FileHandler.hpp>
#include <InputReader.hpp>
#include <Mapper.hpp>
#include <OpenMvgJsonHandler.hpp>
#include <OpenMvgSysCall.hpp>
#include <PoseReader.hpp>
#include <SshHandler.hpp>
#include <utilities.hpp>

namespace cameval {

    #define SLEEP_SSH_ERROR 10000000    // 10 sec
    #define THRESHOLD_SSH 120000000    // 120 sec
    
    class Evaluator {
    public:
        Evaluator(std::string &databaseFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                                        std::string &baseImageFolder, std::string &intrinsicParams, std::string &sshconfig);
        ~Evaluator();

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
        virtual AnglePose extractPose(std::string &filename);
        virtual std::string robustDownload(IntStringPair &entry);

    private:
        std::string groundTruthFilename;
        std::string databaseFilename;
        std::string baseImageFolder;
        std::string intrinsicParams;
        std::string basicPoseFilename;

        std::string filePrefix;
        std::string fileExtention;

        PoseList basicPoses;
        QueueIntStringPair database;
        StringList stringDB;

        SshHandlerPtr sshHandler;

        MapperPtr mapper;

        std::regex distanceRegex;

        OpenMvgJsonHandlerPtr mvgJsonHandler;
        size_t defaultCamNumber;

        const std::chrono::milliseconds thresholdWaitSsh;

        int getIndexOfSmallestDistance(DoubleList &distances);
        void moveImageIntoImagesFolder(std::string &filename);
        void remapListToQueue(StringList &dataList);
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_H_
