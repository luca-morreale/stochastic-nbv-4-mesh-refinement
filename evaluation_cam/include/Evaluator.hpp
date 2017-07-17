#ifndef EVALUATION_CAMERA_POSITION_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_EVALUATOR_H_

#include <regex>

#include <boost/filesystem.hpp>

#include <boost/algorithm/string.hpp>

#include <aliases.h>
#include <FileHandler.hpp>
#include <InputReader.hpp>
#include <OpenMvgJsonHandler.hpp>
#include <OpenMvgPoseConverter.hpp>
#include <PoseReader.hpp>
#include <OpenMvgSysCall.hpp>
#include <SshHandler.hpp>
#include <utilities.hpp>

namespace cameval {
    
    class Evaluator {
    public:
        Evaluator(std::string &databaseFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                                        std::string &baseImageFolder, std::string &intrinsicParams, std::string &sshconfig);
        ~Evaluator();

        virtual void evaluate();

    protected:
        virtual std::string initOpenMvg();
        virtual double evaluatePose(std::string &file, std::string &basicFolder);
        virtual void setDefaultCameraPoses();
        virtual std::string generatePairFile(size_t uniqueId);
        virtual size_t appendImageToJson(std::string &sfmFile, std::string &imageFile);
        virtual void setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId);
        virtual std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename);
        virtual double parseDistance(std::string &logFile);
        virtual void generateBasicPairFile();
        virtual AnglePose extractPose(std::string &filename);

    private:
        std::string groundTruthFilename;
        std::string databaseFilename;
        std::string baseImageFolder;
        std::string intrinsicParams;
        std::string basicPoseFilename;

        std::string filePrefix;
        std::string fileExtention;

        PoseList basicPoses;
        StringList database;

        SshHandlerPtr sshHandler;

        std::regex distanceRegex;

        OpenMvgJsonHandlerPtr mvgJsonHandler;
        size_t defaultCamNumber;

        int getIndexOfSmallestDistance(DoubleList &distances);
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_H_
