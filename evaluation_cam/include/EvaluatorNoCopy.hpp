#ifndef EVALUATION_CAMERA_POSITION_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_EVALUATOR_H_

#include <regex>

#include <boost/filesystem.hpp>

#include <boost/algorithm/string.hpp>

#include <aliases.h>
#include <OpenMvgJsonHandler.hpp>
#include <SshHandler.hpp>
#include <utilities.hpp>
#include <SourceDirectoryDoesNotExistsException.hpp>
#include <UnableToCreateDirectoryException.hpp>

namespace cameval {
    
    class EvaluatorNoCopy {
    public:
        EvaluatorNoCopy(std::string &databaseFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                                        std::string &baseImageFolder, std::string &intrinsicParams, std::string &sshconfig);
        ~EvaluatorNoCopy();

        virtual void evaluate();

    protected:
        virtual std::string initOpenMvg();
        virtual double evaluatePose(std::string &file, std::string &basicFolder);
        virtual void setDefaultCameraPoses();
        virtual std::string imageListing();
        virtual std::string generatePairFile(size_t uniqueId);
        virtual void extractMvgFeatures(std::string &jsonFile, std::string &pairFile, size_t uniqueId);
        virtual size_t appendImageToJson(std::string &sfmFile, std::string &imageFile);
        virtual void setPositionOfCameras(std::string &sfmFile, EigVector6 &pose, size_t imgId);
        virtual std::string computeStructureFromPoses(std::string &jsonFile, std::string &pairFile, size_t uniqueId);
        virtual std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename);
        virtual double parseDistance(std::string &logFile);
        virtual void copyDataToPrivateFolder(std::string &basicFolder, std::string &privateFolder, std::string &filePath);
        virtual void createDestinationFolder(boost::filesystem::path &destination);
        virtual void checkSourceExistence(boost::filesystem::path &source);
        virtual void copyDirectory(boost::filesystem::path &source, boost::filesystem::path &destination);
        virtual void copyAllFilesFromTo(boost::filesystem::path &source, boost::filesystem::path &destination);
        virtual void copyFileInside(std::string sourceString, boost::filesystem::path &dest);
        virtual void copyFileInside(boost::filesystem::path &source, boost::filesystem::path &dest);
        virtual void cleanAll(StringList folders);
        virtual bool isSpaceSystemError(BoostSystemErrorCode errorCode);
        virtual void copyFileInside(std::string sourceString, std::string &destString);
        virtual void generateBasicPairFile();
        virtual EigVector6 extractPose(std::string &filename);
        virtual EigVector6 parseEntry(std::string &entry);

        virtual void moveFileInside(std::string sourceString, std::string &destString);
        virtual void moveFileInside(std::string sourceString, boost::filesystem::path &dest);
        virtual void moveFileInside(boost::filesystem::path &source, boost::filesystem::path &dest);

    private:
        std::string groundTruthFilename;
        std::string databaseFilename;
        std::string baseImageFolder;
        std::string intrinsicParams;
        std::string basicPoseFilename;

        std::string filePrefix;
        std::string fileExtention;

        PoseList poses;
        CameraPoseList basicPoses;
        StringList database;

        SshHandlerPtr sshHandler;

        std::regex distanceRegex;

        OpenMvgJsonHandlerPtr mvgJsonHandler;
        size_t defaultCamNumber;

        CameraPoseList readCameraPoses(std::string &basicPoseFilename);
        PoseList readPoints(std::string &pointsFilename);
        StringList readDatabase(std::string &database);
        int getIndexOfSmallestDistance(DoubleList &distances);
        std::string readStringFromFile(std::string &filename);
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_H_
