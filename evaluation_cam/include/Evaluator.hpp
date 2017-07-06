#ifndef EVALUATION_CAMERA_POSITION_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_EVALUATOR_H_

#include <regex>

#include <boost/filesystem.hpp>

#include <aliases.h>
#include <OpenMvgJsonHandler.hpp>
#include <KDTree.hpp>
#include <SshHandler.hpp>
#include <utilities.hpp>
#include <SourceDirectoryDoesNotExistsException.hpp>
#include <UnableToCreateDirectoryException.hpp>

namespace cameval {
    
    class Evaluator {
    public:
        Evaluator(std::string &pointsFilename, std::string &groundTruthFilename, std::string &databaseFilename, 
                std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, std::string &sshconfig);
        ~Evaluator();

        virtual void evaluate();

    protected:
        virtual std::string initOpenMvg();
        virtual double evaluatePose(EigVector6 &pose, std::string &basicFolder, int uniqueId);
        virtual void setDefaultCameraPoses(std::string &jsonFile, OpenMvgJsonHandler &mvgJsonHandler);
        virtual std::string imageListing();
        virtual std::string extractMvgFeatures(std::string &jsonFile, int uniqueId);
        virtual void cleanAll(StringList folders);
        virtual std::string getClosestImage(EigVector6 &pose);
        virtual void appendImageToJson(std::string &sfmFile, std::string &imageFile, OpenMvgJsonHandler &mvgJsonHandler);
        virtual void setPositionOfCameras(std::string &sfmFile, EigVector6 &pose, OpenMvgJsonHandler &mvgJsonHandler);
        virtual std::string computeStructureFromPoses(std::string &jsonFile, std::string &matchesFolder, int uniqueId);
        virtual std::string computeDistance(std::string &alignedCloud, std::string &groundTruthFilename);
        virtual double parseDistance(std::string &logFile);

        virtual void copyDataToPrivateFolder(std::string &basicFolder, std::string &privateFolder, std::string &filePath);
        virtual void checkSourceExistence(boost::filesystem::path &source);
        virtual void createDestinationFolder(boost::filesystem::path &destination);
        virtual void copyDirectory(boost::filesystem::path &source, boost::filesystem::path &destination);
        virtual void copyAllFilesFromTo(boost::filesystem::path &source, boost::filesystem::path &destination);
        virtual void copyFileInside(std::string sourceString, boost::filesystem::path &dest);
        virtual void copyFileInside(boost::filesystem::path &source, boost::filesystem::path &dest);
        

    private:
        std::string pointsFilename;
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

        KDTReePtr tree;
        SshHandlerPtr sshHandler;

        std::regex distanceRegex;

        CameraPoseList readCameraPoses(std::string &basicPoseFilename);
        PoseList readPoints(std::string &pointsFilename);
        StringList readDatabase(std::string &database);
        int getIndexOfSmallestDistance(DoubleList &distances);
        std::string readStringFromFile(std::string &filename);
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_H_
