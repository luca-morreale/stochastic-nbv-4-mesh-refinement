#include <BasicEvaluator.hpp>

namespace cameval {

    BasicEvaluator::BasicEvaluator(std::string &groundTruthFilename, std::string &basicPoseFilename, 
                        std::string &baseImageFolder, std::string &intrinsicParams, std::string &outputFile)
    {
        this->groundTruthFilename = groundTruthFilename;              // file containing the g-t point cloud
        this->outputFile = outputFile;
        this->basicPoseFilename = basicPoseFilename;
        
        log("\nSet OpenMvg data");
        setOpenMVGData(baseImageFolder, intrinsicParams);

        log("\nSet regex for CloudCompare");
        setCloudCompareLogRegex();

        log("\nPopulating camera poses");
        setBasicPoseCamera();
    }

    BasicEvaluator::~BasicEvaluator()
    {
        delete mvgJsonHandler;
    }

    void BasicEvaluator::setOpenMVGData(std::string &baseImageFolder, std::string &intrinsicParams)
    {
        OpenMvgSysCall::baseImageFolder = baseImageFolder;            // name of folder containing the images of current cloud
        OpenMvgSysCall::intrinsicParams = intrinsicParams;            // file containing the intrinisc params in format ready for opengm
    }

    void BasicEvaluator::setCloudCompareLogRegex()
    {
        distanceRegex = std::regex("\\[(.*)\\] \\[ComputeDistances\\] Mean distance = (.*) / std deviation = (.*)");
    }

    void BasicEvaluator::setBasicPoseCamera()
    {
        PoseReader reader(basicPoseFilename);
        cameraPoses = reader.getPoses();
    }
    
    double BasicEvaluator::evaluatePose(std::string &imageName, std::string &basicFolder)
    {
        log("Getting pose");
        Pose pose = getPose(imageName);

        log("Getting image");
        std::string filename = getImage(imageName);
        if (filename.size() == 0) return DBL_MAX;

        std::string jsonFile = basicFolder + "/sfm_data.json";

        log("Move new image into images folder");
        moveImageIntoImagesFolder(filename);
        
        log("Append image to json");
        size_t imgId = appendImageToJson(jsonFile, filename);
        
        log("Set position camera into json");
        setPositionOfCameras(jsonFile, pose, imgId);

        log("Generate pair file for matches");
        std::string pairFile = generatePairFile(imgId);

        OpenMvgSysCall::extractMvgFeatures(jsonFile, pairFile);

        log("Compute Structure");
        std::string mvgFolder = computeStructure(jsonFile, imgId);

        std::string inputCloud = mvgFolder + "/sfm_data.ply";
        
        log("Compute Distance");
        std::string logFile = computeDistance(inputCloud, groundTruthFilename); 

        log("Extract distance from log");
        double distance = parseDistance(logFile);     
        
        log("Clean files");
        cleanFiles({filename, pairFile, mvgFolder, "images/"+filename, "matches/matches.f.bin"});

        return distance;
    }

    std::string BasicEvaluator::computeStructure(std::string &jsonFile, int imgId)
    {
        return OpenMvgSysCall::computeStructureFromPoses(jsonFile, imgId);
    }

    void BasicEvaluator::moveImageIntoImagesFolder(std::string &filename)
    {
        try {
            std::string imagesFolder = "images";
            FileHandler::moveFileInside(filename, imagesFolder);
        } catch (const boost::filesystem::filesystem_error &ex) {
            auto errorCode = ex.code();
            if (FileHandler::isSpaceSystemError(errorCode)) {
                std::cerr << "Out of Memory Exception" << std::endl;
                return DBL_MAX;
            }
        }
    }

    size_t BasicEvaluator::appendImageToJson(std::string &sfmFile, std::string &imageFile)
    {
        std::string prefix = sfmFile.substr(0, sfmFile.find_last_of("/"));

        size_t key = mvgJsonHandler->appendImage(prefix, imageFile);
        mvgJsonHandler->saveChanges();
        
        return key;
    }

    void BasicEvaluator::setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId) 
    {
        GLMMat3 rotation = rotationMatrix(pose.second);
        Pose realPose(pose.first, rotation);

        setPositionOfCameras(sfmFile, realPose, imgId);
    }

    void BasicEvaluator::setPositionOfCameras(std::string &sfmFile, Pose &pose, size_t imgId)
    {
        mvgJsonHandler->setCamPosition(imgId, pose.first, pose.second);
        mvgJsonHandler->saveChanges();
    }

    void BasicEvaluator::setDefaultCameraPoses()
    {
        for (int index = 0; index < cameraPoses.size(); index++) {
            mvgJsonHandler->setCamPosition(index, cameraPoses[index].first, cameraPoses[index].second);
        }

        defaultCamNumber = cameraPoses.size();
    }

    std::string BasicEvaluator::generatePairFile(size_t uniqueId)
    {
        std::string original = "matches_pairs.txt";
        std::string filename = "matches_pairs_" + std::to_string(uniqueId) + ".txt";
        FileHandler::copyFile(original, filename);

        std::ofstream out;
        out.open(filename, std::ofstream::out | std::ofstream::app);
        for (int i = 0; i < defaultCamNumber; i++) {
            out << i << " " << uniqueId << std::endl;
        }
        out.close();

        return filename;
    }

    std::string BasicEvaluator::computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
    {
        std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "/log_distance.txt";

        // NOTE no need to remove cameras because assumed main_Structure... already output a structure without cameras and in ascii

        std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + " -c2c_dist";
        execute(command);

        return logfilename;
    }

    double BasicEvaluator::parseDistance(std::string &logFile)
    {
        std::string content = readStringFromFile(logFile);
        
        std::string line;
        std::istringstream strstream(content);
           
        while (std::getline(strstream, line)) {
            line = trim(line);
            if (!std::regex_match(line.begin(), line.end(), distanceRegex)) {
                continue;
            }

            std::smatch sm;
            std::regex_match(line, sm, distanceRegex);

            // Index 0 is whole line
            // Index 1- ... matches
            double distance = std::stod(sm[2]);
            double std = std::stod(sm[3]);

            return distance;
        }
        std::cerr << "Pattern not found" << std::endl;
        return DBL_MAX;
    }

    void BasicEvaluator::cleanFiles(StringList files)
    {
        FileHandler::cleanAll(files);     // remove all folders created!
    }


    std::string BasicEvaluator::getOuputFile()
    {
        return this->outputFile;
    }

    std::string BasicEvaluator::getGroundTruthFilename()
    {
        return groundTruthFilename;
    }

    PoseList BasicEvaluator::getCameraPoses()
    {
        return cameraPoses;
    }

    void BasicEvaluator::appendToCameraPoses(Pose &camPose)
    {
        cameraPoses.push_back(camPose);   
    }

    OpenMvgJsonHandlerPtr BasicEvaluator::getMvgJsonHandler()
    {
        return mvgJsonHandler;
    }

    void BasicEvaluator::setMvgJsonHandler(OpenMvgJsonHandlerPtr mvgJsonHandler)
    {
        this->mvgJsonHandler = mvgJsonHandler;
    }

    int BasicEvaluator::execute(std::string command)
    {
        return system(command.c_str());
    }

    std::string BasicEvaluator::getImageFolder()
    {
        return baseImageFolder;
    }

} // namespace cameval
