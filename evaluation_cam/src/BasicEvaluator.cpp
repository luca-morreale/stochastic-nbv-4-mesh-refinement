#include <BasicEvaluator.hpp>

namespace cameval {

    BasicEvaluator::BasicEvaluator(std::string &posesFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                        std::string &baseImageFolder, std::string &intrinsicParams, std::string &outputFile)
    {
        this->posesFilename = posesFilename;                          // file from which read poses to evaluate
        this->groundTruthFilename = groundTruthFilename;              // file containing the g-t cloud of points
        this->outputFile = outputFile;
        OpenMvgSysCall::baseImageFolder = baseImageFolder;            // name of folder containing the images of current cloud
        OpenMvgSysCall::intrinsicParams = intrinsicParams;            // file containing the intrinisc params in format ready for opengm
        this->distanceRegex = std::regex("\\[(.*)\\] \\[ComputeDistances\\] Mean distance = (.*) / std deviation = (.*)");

        log("\nPopulating basic camera poses");
        PoseReader reader(basicPoseFilename);
        this->cameraPoses = reader.getPoses();
        log("\nBasic poses size: " + std::to_string(cameraPoses.size()));

        log("\nPopulating Poses To be Analyzed");
        posesString = InputReader::readDatabase(posesFilename);
        remapListToQueue(posesString);
        log("\nPoses size: " + std::to_string(posesString.size()));

        std::string fileExample = poses.front().second;
        this->filePrefix = fileExample.substr(0, fileExample.find_first_of("_"));
        this->fileExtention = fileExample.substr(fileExample.find_last_of(".") + 1, fileExample.size());
    }

    BasicEvaluator::~BasicEvaluator()
    {
        delete mvgJsonHandler;
    }

    void BasicEvaluator::appendToPoses(IntStringPair &pair)
    {
        poses.push(pair);
    }

    void BasicEvaluator::remapListToQueue(StringList &dataList)
    {
        for (int i = 0; i < dataList.size(); i++) {
            this->poses.push(IntStringPair(i, dataList[i]));
        }
    }

    void BasicEvaluator::evaluate()
    {
        std::ofstream out(this->outputFile);
        std::string basicFolder = initOpenMvg();
        log("\nInitialized folders");

        DoubleList distances;
        distances.assign(poses.size(), 0.0);

        while (!poses.empty()) {
            IntStringPair entry = poses.front();
            poses.pop();

            log("\nStart analysis pose #" + std::to_string(entry.first));

            double tmp = evaluatePose(entry, basicFolder);
            distances[entry.first] = tmp;

            if (tmp != DBL_MAX) {
                out << entry.second << " " << distances[entry.first] << std::endl;
                out.flush();
            }

            log("\nDone analysis pose #" + std::to_string(entry.first));
        }

        log("\nDone computation of all poses");
        
        int index = getIndexOfSmallestDistance(distances);

        log("min distance: " + std::to_string(distances[index]));
        log("pose: " + posesString[index]);

        out << "min distance: " << distances[index] << std::endl;
        out << "pose: " << posesString[index] << std::endl;

        out.close();

        // FileHandler::cleanAll({basicFolder, "matches/", "poses_sfm_data/"});     // remove all folders created!
    }

    std::string BasicEvaluator::initOpenMvg()
    {
        OpenMvgSysCall::initOpenMvg();

        mvgJsonHandler = new OpenMvgJsonHandler("matches/sfm_data.json");
        log("\nSet default camera poses");
        setDefaultCameraPoses();
        mvgJsonHandler->saveChanges();

        return "matches";
    }
    
    double BasicEvaluator::evaluatePose(IntStringPair &entry, std::string &basicFolder)
    {
        log("\nParsing filename to pose");
        Pose pose = getPose(entry.second);

        log("\nDownloading image");
        std::string filename = getImage(entry);
        if (filename.size() == 0) return DBL_MAX;

        std::string jsonFile = basicFolder + "/sfm_data.json";

        log("\nMove file into folder");
        moveImageIntoImagesFolder(filename);
        
        log("\nAppend image");
        size_t imgId = appendImageToJson(jsonFile, filename);
        
        log("\nSet position camera");
        setPositionOfCameras(jsonFile, pose, imgId);

        std::string pairFile = generatePairFile(imgId);

        OpenMvgSysCall::extractMvgFeatures(jsonFile, pairFile);

        log("\nCompute Structure");
        std::string mvgFolder = OpenMvgSysCall::computeStructureFromPoses(jsonFile, imgId);

        std::string inputCloud = mvgFolder + "/sfm_data.ply";
        
        log("\nCompute Distance");
        std::string logFile = computeDistance(inputCloud, groundTruthFilename); 

        log("\nExtract distance from log");
        double distance = parseDistance(logFile);     
        
        FileHandler::cleanAll({filename, pairFile, mvgFolder, "images/"+filename, "matches/matches.f.bin"});     // remove all folders created!

        return distance;
    }

    void BasicEvaluator::setDefaultCameraPoses()
    {
        for (int index = 0; index < cameraPoses.size(); index++) {
            mvgJsonHandler->setCamPosition(index, cameraPoses[index].first, cameraPoses[index].second);
        }
        defaultCamNumber = cameraPoses.size();
    }

    size_t BasicEvaluator::appendImageToJson(std::string &sfmFile, std::string &imageFile)
    {
        std::string prefix = sfmFile.substr(0, sfmFile.find_last_of("/"));

        size_t key;
        #pragma omp critical
        key = mvgJsonHandler->appendImage(prefix, imageFile);
        #pragma omp critical
        mvgJsonHandler->saveChanges();

        return key;
    }

    void BasicEvaluator::setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId) 
    {
        GLMMat3 rotation = rotationMatrix(pose.second);
        #pragma omp critical
        mvgJsonHandler->setCamPosition(imgId, pose.first, rotation);
        #pragma omp critical
        mvgJsonHandler->saveChanges();
    }

    void BasicEvaluator::setPositionOfCameras(std::string &sfmFile, Pose &pose, size_t imgId)
    {
        #pragma omp critical
        mvgJsonHandler->setCamPosition(imgId, pose.first, pose.second);
        #pragma omp critical
        mvgJsonHandler->saveChanges();
    }

    std::string BasicEvaluator::computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
    {
        std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "/log_distance.txt";

        // NOTE no need to remove cameras because assumed main_Structure... already output a structure without cameras and in ascii

        std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + " -c2c_dist";
        system(command.c_str());

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

    int BasicEvaluator::getIndexOfSmallestDistance(DoubleList &distances)
    {
        int index = 0;
        for (int i = 0; i < distances.size(); i++) {
            if (distances[i] < distances[index]) {
                index = i;
            }
        }
        return index;
    }

    void BasicEvaluator::generateBasicPairFile()
    {
        std::string pairFile = "matches_pairs.txt";
        if (!FileHandler::checkSourceExistence(pairFile)) {
            std::ofstream out("matches_pairs.txt");
            for (int i = 0; i < cameraPoses.size() - 1; i++) {
                for (int j = i + 1; j < cameraPoses.size(); j++) {
                    out << i << " " << j << std::endl;
                }
            }
            out.close();
        }
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


} // namespace cameval
