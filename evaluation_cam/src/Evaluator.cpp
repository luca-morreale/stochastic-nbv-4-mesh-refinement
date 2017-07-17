#include <Evaluator.hpp>

namespace cameval {

    Evaluator::Evaluator(std::string &databaseFilename, std::string &groundTruthFilename, 
            std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, 
            std::string &sshconfig)
    {
        this->databaseFilename = databaseFilename;                  // file from which read poses to evaluate
        this->groundTruthFilename = groundTruthFilename;            // file containing the g-t cloud of points
        OpenMvgSysCall::baseImageFolder = baseImageFolder;                    // name of folder containing the images of current cloud
        OpenMvgSysCall::intrinsicParams = intrinsicParams;                    // file containing the intrinisc params in format ready for opengm
        this->distanceRegex = std::regex("\\[(.*)\\] \\[ComputeDistances\\] Mean distance = (.*) / std deviation = (.*)");


        log("\nPopulating basic camera poses");
        PoseReader reader(basicPoseFilename);
        this->basicPoses = reader.getPoses();
        OpenMvgPoseConverter::convertPoses(this->basicPoses);
        log("\nBasic poses size: " + std::to_string(basicPoses.size()));

        log("\nPopulating Database");
        this->database = InputReader::readDatabase(databaseFilename);
        log("\nDatabase size: " + std::to_string(database.size()));

        this->filePrefix = database[0].substr(0, database[0].find_first_of("_"));
        this->fileExtention = database[0].substr(database[0].find_last_of(".") + 1, database[0].size());

        this->sshHandler = new SshHandler(sshconfig);
    }

    Evaluator::~Evaluator()
    {
        delete sshHandler;
        delete mvgJsonHandler;
    }

    void Evaluator::evaluate()
    {
        std::ofstream out("report.txt");
        std::string basicFolder = initOpenMvg();
        log("\nInitialized folders");

        DoubleList distances;
        distances.assign(database.size(), 0.0);


        for (int i = 2; i < database.size(); ++i) {
            log("\nStart analysis pose #" + std::to_string(i));
            double tmp = evaluatePose(database[i], basicFolder);
            distances[i] = tmp;
            out << database[i] << " " << distances[i] << std::endl;
            log("\nDone analysis pose #" + std::to_string(i));
        }

        log("\nDone computation of all poses");
        
        int index = getIndexOfSmallestDistance(distances);
        std::cout << "min distance: " << distances[index] << std::endl;
        std::cout << "pose: " << database[index] << std::endl;

        out << "min distance: " << distances[index] << std::endl;
        out << "pose: " << database[index] << std::endl;

        out.close();

        FileHandler::cleanAll({basicFolder, "matches/", "poses_sfm_data/"});     // remove all folders created!
    }

    std::string Evaluator::initOpenMvg()
    {
        OpenMvgSysCall::initOpenMvg();

        mvgJsonHandler = new OpenMvgJsonHandler("matches/sfm_data.json");
        log("\nSet default camera poses");
        setDefaultCameraPoses();
        mvgJsonHandler->saveChanges();

        return "matches";
    }
    
    double Evaluator::evaluatePose(std::string &file, std::string &basicFolder)
    {
        log("\nParsing filename to pose");
        AnglePose pose = extractPose(file);
        log("\nDownloading image");
        std::string filename = sshHandler->download(file);

        std::string jsonFile = basicFolder + "/sfm_data.json";

        log("\nMove file into folder");
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
        
        FileHandler::cleanAll({filename, pairFile, mvgFolder, "images/"+filename, "out_Incremental_Reconstruction"});     // remove all folders created!

        return distance;
    }

    void Evaluator::setDefaultCameraPoses()
    {
        for (int index = 0; index < basicPoses.size(); index++) {
            mvgJsonHandler->setCamPosition(index, basicPoses[index].first, basicPoses[index].second);
        }
        defaultCamNumber = basicPoses.size();
    }

    void Evaluator::generateBasicPairFile()
    {
        std::string pairFile = "matches_pairs.txt";
        if (!FileHandler::checkSourceExistence(pairFile)) {
            std::ofstream out("matches_pairs.txt");
            for (int i = 0; i < basicPoses.size() - 1; i++) {
                for (int j = i + 1; j < basicPoses.size(); j++) {
                    out << i << " " << j << std::endl;
                }
            }
            out.close();
        }
    }
    
    std::string Evaluator::generatePairFile(size_t uniqueId)
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

    size_t Evaluator::appendImageToJson(std::string &sfmFile, std::string &imageFile)
    {
        std::string prefix = sfmFile.substr(0, sfmFile.find_last_of("/"));

        size_t key;
        #pragma omp critical
        key = mvgJsonHandler->appendImage(prefix, imageFile);
        #pragma omp critical
        mvgJsonHandler->saveChanges();

        return key;
    }

    void Evaluator::setPositionOfCameras(std::string &sfmFile, AnglePose &pose, size_t imgId) 
    {
        GLMMat3 rotation = rotationMatrix(pose.second);
        #pragma omp critical
        mvgJsonHandler->setCamPosition(imgId, pose.first, rotation);
        #pragma omp critical
        mvgJsonHandler->saveChanges();
    }

    std::string Evaluator::computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
    {
        std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "/log_distance.txt";

        // assume definition of alias for CloudCompare -> 
        std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + " -c2c_dist";
        system(command.c_str());

        return logfilename;
    }

    double Evaluator::parseDistance(std::string &logFile)
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
    
    AnglePose Evaluator::extractPose(std::string &filename)
    {
        std::string path = filename.substr(0, filename.find_last_of("/") + 1);
        std::string name = filename.substr(filename.find_last_of("/") + 1, filename.size());

        return parseEntry(name);
    }

    int Evaluator::getIndexOfSmallestDistance(DoubleList &distances)
    {
        int index = 0;
        for (int i = 0; i < distances.size(); i++) {
            if (distances[i] < distances[index]) {
                index = i;
            }
        }
        return index;
    }


} // namespace cameval
