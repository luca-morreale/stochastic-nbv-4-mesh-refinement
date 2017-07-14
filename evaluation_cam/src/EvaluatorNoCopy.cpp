#include <EvaluatorNoCopy.hpp>

namespace cameval {

    EvaluatorNoCopy::EvaluatorNoCopy(std::string &databaseFilename, std::string &groundTruthFilename, 
            std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, 
            std::string &sshconfig)
    {
        this->databaseFilename = databaseFilename;                  // file from which read poses to evaluate
        this->groundTruthFilename = groundTruthFilename;            // file containing the g-t cloud of points
        this->baseImageFolder = baseImageFolder;                    // name of folder containing the images of current cloud
        this->intrinsicParams = intrinsicParams;                    // file containing the intrinisc params in format ready for opengm
        this->distanceRegex = std::regex("\\[(.*)\\] \\[ComputeDistances\\] Mean distance = (.*) / std deviation = (.*)");


        log("\nPopulating basic camera poses");
        this->basicPoses = readCameraPoses(basicPoseFilename);
        log("\nBasic poses size: " + std::to_string(basicPoses.size()));

        log("\nPopulating Database");
        this->database = readDatabase(databaseFilename);
        log("\nDatabase size: " + std::to_string(database.size()));

        this->filePrefix = database[0].substr(0, database[0].find_first_of("_"));
        this->fileExtention = database[0].substr(database[0].find_last_of(".") + 1, database[0].size());

        this->sshHandler = new SshHandler(sshconfig);
    }

    EvaluatorNoCopy::~EvaluatorNoCopy()
    {
        delete sshHandler;
        delete mvgJsonHandler;
    }

    void EvaluatorNoCopy::evaluate()
    {
        std::ofstream out("report.txt");
        std::string basicFolder = initOpenMvg();
        log("\nInitialized folders");

        DoubleList distances(database.size());

        #pragma omp parallel for
        for (int i = 0; i < database.size(); ++i) {
            log("\nStart analysis pose #" + std::to_string(i));
            double tmp = evaluatePose(database[i], basicFolder);
            distances[i] = tmp;
            out << database[i] << " " << distances[i] << std::endl;
            log("\nDone analysis pose #" + std::to_string(i));
        }

        log("\nDone computation of all poses");
        
        int index = getIndexOfSmallestDistance(distances);
        std::cout << "min distance: " << distances[index] << std::endl;
        std::cout << "pose: " << poses[index] << std::endl;

        out << "min distance: " << distances[index] << std::endl;
        out << "pose: " << poses[index] << std::endl;

        out.close();

        cleanAll({basicFolder, "matches/", "poses_sfm_data/"});     // remove all folders created!
    }

    std::string EvaluatorNoCopy::initOpenMvg()
    {
        log("\nImage listing");
        std::string jsonFile = imageListing();
        log(jsonFile);

        log("\nCompute Features");
        std::string command = "openMVG_main_ComputeFeatures -i " + jsonFile + " -o matches/ -m AKAZE_FLOAT";
        system(command.c_str());

        log("\nCompute matches");
        command = "openMVG_main_ComputeMatches -i " + jsonFile + " -o matches/";
        system(command.c_str());

        mvgJsonHandler = new OpenMvgJsonHandler("matches/sfm_data.json");
        log("\nSet default camera poses");
        setDefaultCameraPoses();
        mvgJsonHandler->saveChanges();

        return "matches";
    }
    
    double EvaluatorNoCopy::evaluatePose(std::string &file, std::string &basicFolder)
    {
        log("\nParsing filename to pose");
        EigVector6 pose = extractPose(file);
        log("\nDownloading image");
        std::string filename = sshHandler->download(file);

        std::string jsonFile = basicFolder + "/sfm_data.json";

        log("\nMove file into folder");
        try {
            std::string imagesFolder = "images";
            moveFileInside(filename, imagesFolder);
        } catch (const boost::filesystem::filesystem_error &ex) {
            auto errorCode = ex.code();
            if (isSpaceSystemError(errorCode)) {
                std::cerr << "Out of Memory Exception" << std::endl;
                return DBL_MAX;
            }
        }
        
        log("\nAppend image");
        size_t imgId = appendImageToJson(jsonFile, filename);
        
        log("\nSet position camera");
        setPositionOfCameras(jsonFile, pose, imgId);
        

        std::string pairFile = generatePairFile(imgId);

        log("\nExtract features");
        extractMvgFeatures(jsonFile, pairFile, imgId);
        log("\nCompute Structure");
        std::string mvgFolder = computeStructureFromPoses(jsonFile, pairFile, imgId);

        std::string inputCloud = mvgFolder + "/sfm_data.ply";
        log("\nCompute Distance");
        std::string logFile = computeDistance(inputCloud, groundTruthFilename); 

        log("\nExtract distance from log");
        double distance = parseDistance(logFile);     
        
        cleanAll({filename, pairFile, mvgFolder});     // remove all folders created!

        return distance;
    }

    void EvaluatorNoCopy::setDefaultCameraPoses()
    {
        for (auto pose : basicPoses) {
            // convert eigen to glm
            auto center = convert(pose.center);
            auto rotation = convert(pose.rotation);
            mvgJsonHandler->setCamPosition(pose.index, center, rotation);
        }
        defaultCamNumber = basicPoses.size();
    }

    std::string EvaluatorNoCopy::imageListing()
    {
        std::string command = "openMVG_main_SfMInit_ImageListing -i " + baseImageFolder + 
            " -d /usr/local/share/openMVG/sensor_width_camera_database.txt -o matches -k '" + intrinsicParams + "'";
        system(command.c_str());
        
        log("\nGenerate basic pair");
        generateBasicPairFile();
        return "matches/sfm_data.json"; // NOTE this can even not be unique because this has to be done just once!!
    }

    void EvaluatorNoCopy::generateBasicPairFile()
    {
        if (!boost::filesystem::exists("matches_pairs.txt")) {
            std::ofstream out("matches_pairs.txt");
            for (int i = 0; i < basicPoses.size() - 1; i++) {
                for (int j = i + 1; j < basicPoses.size(); j++) {
                    out << i << " " << j << std::endl;
                }
            }
            out.close();
        }
    }
    
    std::string EvaluatorNoCopy::generatePairFile(size_t uniqueId)
    {
        std::string filename = "matches_pairs_" + std::to_string(uniqueId) + ".txt";
        boost::filesystem::path source("matches_pairs.txt");
        boost::filesystem::path dest(filename);
        boost::filesystem::copy_file(source, dest, boost::filesystem::copy_option::overwrite_if_exists);

        std::ofstream out;
        out.open(filename, std::ofstream::out | std::ofstream::app);
        for (int i = 0; i < defaultCamNumber; i++) {
            out << i << " " << uniqueId << std::endl;
        }
        out.close();

        return filename;
    }

    void EvaluatorNoCopy::extractMvgFeatures(std::string &jsonFile, std::string &pairFile, size_t uniqueId) 
    {
        // std::string matchesFolder = "matches_" + std::to_string(uniqueId);
        // TODO run this as fork
        std::string command = "openMVG_main_ComputeFeatures -i " + jsonFile + " -o matches/ -m AKAZE_FLOAT";
        system(command.c_str());

        command = "openMVG_main_ComputeMatches -i " + jsonFile + " -o matches/ -l " + pairFile;
        system(command.c_str());
    }

    size_t EvaluatorNoCopy::appendImageToJson(std::string &sfmFile, std::string &imageFile)
    {
        std::string prefix = sfmFile.substr(0, sfmFile.find_last_of("/"));

        size_t key;
        #pragma omp critical
        key = mvgJsonHandler->appendImage(prefix, imageFile);
        #pragma omp critical
        mvgJsonHandler->saveChanges();

        return key;
    }

    void EvaluatorNoCopy::setPositionOfCameras(std::string &sfmFile, EigVector6 &pose, size_t imgId) 
    {
        CameraPose cam(imgId, pose[0], pose[1], pose[2], pose[5], pose[4], pose[3]);
        auto center = convert(cam.center);
        auto rotation = convert(cam.rotation);
        #pragma omp critical
        mvgJsonHandler->setCamPosition(imgId, center, rotation);
        #pragma omp critical
        mvgJsonHandler->saveChanges();
    }

    std::string EvaluatorNoCopy::computeStructureFromPoses(std::string &jsonFile, std::string &pairFile, size_t uniqueId) 
    {
        std::string outFolder = "poses_sfm_data_" + std::to_string(uniqueId);
        std::string command = "mkdir " + outFolder;
        system(command.c_str());
        command = "openMVG_main_ComputeStructureFromKnownPoses -i "+jsonFile+" -o "+outFolder+"/sfm_data.json -m matches/ -p " + pairFile;
        system(command.c_str());
        return outFolder;
    }

    std::string EvaluatorNoCopy::computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
    {
        std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "/log_distance.txt";

        // assume definition of alias for CloudCompare -> 
        std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + " -c2c_dist";
        system(command.c_str());

        return logfilename;
    }

    double EvaluatorNoCopy::parseDistance(std::string &logFile)
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

    std::string EvaluatorNoCopy::readStringFromFile(std::string &filename)
    {
        std::ifstream cin(filename);
        std::string content((std::istreambuf_iterator<char>(cin)), std::istreambuf_iterator<char>());
        cin.close();
        return content;
    }

    void EvaluatorNoCopy::copyDataToPrivateFolder(std::string &basicFolder, std::string &privateFolder, std::string &filePath)
    {
        boost::filesystem::path imagePath(filePath);
        boost::filesystem::path source(basicFolder);
        boost::filesystem::path destination(privateFolder);

        // Check whether the function call is valid
        copyDirectory(source, destination);
        copyFileInside(imagePath, destination);
        copyFileInside("sfm_data.json", destination);
    }

    void EvaluatorNoCopy::createDestinationFolder(boost::filesystem::path &destination)
    {
        
        if(boost::filesystem::exists(destination)) {
            std::cerr << "Destination directory " << destination.string() << " already exists." << std::endl;
        } else if(!boost::filesystem::create_directory(destination)) { // Create the destination directory
            std::cerr << "Unable to create destination directory" << destination.string() << std::endl;
            throw UnableToCreateDirectoryException(destination);
        }
    }

    void EvaluatorNoCopy::checkSourceExistence(boost::filesystem::path &source)
    {
        if(!boost::filesystem::exists(source) || !boost::filesystem::is_directory(source)) {
            throw SourceDirectoryDoesNotExistsException(source);
        }
    }

    void EvaluatorNoCopy::copyDirectory(boost::filesystem::path &source, boost::filesystem::path &destination)
    {
        checkSourceExistence(source);
        createDestinationFolder(destination);

        copyAllFilesFromTo(source, destination);
    }

    void EvaluatorNoCopy::copyAllFilesFromTo(boost::filesystem::path &source, boost::filesystem::path &destination)
    {
        // Iterate through the source directory
        for(boost::filesystem::directory_iterator file(source); file != boost::filesystem::directory_iterator(); ++file) {
            boost::filesystem::path current(file->path());
            copyFileInside(current, destination);
        }
    }

    void EvaluatorNoCopy::copyFileInside(std::string sourceString, std::string &destString)
    {
        boost::filesystem::path dest(destString);
        boost::filesystem::path source(sourceString);
        copyFileInside(source, dest);
    }

    void EvaluatorNoCopy::copyFileInside(std::string sourceString, boost::filesystem::path &dest)
    {
        boost::filesystem::path source(sourceString);
        copyFileInside(source, dest);
    }

    void EvaluatorNoCopy::copyFileInside(boost::filesystem::path &source, boost::filesystem::path &dest)
    {
        boost::filesystem::copy_file(source, dest / source.filename(), boost::filesystem::copy_option::overwrite_if_exists);
    }

    void EvaluatorNoCopy::moveFileInside(std::string sourceString, std::string &destString)
    {
        boost::filesystem::path dest(destString);
        boost::filesystem::path source(sourceString);
        copyFileInside(source, dest);
    }

    void EvaluatorNoCopy::moveFileInside(std::string sourceString, boost::filesystem::path &dest)
    {
        boost::filesystem::path source(sourceString);
        copyFileInside(source, dest);
    }

    void EvaluatorNoCopy::moveFileInside(boost::filesystem::path &source, boost::filesystem::path &dest)
    {
        boost::filesystem::rename(source, dest / source.filename());

    }

    void EvaluatorNoCopy::cleanAll(StringList folders)
    {
        for (auto folder : folders) {
            boost::filesystem::path path(folder);
            boost::filesystem::remove_all(path);
        }
    }

    CameraPoseList EvaluatorNoCopy::readCameraPoses(std::string &basicPoseFilename)
    {
        std::ifstream cin(basicPoseFilename);
        CameraPoseList poses;

        int index = 0;
        float x, y, z, yaw, pitch, roll;
        while (!cin.eof()) {
            cin >> x >> y >> z >> yaw >> pitch >> roll;
            poses.push_back(CameraPose(index, x, y, z, yaw, pitch, roll));
            index++;
        }
        cin.close();
        return poses;
    }

    EigVector6 EvaluatorNoCopy::extractPose(std::string &filename)
    {
        std::string path = filename.substr(0, filename.find_last_of("/") + 1);
        std::string name = filename.substr(filename.find_last_of("/") + 1, filename.size());

        return parseEntry(name);
    }
    
    EigVector6 EvaluatorNoCopy::parseEntry(std::string &entry)
    {
        EigVector6 pose;

        entry = entry.substr(0, entry.find_last_of("."));

        StringList blocks;
        std::size_t offset = 0;
        boost::split(blocks, entry, boost::is_any_of("_"));
        
        for (int i = 0; i < 6; i++) {
            pose[i] = std::strtod(blocks[i + 1].c_str(), NULL);
        }
        
        return pose;
    }

    StringList EvaluatorNoCopy::readDatabase(std::string &database) 
    {
        std::ifstream cin(database);
        StringList filelist;

        while (!cin.eof()) {
            std::string file;
            cin >> file;
            file = trim(file);
            if (file.size() > 0){
                filelist.push_back(file);
            }
        }
        cin.close();
        return filelist;
    }

    int EvaluatorNoCopy::getIndexOfSmallestDistance(DoubleList &distances)
    {
        int index = 0;
        for (int i = 0; i < distances.size(); i++) {
            if (distances[i] < distances[index]) {
                index = i;
            }
        }
        return index;
    }

    bool EvaluatorNoCopy::isSpaceSystemError(BoostSystemErrorCode errorCode)
    {
        return errorCode == boost::system::errc::errc_t::file_too_large ||
                errorCode == boost::system::errc::errc_t::not_enough_memory ||
                errorCode == boost::system::errc::errc_t::no_space_on_device;
    }


} // namespace cameval
