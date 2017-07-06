#include <Evaluator.hpp>

namespace cameval {

    Evaluator::Evaluator(std::string &pointsFilename, std::string &groundTruthFilename, std::string &databaseFilename, 
            std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, 
            std::string &sshconfig)
    {
        this->pointsFilename = pointsFilename;                      // file from which read poses to evaluate
        this->groundTruthFilename = groundTruthFilename;            // file containing the g-t cloud of points
        this->databaseFilename = databaseFilename;                  // file containing the list of images in the database
        this->baseImageFolder = baseImageFolder;                    // name of folder containing the images of current cloud
        this->intrinsicParams = intrinsicParams;                    // file containing the intrinisc params in format ready for opengm
        this->distanceRegex = std::regex("\\[(.*)\\] \\[ComputeDistances\\] Mean distance = (.*) / std deviation = (.*)");

        this->poses = readPoints(pointsFilename);
        this->database = readDatabase(databaseFilename);
        this->basicPoses = readCameraPoses(basicPoseFilename);

        this->filePrefix = database[0].substr(0, database[0].find_first_of("_"));
        this->fileExtention = database[0].substr(database[0].find_last_of(".") + 1, database[0].size());

        this->tree = new KDTree(database);
        this->sshHandler = new SshHandler(sshconfig);
    }

    Evaluator::~Evaluator()
    {
        delete tree;
        delete sshHandler;
    }

    void Evaluator::evaluate()
    {
        std::string basicFolder = initOpenMvg();

        DoubleList distances(poses.size());

        #pragma omp parallel for
        for (int i = 0; i < poses.size(); ++i) {
            distances[i] = evaluatePose(poses[i], basicFolder, i+1);    // i+1 because 0 is the original copy
        }

        int index = getIndexOfSmallestDistance(distances);
        std::cout << "min distance: " << distances[index] << std::endl;
        std::cout << "pose: " << poses[index] << std::endl;
    }

    std::string Evaluator::initOpenMvg()
    {
        std::string jsonFile = imageListing();
        std::string baseFolder = extractMvgFeatures(jsonFile, 0);
        OpenMvgJsonHandler mvgJsonHandler(baseFolder+"/sfm_data.json");
        setDefaultCameraPoses(jsonFile, mvgJsonHandler);

        // TODO I should save changes after modification of sfm_data.json

        return baseFolder;
    }

    double Evaluator::evaluatePose(EigVector6 &pose, std::string &basicFolder, int uniqueId)
    {
        std::string filePath = getClosestImage(pose);
        sshHandler->download(filePath);

        std::string privateFolder = basicFolder + std::to_string(uniqueId);
        std::string jsonFile = privateFolder + "/sfm_data.json";
        std::string imageFile = privateFolder + "/" + filePath;

        // TODO improve readability
        try {
            copyDataToPrivateFolder(baseImageFolder, privateFolder, filePath);
        } catch (const boost::filesystem::filesystem_error &ex) {
            auto errorCode = ex.code();
            if (errorCode == boost::system::errc::errc_t::file_too_large ||
                errorCode == boost::system::errc::errc_t::not_enough_memory ||
                errorCode == boost::system::errc::errc_t::no_space_on_device) {
                std::cerr << "Out of Memory Exception" << std::endl;
                return DBL_MAX;
            }
        }

        OpenMvgJsonHandler mvgJsonHandler(jsonFile);

        appendImageToJson(jsonFile, filePath, mvgJsonHandler);
        setPositionOfCameras(jsonFile, pose, mvgJsonHandler);
        // TODO I should save changes after modification of sfm_data.json

        std::string matchesFolder = extractMvgFeatures(jsonFile, uniqueId);
        std::string mvgFolder = computeStructureFromPoses(jsonFile, matchesFolder, uniqueId);

        std::string inputCloud = mvgFolder + "/sfm_data.ply";

        // std::string alignedCloud = alignClouds(inputCloud);  // not needed because inserted camera position
    
        std::string logFile = computeDistance(inputCloud, groundTruthFilename); 

        double distance = parseDistance(logFile);     

        cleanAll({privateFolder, filePath, matchesFolder, mvgFolder, inputCloud, logFile});     // remove all folders created!

        return distance;
    }

    void Evaluator::setDefaultCameraPoses(std::string &jsonFile, OpenMvgJsonHandler &mvgJsonHandler)
    {
        for (auto pose : basicPoses) {
            // convert eigen to glm
            auto center = convert(pose.center);
            auto rotation = convert(pose.rotation);
            mvgJsonHandler.setCamPosition(pose.index, center, rotation);
        }
    }

    std::string Evaluator::imageListing()
    {
        std::string command = "openMVG_main_SfMInit_ImageListing -i " + baseImageFolder + 
            " -d /usr/local/share/openMVG/sensor_width_camera_database.txt -o matches -k " + intrinsicParams;
        system(command.c_str());
        return "matches/sfm_data.json"; // NOTE this can even not be unique because this has to be done just once!!
    }

    std::string Evaluator::extractMvgFeatures(std::string &jsonFile, int uniqueId) 
    {
        std::string matchesFolder = "matches_" + std::to_string(uniqueId);
        std::string command = "openMVG_main_ComputeFeatures -i " + jsonFile + " -o " + matchesFolder + "/ -m AKAZE_FLOAT";
        system(command.c_str());
        command = "openMVG_main_ComputeMatches -i " + jsonFile + " -o " + matchesFolder + "/";
        system(command.c_str());
        
        return matchesFolder;
    }

    std::string Evaluator::getClosestImage(EigVector6 &pose)
    {
        EigVector6 closest = tree->searchClosestPoint(pose);
        std::string out;
        for (int i = 0; i < 6; i++) {
            out += "_" + std::to_string(closest[i]);
        }
        return this->filePrefix + out + "." + fileExtention;
    }

    void Evaluator::appendImageToJson(std::string &sfmFile, std::string &imageFile, OpenMvgJsonHandler &mvgJsonHandler)
    {
        std::string prefix = sfmFile.substr(0, sfmFile.find_last_of("/"));
        mvgJsonHandler.appendImage(prefix, imageFile);
    }

    void Evaluator::setPositionOfCameras(std::string &sfmFile, EigVector6 &pose, OpenMvgJsonHandler &mvgJsonHandler) 
    {
        int imageCount = mvgJsonHandler.countImages();
        CameraPose cam(-1, pose[0], pose[1], pose[2], pose[5], pose[4], pose[3]);
        auto center = convert(cam.center);
        auto rotation = convert(cam.rotation);
        mvgJsonHandler.setCamPosition(imageCount-1, center, rotation);
    }

    std::string Evaluator::computeStructureFromPoses(std::string &jsonFile, std::string &matchesFolder, int uniqueId) 
    {
        std::string outFolder = "poses_sfm_data_" + std::to_string(uniqueId);
        std::string command = "openMVG_main_ComputeStructureFromKnownPoses -i "+jsonFile+" -o "+outFolder+"/sfm_data.json -m "+matchesFolder+"/";
        system(command.c_str());
        return outFolder;
    }

    std::string Evaluator::computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
    {
        std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "log_distance.txt";

        // assume definition of alias for CloudCompare -> 
        std::string command = "ccmp -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + "  -c2c_dist";
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

    std::string Evaluator::readStringFromFile(std::string &filename)
    {
        std::ifstream cin(filename);
        std::string content((std::istreambuf_iterator<char>(cin)), std::istreambuf_iterator<char>());
        cin.close();
        return content;
    }

    void Evaluator::copyDataToPrivateFolder(std::string &basicFolder, std::string &privateFolder, std::string &filePath)
    {
        boost::filesystem::path imagePath(filePath);
        boost::filesystem::path source(basicFolder);
        boost::filesystem::path destination(privateFolder);

        // Check whether the function call is valid
        copyDirectory(source, destination);
        copyFileInside(imagePath, destination);
        copyFileInside("sfm_data.json", destination);
    }

    void Evaluator::createDestinationFolder(boost::filesystem::path &destination)
    {
        
        if(boost::filesystem::exists(destination)) {
            std::cerr << "Destination directory " << destination.string() << " already exists." << std::endl;
        } else if(!boost::filesystem::create_directory(destination)) { // Create the destination directory
            std::cerr << "Unable to create destination directory" << destination.string() << std::endl;
            throw UnableToCreateDirectoryException(destination);
        }
    }

    void Evaluator::checkSourceExistence(boost::filesystem::path &source)
    {
        if(!boost::filesystem::exists(source) || !boost::filesystem::is_directory(source)) {
            throw SourceDirectoryDoesNotExistsException(source);
        }
    }

    void Evaluator::copyDirectory(boost::filesystem::path &source, boost::filesystem::path &destination)
    {
        checkSourceExistence(source);
        createDestinationFolder(destination);

        copyAllFilesFromTo(source, destination);
    }

    void Evaluator::copyAllFilesFromTo(boost::filesystem::path &source, boost::filesystem::path &destination)
    {
        // Iterate through the source directory
        for(boost::filesystem::directory_iterator file(source); file != boost::filesystem::directory_iterator(); ++file) {
            boost::filesystem::path current(file->path());
            copyFileInside(current, destination);
        }
    }

    void Evaluator::copyFileInside(std::string sourceString, boost::filesystem::path &dest)
    {
        boost::filesystem::path source(sourceString);
        copyFileInside(source, dest);
    }

    void Evaluator::copyFileInside(boost::filesystem::path &source, boost::filesystem::path &dest)
    {
        boost::filesystem::copy_file(source, dest / source.filename(), boost::filesystem::copy_option::overwrite_if_exists);

    }

    void Evaluator::cleanAll(StringList folders)
    {
        for (auto folder : folders) {
            boost::filesystem::path path(folder);
            remove_all(path);
        }
    }

    CameraPoseList Evaluator::readCameraPoses(std::string &basicPoseFilename)
    {
        std::ifstream cin(basicPoseFilename);
        CameraPoseList poses;

        int index;
        float x, y, z, yaw, pitch, roll;
        while (!cin.eof()) {
            cin >> index >> x >> y >> z >> roll >> pitch >> yaw;
            poses.push_back(CameraPose(index, x, y, z, yaw, pitch, roll));
        }
        cin.close();
        return poses;
    }

    PoseList Evaluator::readPoints(std::string &pointsFilename) 
    {
        std::ifstream cin(pointsFilename);
        PoseList poses;

        while (!cin.eof()) {
            EigVector6 point;
            cin >> point[0] >> point[1] >> point[2] >> point[3] >> point[4] >> point[5];
            poses.push_back(point);
        }
        cin.close();
        return poses;
    }

    StringList Evaluator::readDatabase(std::string &database) 
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
