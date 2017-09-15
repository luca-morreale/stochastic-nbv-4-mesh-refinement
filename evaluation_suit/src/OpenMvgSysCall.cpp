#include <OpenMvgSysCall.hpp>

namespace cameval {

    std::string OpenMvgSysCall::baseImageFolder = "images";
    std::string OpenMvgSysCall::intrinsicParams = "1;0;0;0;1;0;0;0;1";
    
    std::string OpenMvgSysCall::initOpenMvg()
    {
        log("OpenMvg: Init OpenMvg");
        std::string jsonFile = OpenMvgSysCall::imageListing();
        
        std::string emptyString = "";
        OpenMvgSysCall::extractMvgFeatures(jsonFile, emptyString);

        return "matches";
    }


    std::string OpenMvgSysCall::imageListing()
    {
        log("OpenMvg: Listing images");
        std::string command = "openMVG_main_SfMInit_ImageListing -i " + OpenMvgSysCall::baseImageFolder + 
                " -d /usr/local/share/openMVG/sensor_width_camera_database.txt -o matches -k '" + OpenMvgSysCall::intrinsicParams + "'";
        system(command.c_str());
        
        return "matches/sfm_data.json"; // NOTE this can even not be unique because this has to be done just once!!
    }

    void OpenMvgSysCall::extractMvgFeatures(std::string &jsonFile, std::string &pairFile) 
    {
        log("OpenMvg: Compute Features");
        std::string command = "openMVG_main_ComputeFeatures -i " + jsonFile + " -o matches/ -m AKAZE_FLOAT";
        system(command.c_str());

        log("OpenMvg: Compute Matches");
        if (pairFile.empty()) {
            command = "openMVG_main_ComputeMatches -i " + jsonFile + " -o matches/ ";
        } else {
            command = "openMVG_main_ComputeMatches -i " + jsonFile + " -o matches/ -l " + pairFile;
        }
        system(command.c_str());
    }

    std::string OpenMvgSysCall::computeStructureFromPoses(std::string &jsonFile, size_t uniqueId, bool pairs) 
    {
        std::string outFolder = "poses_sfm_data_" + std::to_string(uniqueId);
        std::string command = "mkdir " + outFolder;
        system(command.c_str());

        std::string outputjson = outFolder + "/sfm_data.json";
        std::string outputply = outFolder + "/sfm_data.ply";
        GLMVec3 color(255, 255, 255);
        float outliersthreshold = 0.25;

        log("OpenMvg: Compute structure from known poses");
        command = "openMVG_main_ComputeStructureFromKnownPoses -i matches/sfm_data.json -o "+outputjson+" -m matches/ -b";
        if (pairs) {
            command += " -f matches/matches.f.bin ";
        }
        system(command.c_str());

        return outFolder;
    }


} // namespace cameval
