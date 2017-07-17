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

    std::string OpenMvgSysCall::computeStructureFromPoses(std::string &jsonFile, size_t uniqueId) 
    {
        std::string outFolder = "poses_sfm_data_" + std::to_string(uniqueId);
        std::string command = "mkdir " + outFolder;
        system(command.c_str());

        log("OpenMvg: Compute incremental sfm");
        command = "openMVG_main_IncrementalSfM -i "+jsonFile+" -m matches/ -o out_Incremental_Reconstruction";
        system(command.c_str());

        log("OpenMvg: Compute structure from known poses");
        command = "openMVG_main_ComputeStructureFromKnownPoses -i out_Incremental_Reconstruction/sfm_data.bin -o "+outFolder+"/sfm_data.json -m matches/ -f matches/matches.f.bin -b";
        system(command.c_str());

        log("OpenMvg: Convert ply from binary to ascii");
        command = "CloudCompare -SILENT -O "+outFolder+"/sfm_data.ply -C_EXPORT_FMT PLY -PLY_EXPORT_FMT ASCII -NO_TIMESTAMP -SAVE_CLOUDS";
        system(command.c_str());

        log("OpenMvg: Scale cloud");
        command = "matlab -nojvm -nodisplay -nosplash -r \"addpath('../../pc_compare'); addpath('"+outFolder+"'); transform_function('fixed_current_points.csv', 'reference_points.csv', '"+outFolder+"/sfm_data.ply', '"+outFolder+"/sfm_data.ply'); exit\"";
        system(command.c_str());

        return outFolder;
    }


} // namespace cameval
