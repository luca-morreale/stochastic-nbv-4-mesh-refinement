#include <omp.h>

#include <OpenMvgParser.h>

#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/SSDNFaceAccuracyModel.hpp>
#include <meshac/FacetColorer.hpp>

#include <iostream>
#include <fstream>

#include <aliases.hpp>

#define TIMING
#define OMP_THREADS 8


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    if (argc < 3) {
        std::cout << argv[0] << " mvg.json mesh.off outputlog" << std::endl;
        return 1;
    }

    std::string jsonFile = argv[1];
    std::string meshFile = argv[2];
    std::string log = argv[3];
    
    OpenMvgParser op_openmvg(jsonFile);
    op_openmvg.parse();
    std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    SfMData sfm_data_ = op_openmvg.getSfmData();

    std::string pathPrefix = jsonFile.substr(0, jsonFile.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);

    std::cout << log << std::endl;

    meshac::FaceAccuracyModelPtr model = new meshac::SSDNFaceAccuracyModel(meshFile, sfm_data_, pathPrefix);
    
    std::cout << "done\n";

    std::string colors = "res/config/ssd_facets_colors.json";
    std::string mesh_out = log + ".off";
    std::string report_out = log;
    meshac::FacetColorer colorer(colors, model);

    std::cout << "ok\n";

#ifdef TIMING
    millis accCount, accStart;
    accStart = now();
#endif

    colorer.generateColoredMesh(mesh_out);
    // colorer.generateReport(report_out); 

    // delete model;

#ifdef TIMING
        accCount = now() - accStart; 
        std::cout << std::endl << std::endl << "Total time to estimate accuracy: " << accCount.count() << "ms" << std::endl;
#endif

    
    return 0;
}

/*****
SSD:
    Building:
        4172ms
    Fortress:
        Total time to estimate accuracy: 17971ms

    Car:
        Total time to estimate accuracy: 36245ms
 *****/