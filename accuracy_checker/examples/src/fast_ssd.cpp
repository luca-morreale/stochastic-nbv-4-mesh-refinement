#include <omp.h>

#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/FacetColorer.hpp>
#include <meshac/SfMData.h>
#include <meshac/SSDNFaceAccuracyModel.hpp>

#include <iostream>
#include <fstream>

#include <aliases.hpp>
#include <OpenMvgParser.h>

#define TIMING
#define OMP_THREADS 8


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    if (argc < 4) {
        std::cout << argv[0] << " mvg.json mesh.off outputlog" << std::endl;
        return 1;
    }

    std::string jsonFile = argv[1];
    std::string meshFile = argv[2];
    std::string log = argv[3];
    
    OpenMvgParser op_openmvg(jsonFile);
    op_openmvg.parse();

    meshac::SfMData sfm_data_ = op_openmvg.getSfmData();

    std::string pathPrefix = jsonFile.substr(0, jsonFile.find_last_of("/"));
    pathPrefix = pathPrefix.substr(0, pathPrefix.find_last_of("/")+1);

    meshac::FaceAccuracyModelPtr model = new meshac::SSDNFaceAccuracyModel(meshFile, sfm_data_, pathPrefix);

    std::string colors = "res/config/ssd_facets_colors.json";
    std::string mesh_out = log + ".off";
    std::string report_out = log;
    meshac::FacetColorer colorer(colors, model);


#ifdef TIMING
    millis accCount, accStart;
    accStart = now();
#endif

    // colorer.generateColoredMesh(mesh_out);
    colorer.generateReport(report_out); 

    // delete model;

#ifdef TIMING
        accCount = now() - accStart; 
        std::cout << std::endl << std::endl << "Total time to estimate accuracy: " << accCount.count() << "ms" << std::endl;
#endif

    
    return 0;
}
