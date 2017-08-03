#include <omp.h>

#include <OpenMvgParser.h>

#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/SSDNFaceAccuracyModel.hpp>
#include <meshac/FacetColorer.hpp>

#include <iostream>
#include <fstream>


#define OMP_THREADS 8


int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

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

    std::string colors = "res/config/facet_colors.json";
    std::string mesh_out = log + ".off";
    std::string report_out = log + ".txt";
    meshac::FacetColorer colorer(colors, model);

    std::cout << "ok\n";

    colorer.generateColoredMesh(mesh_out);
    // colorer.generateReport("color_facets.txt"); 

    // poche viste comuni dalle diverse camere
    
    delete model;
    
    return 0;
}
