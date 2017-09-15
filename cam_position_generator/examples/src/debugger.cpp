
#include <cstdlib>
#include <iostream>
#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/type_definition.h>
#include <opview/MCMCCamGenerator.hpp>
#include <opview/AutonomousMCMCCamGenerator.hpp>
#include <opview/utilities.hpp>

#include <aliases.hpp>
#include <utilities.h>


#define OMP_THREADS 8
#define ARGS 4

#define RESAMPLE 10

namespace opview {

TreePtr tree;

std::string meshFilename = "../../../evaluation_cam/exec/output/from_gen_config/manifold_final.off";

CameraGeneralConfiguration camConfig(1920, 1080, 959.9965); // car
// opview::CameraGeneralConfiguration camConfig(1920, 1080, 1662.8); // 1662.8 car
// centroid, normal

// MeshConfiguration meshConfig(meshFilename, GLMVec3List(), GLMVec3List(), GLMVec3List(), DoubleList());

SpaceBounds bounds(glm::vec3(-60, 0, -60), glm::vec3(60, 70, 60)); // building
// opview::SpaceBounds bounds(glm::vec3(-40, 0, -130), glm::vec3(40, 70, 100)); // fortress
// opview::SpaceBounds bounds(glm::vec3(-15, 0, -15), glm::vec3(15, 10, 0)); // car
ParticlesInformation particles(100000, 100, 30);

MCConfiguration mcConfig(RESAMPLE, particles, bounds);

VonMisesConfiguration vonMisesConfig = {deg2rad(45), 5};


void fillTree();
Polyhedron extractPolyhedron();
TriangleList getTriangleList(Polyhedron &poly);
double computeBDConstraint(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &cam);
double logVonMisesWrapper(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector);
double visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
double imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
double test();


void fillTree()
{
    Polyhedron poly = extractPolyhedron();
    TriangleList triangles = getTriangleList(poly);

    tree = new Tree(triangles.begin(), triangles.end());
}

Polyhedron extractPolyhedron()
{
    std::ifstream meshIn(meshFilename);
    Polyhedron poly;
    meshIn >> poly;
    meshIn.close();

    return poly;
}

TriangleList getTriangleList(Polyhedron &poly)
{
    TriangleList triangles;
    for (Facet_iterator it = poly.facets_begin(); it != poly.facets_end(); it++) {
        Halfedge_facet_circulator p = it->facet_begin();
        Vertex p0 = p->vertex();
        Vertex p1 = (++p)->vertex();
        Vertex p2 = (++p)->vertex();

        triangles.push_back(Triangle(p0->point(), p1->point(), p2->point()));
    }
    return triangles;
}

double computeBDConstraint(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &cam)
{
    GLMVec3 point = GLMVec3(newCamPose[0], newCamPose[1], newCamPose[2]);
    double D, B;
    #pragma omp parallel sections
    {
        #pragma omp section
        B = glm::distance(point, cam);
        #pragma omp section
        D = std::min(glm::distance(cam, centroid), glm::distance(point, centroid));
    }
    
    if (B / D < BD_TERRESTRIAL_ARCHITECTURAL) {
        return -1.0;
    }
    return 0.0;
}

double logVonMisesWrapper(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector)
{
    GLMVec3 point = GLMVec3(newCamPose[0], newCamPose[1], newCamPose[2]);
    return -logVonMises(point, centroid, normVector, vonMisesConfig);
}

double visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
{
    if (isPointInsideImage(pose, centroid, camConfig) && isMeaningfulPose(pose, centroid, tree, camConfig)){
        return 0.0;
    }
    return -10.0;
}

double imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
{
    if (!isPointInsideImage(pose, centroid, camConfig)) {  // fast rejection, fast to compute.
        return -10.0;
    }
    if (!isMeaningfulPose(pose, centroid, tree, camConfig)) {
        return -10.0;
    }
    
    GLMVec2 point = getProjectedPoint(pose, centroid, camConfig);
    double centerx = 0.0;
    double centery = 0.0;
    double sigma_x = (double)camConfig.size_x / 3.0;
    double sigma_y = (double)camConfig.size_y / 3.0;

    return logBivariateGaussian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
}


double test() {
    EigVector5 cam;
    cam << -30, 30, 0, -1.5708, 0.0;
    GLMVec3 centroid(-14, 100, 0);
    GLMVec3 normal(-1, 0, 0);
    GLMVec3 camera(-30, 10, 0);

    // GLMVec4 point3D = GLMVec4(centroid, 1.0f);
    
    

    float roll = 0;
    float pitch = cam[3];
    float yaw = cam[4];

    CameraMatrix P;
    RotationMatrix R; // = getRotationMatrix(0, cam[3], cam[4]);  // already radians
    
    // Calculate rotation about x axis
    RotationMatrix Rx = RotationMatrix(1, 0, 0, 
                                    0, std::cos(roll), -std::sin(roll),
                                    0, std::sin(roll), std::cos(roll));
    // Calculate rotation about y axis
    RotationMatrix Ry = RotationMatrix(std::cos(pitch), 0, std::sin(pitch), 
                                    0, 1, 0,
                                    -std::sin(pitch), 0, std::cos(pitch));
    // Calculate rotation about z axis
    RotationMatrix Rz = RotationMatrix(std::cos(yaw), -std::sin(yaw), 0, 
                                    std::sin(yaw), std::cos(yaw), 0,
                                    0, 0, 1);
    R = Rz * glm::transpose(Ry) * Rx;
    // R = glm::transpose(R);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << R[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;

    GLMVec3 t(cam[0], cam[1], cam[2]);
    t = -t * R;

    CameraMatrix E = CameraMatrix(R);
    E[0][3] = t[0];
    E[1][3] = t[1];
    E[2][3] = t[2];
    E[3][3] = 1.0f;
    
    CameraMatrix K(0.0f);
    K[0][0] = (float)camConfig.f;
    K[1][1] = (float)camConfig.f;  // correct
    K[0][2] = (float)camConfig.size_x / 2.0f;
    K[1][2] = (float)camConfig.size_y / 2.0f;
    K[2][2] = 1.0f;
    K[3][3] = 1.0f;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << K[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << E[i][j] << " ";
        }
        std::cout << std::endl;
    }

    P = E*K;
    std::cout << std::endl << std::endl;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << P[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << std::endl;

    GLMVec4 test = GLMVec4(-14.0f, 28.0f, 0.0f, 1.0f) * E;
    std::cout << test.x << " " << test.y << " " << test.z << std::endl;

    GLMVec3 test3 = glm::transpose(R)*(GLMVec3(-14.0f, 30.0f, 0.0f)-GLMVec3(-30.0f,30.0f,0.0f));
    std::cout << test3.x << " " << test3.y << " " << test3.z << std::endl;

    GLMVec3 test2 = R*test3+GLMVec3(-30.0f,30.0f,0.0f);
    std::cout << test2.x << " " << test2.y << " " << test2.z << std::endl;

    return 0.0;
}


}
int main(int argc, char **argv) {
    opview::test();

    return 0;
}
