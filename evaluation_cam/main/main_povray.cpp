#include <iostream>
#include <fstream>

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <utilities.hpp>

void scale(std::ofstream &cout, std::string &mode, glm::vec3 &pos, glm::vec3 &lt);
void fill(glm::mat3 &mat, float &scale, glm::vec3 &translate, std::string file);

int main(int argc, char **argv) {

    if (argc < 4) {
        std::cout << argv[0] << " mode[f-b-i] points.txt outputfile.txt" << std::endl;
        return 1;
    }

    std::string mode = argv[1];
    std::string points = argv[2];
    std::string outputfile = argv[3];

    std::ifstream cin(points);
    std::ofstream cout(outputfile);
    std::string comment;

    while (!cin.eof()) {
        glm::vec3 pos, lt;
        cin >> pos.x >> pos.y >> pos.z >> lt.x >> lt.y >> lt.z;
        if (pos.x == 0.0f && pos.y == 0.0f && pos.z == 0.0f && lt.x == 0.0f && lt.y == 0.0f && lt.z == 0.0f) {
            continue;
        }
        scale(cout, mode, pos, lt);
    }
    cin.close();
    cout.close();

    return 0;
}

void scale(std::ofstream &cout, std::string &mode, glm::vec3 &pos, glm::vec3 &lt)
{
    glm::mat3 transform;
    float scale;
    glm::vec3 translate;
    if (mode.compare("f") == 0) {
        fill(transform, scale, translate, "forward_transform.txt");
    } else if (mode.compare("b") == 0) {
        fill(transform, scale, translate, "backward_transform.txt");
    } else { // else no transformation is performed
        scale = 1.0;
        translate.x = 0;
        translate.y = 0;
        translate.z = 0; 
    }
    // std::cout << scale << " " << glm::to_string(translate) << " " << glm::to_string(transform) << std::endl; 
    
    pos = scale * transform * pos + translate;
    lt = scale * transform * lt + translate;

    cout << pos.x << "_" << pos.y << "_" << pos.z << "_" << lt.x << "_" << lt.y << "_" << lt.z << std::endl;
}

void fill(glm::mat3 &mat, float &scale, glm::vec3 &translate, std::string file)
{
    std::ifstream cin(file);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cin >> mat[i][j];
        }
    }
    cin >> scale;
    for (int i = 0; i < 3; i++) {
        cin >> translate[i];
    }
    cin.close();
}
