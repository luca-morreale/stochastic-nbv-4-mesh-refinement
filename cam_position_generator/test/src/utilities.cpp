#include <utilities.h>

namespace utilities {


glm::mat3 rotX(float alpha) {
    glm::mat3 rot; //set identity matrix
    rot[1][1] = cos(alpha);
    rot[1][2] = -sin(alpha);
    rot[2][1] = sin(alpha);
    rot[2][2] = cos(alpha);
    return rot;
}

glm::mat3 rotY(float alpha) {
    glm::mat3 rot; //set identity matrix
    rot[0][0] = cos(alpha);
    rot[0][2] = sin(alpha);
    rot[2][0] = -sin(alpha);
    rot[2][2] = cos(alpha);
    return rot;
}

glm::mat3 rotZ(float alpha) {
    glm::mat3 rot; //set identity matrix
    rot[0][0] = cos(alpha);
    rot[0][1] = -sin(alpha);
    rot[1][0] = sin(alpha);
    rot[1][1] = cos(alpha);
    return rot;
}

AccuracyScore readScores(std::string accScore) {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;
    std::vector<double> uncertainty;

    double x, y, z, nx, ny, nz, unc;
    std::ifstream cin(accScore);
    while(!cin.eof()) {
        cin >> x >> y >> z >> nx >> ny >> nz >> unc;

        if (unc > 100000) continue; // steiner point in most methods

        points.push_back(glm::vec3(x, y, z));
        normals.push_back(glm::vec3(nx, ny, nz));
        uncertainty.push_back(unc);
    }
    cin.close();

    return {points, normals, uncertainty};
}

}

