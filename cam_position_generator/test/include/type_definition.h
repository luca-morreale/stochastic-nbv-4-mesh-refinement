//  Copyright 2014 Andrea Romanoni
//
//  This file is part of edgePointSpaceCarver.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

/**
 * Header-only file with various types, especially those related to the CGAL library
 */
#ifndef TYPES_RECONSTR_HPP_
#define TYPES_RECONSTR_HPP_

#include <string>
#include <glm/glm.hpp>
#include <set>


struct index3 {
    int i, j, k;

    index3(int i_, int j_, int k_) {
        i = i_;
        j = j_;
        k = k_;
    }

    bool operator==(const index3& b) const {
        return i == b.i && j == b.j && k == b.k;
    }
    bool operator<(const index3& b) const {
        return i < b.i || (i == b.i && (j < b.j || (j == b.j && (k < b.k))));
    }

};

struct PointType;

struct CameraType {
    long unsigned int idCam;
    long int idReconstruction = -1;

    glm::mat3 intrinsics;
    glm::mat3 rotation;
    glm::vec3 translation;
    glm::mat4 cameraMatrix;
    glm::vec3 center;
    glm::mat4 mvp;

    std::string pathImage;

    int imageWidth;
    int imageHeight;

//  std::set<int> visiblePoints;
    std::set<PointType*> visiblePointsT, erasedPoints;

    void addPoint(PointType* point) {
        visiblePointsT.insert(point);
    }
};

struct PointType {
    long unsigned int idPoint;
    long int idReconstruction = -1;

    glm::vec3 position;
    std::set<CameraType*> viewingCams;

    float r = 0, g = 0, b = 0, a = 0;

    int getNumberObservations() {
        return viewingCams.size();
    }

    void addCamera(CameraType *cam) {
        viewingCams.insert(cam);
    }
};

#endif /* TYPES_HPP_ */
