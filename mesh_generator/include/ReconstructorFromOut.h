/*
 * ReconstructorFromOut.h
 *
 *  Created on: 13/lug/2015
 *      Author: andrea
 */

#ifndef RECONSTRUCTORFROMOUT_H_
#define RECONSTRUCTORFROMOUT_H_


#include <fstream>
#include <iostream>
#include <vector>

#include <glm/glm.hpp>

#include <manifoldReconstructor/PointsParserFromOut.h>
#include <manifoldReconstructor/ManifoldMeshReconstructor.h>


class ReconstructorFromOut {
public:
  ReconstructorFromOut();
  virtual ~ReconstructorFromOut();

  void run();

private:
  void parseCenters();

  PointsParserFromOut *pointP;
  std::ifstream fileCams_;
  std::vector<glm::vec3>  camCenters_;
  ManifoldMeshReconstructor *manifRec_;
};

#endif /* RECONSTRUCTORFROMOUT_H_ */
