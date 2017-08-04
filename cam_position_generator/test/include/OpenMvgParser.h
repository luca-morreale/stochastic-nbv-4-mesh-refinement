/*
 * OpenMvgParser.h
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef CAM_PARSERS_OPENMVGPARSER_H_
#define CAM_PARSERS_OPENMVGPARSER_H_


#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <glm/glm.hpp>

#include <rapidjson/document.h>

#include <stdexcept>

#include <type_definition.h>

struct JsonParseException : public std::runtime_error
{
  JsonParseException(const std::string& msg) : std::runtime_error(msg) {}
};

struct JsonAccessException : public std::runtime_error
{
  JsonAccessException(const std::string& msg) : std::runtime_error(msg) {}
};


struct SfMData {

  int numPoints_;
  int numCameras_;

  std::vector<glm::vec3> points_;                           // list of 3D points
  std::vector<CameraType> camerasList_;                     // list of cameras
  std::vector<std::string> camerasPaths_;                   // list of camera's path

  std::vector<std::vector<int> > camViewingPointN_;         // each list contains the index of camera that see that point
  std::vector<std::vector<int> > pointsVisibleFromCamN_;    // each list contains the index of points that are seen from that cam
  std::vector<std::vector<glm::vec2> > point2DoncamViewingPoint_;   // a list for each 3D point, each one contains the 2D observations

  std::vector<std::vector<glm::vec2> > camViewing2DPoint_;    // list of observations (2D points) for each camera
  std::vector<std::map<int, glm::vec2> > point3DTo2DThroughCam_;   // maps points 3D to 2D through cam

  int imageWidth_, imageHeight_;
};




class OpenMvgParser {
public:
  OpenMvgParser(std::string path);
  virtual ~OpenMvgParser();
  virtual void parse();

  const SfMData& getSfmData() const {
    return sfm_data_;
  }

private:
  void parseViews(const std::map<int,glm::mat3> & intrinsics, const std::map<int,CameraType> & extrinsics);
  void parseIntrinsics(std::map<int,glm::mat3> & intrinsics);
  void parseExtrinsics(std::map<int,CameraType> & extrinsics);
  void parsePoints();


  rapidjson::Document document_;
  std::string fileName_;
  std::ifstream fileStream_;
  SfMData sfm_data_;

};

#endif /* CAM_PARSERS_OPENMVGPARSER_H_ */
