/*
 * OpenMvgParser.h
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef CAM_PARSERS_OPENMVGPARSER_H_
#define CAM_PARSERS_OPENMVGPARSER_H_

#include <manifoldReconstructor/Exceptions.hpp>
#include <manifoldReconstructor/SfMData.h>
#include <manifoldReconstructor/types_reconstructor.hpp>
#include <manifoldReconstructor/utilities.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <rapidjson/document.h>
#include <rapidjson/reader.h>


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
