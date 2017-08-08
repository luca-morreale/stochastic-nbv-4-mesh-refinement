#ifndef MESH_ACCURACY_OFF_PARSER_H
#define MESH_ACCURACY_OFF_PARSER_H

#include <fstream>
#include <iostream>

#include <meshac/alias_definition.hpp>
#include <meshac/type_definition.hpp>

namespace meshac {

    class OffParser {
    public:
        OffParser(std::string &offFile);
        ~OffParser();

        PointList getPoints();
        IntPointMap getIndexToPointMap();
        PointIntMap getPointToIndexMap();
        FaceIndexList getFaceIndexList();

    protected:

        void parse();

    private:
        std::string offFile;
        PointList points;
        PointIntMap pointToIndex;
        IntPointMap indexToPoint;
        FaceIndexList facesList;




    };

} // namespace meshac

#endif // MESH_ACCURACY_OFF_PARSER_H
