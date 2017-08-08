#include <meshac/OffParser.hpp>

namespace meshac {

    OffParser::OffParser(std::string &offFile)
    {
        this->offFile = offFile;
        parse();
    }

    OffParser::~OffParser()
    {
        points.clear();
        indexToPoint.clear();
        pointToIndex.clear();
        facesList.clear();
    }

    void OffParser::parse()
    {
        std::ifstream in(offFile);

        std::string tmpString;
        int tmpInt, v1, v2, v3, pointCount, facetsCount;
        double x, y, z;

        
        in >> tmpString;
        in >> pointCount >> facetsCount >> tmpInt;

        
        for (int i = 0; i < pointCount; i++) {
            in >> x >> y >> z;
            Point p(x, y, z);
            points.push_back(p);
            indexToPoint.insert(std::make_pair(i, p));
            pointToIndex.insert(std::make_pair(p, i));
        }

        for (int i = 0; i < facetsCount; i++) {
            FaceIndex face;

            in >> tmpInt >> v1 >> v2 >> v3;
            
            face.set(0, v1);
            face.set(1, v2);
            face.set(2, v3);
            facesList.push_back(face);
        }
        in.close();
    }


    PointList OffParser::getPoints()
    {
        return points;
    }

    IntPointMap OffParser::getIndexToPointMap()
    {
        return indexToPoint;
    }

    PointIntMap OffParser::getPointToIndexMap()
    {
        return pointToIndex;
    }

    FaceIndexList OffParser::getFaceIndexList()
    {
        return facesList;
    }

} // namespace meshac
