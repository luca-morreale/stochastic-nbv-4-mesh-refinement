//  Copyright 2014 Andrea Romanoni
//
//  This file is part of manifoldReconstructor.
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

#include <string>

#include <manifoldReconstructor/OpenMvgParser.h>
#include <manifoldReconstructor/types_reconstructor.hpp>

#include <ManifoldReconstructorConfigurator.h>
#include <PathCreator.h>
#include <ReconstructFromSfMData.h>
#include <ReconstructorFromOut.h>

//*************************************************************************************************/
//********************************RECONSTRUCTION FROM VISIBILITY***********************************/
//*************************************************************************************************/

int main(int argc, char **argv) {
  
    OpenMvgParser op(argv[1]);

    op.parse();
    //utilities::saveVisibilityPly(op.getSfmData());

    ManifoldReconstructionConfig confManif;
    confManif.inverseConicEnabled = true;
    confManif.maxDistanceCamFeature = 100.0;
    confManif.probOrVoteThreshold = 2.1;
    confManif.enableSuboptimalPolicy = false;
    confManif.suboptimalMethod = 0;
    confManif.w_1 = 1.0;
    confManif.w_2 = 0.0;
    confManif.w_3 = 0.00;


    ReconstructFromSfMData m(op.getSfmData(), confManif);
    //m.overwriteFocalY(1525.900000);
    m.run();
    return 0;
}
