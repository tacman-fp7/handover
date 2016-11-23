/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class poseSelection : public RFModule
{
    deque<Vectors> poses;

    string orientationFileName;
    string positionFileName;
    string frame;

    bool change_frame;

    /*********************************************************/
    bool configure(ResourceFinder &rf)
    {
        positionFileName=rf.check("positionFileName", Value("positions.off"), "Default positions file name").asString();
        orientationFileName=rf.check("orientationFileName", Value("orientations-right.off"), "Default orientations file name").asString();
        readPoses(orientationFileName, positionFileName);

        if (frame == "hand")
            changeEstimatedPoseFrame();

        return true;
    }

    /*********************************************************/
    bool updateModule()
    {
        choosePose();

        return true;
    }

    /*********************************************************/
    bool close()
    {
        return true;
    }


};

int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    inHandSegmentation mod;
    ResourceFinder rf;
    rf.setDefaultContext("poseSelection");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
