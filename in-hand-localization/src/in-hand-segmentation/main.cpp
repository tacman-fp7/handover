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

#include "inHandSegmentation.cpp"

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
    rf.setDefaultContext("inHandSegmentation");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
