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

#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/*******************************************************************************/
class pointCloudExtraction : public RFModule
{
protected:

    string module_name;
    string homeContextPath;
    string savename;
    string fileFormat;
    int fileCount;

    int downsampling;
    double spatial_distance;

    bool flood3d;
    bool saving;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;

    BufferedPort<Bottle> portPointsOut;
    BufferedPort<ImageOf<PixelRgb> > portDispOut;

    RpcClient portSFM;
    RpcClient portSeg;
    RpcServer portRpc;

public:

    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name = rf.check("module_name", Value("in-hand-segmentation"), "Getting module name").asString();

        homeContextPath = rf.getHomeContextPath().c_str();
        savename = rf.check("savename", Value("cloud3D"), "Default file savename").asString();
        saving = rf.check("savingClouds", Value(false), "Toggle save clouds as file").asBool();
        fileFormat = rf.check("format", Value("off"), "Default file format").asString();

        cout << "Files will be saved in "<< homeContextPath << " folder, as " << savename <<"N." << fileFormat <<", with increasing numeration N"  << endl;
        fileCount = 0;

        downsampling = std::max(1,rf.check("downsampling",Value(1)).asInt());
        spatial_distance = rf.check("spatial_distance",Value(0.005)).asDouble();

        cout<<"Opening ports"<<endl;

        portDispIn.open("/" + module_name + "/disp:i");
        portImgIn.open("/" + module_name + "/img:i");

        portPointsOut.open("/"+module_name+"/pnt:o");
        portDispOut.open("/"+module_name+"/disp:o");

        portSFM.open("/"+module_name+"/SFM:rpc");
        portSeg.open("/"+module_name+"/seg:rpc");
        portRpc.open("/"+module_name+"/rpc:i");

        cout<<"Ports opened"<<endl;

        attach(portRpc);

        flood3d=false;

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        portDispIn.interrupt();
        portImgIn.interrupt();

        portPointsOut.interrupt();
        portDispOut.interrupt();

        portSFM.interrupt();
        portSeg.interrupt();
        portRpc.interrupt();

        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        portDispIn.close();
        portImgIn.close();

        portPointsOut.close();
        portDispOut.close();

        portSFM.close();
        portSeg.close();
        portRpc.close();

        return true;
    }

    /*******************************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /*******************************************************************************/
    bool updateModule()
    {
        return true;
    }
};

/*******************************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    pointCloudExtraction mod;
    ResourceFinder rf;
    rf.setDefaultContext("seg2cloud");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
