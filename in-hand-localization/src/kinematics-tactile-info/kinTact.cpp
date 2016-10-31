
/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.cub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <csignal>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>

#include <yarp/os/Port.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include "kinematicsTactileInfo_IDL.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::iKin;

class kinTactModule: public RFModule,
                     public kinematicsTactileInfo_IDL
{
protected:
    RpcServer portRpc;

    PolyDriver robotDevice;
    PolyDriver robotDevice2;
    PolyDriver analogDevice;

    ICartesianControl *icart_arm;
    ICartesianControl *icart_arm2;
    IEncoders *enc;
    IAnalogSensor *analog;
    Vector encoders;

    ResourceFinder *rf;

    string homeContextPath;
    string module_name;
    string poseOutFileName;
    string tactOutFileName;
    string robot;
    string frame;
    string savename;
    string left_or_right;
    Vector x,o;
    Vector tip_x_init, tip_o_init;
    Matrix H;

    Bottle limits;

    Vector contactPoint_index;
    Vector contactPoint_middle;
    Vector contactPoint_thumb;

    int startup_context_id;
    int fileCount;

    IControlLimits* lim;

    iCubfinger finger_thumb, finger_index, finger_middle;

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /***********************************************************************/
    bool set_frame(const string &entry)
    {
        if (entry=="hand" || entry=="robot")
        {
            frame=entry;
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    string get_frame()
    {
        return frame;
    }
    /***********************************************************************/
    Bottle get_pose()
    {
        icart_arm->getPose(x,o);

        Bottle poseInfo;
        Bottle &positionInfo=poseInfo.addList();
        positionInfo.addString("position");
        positionInfo.addDouble(x[0]); positionInfo.addDouble(x[1]); positionInfo.addDouble(x[2]);
        Bottle &orientationInfo=poseInfo.addList();
        orientationInfo.addString("orientation");
        orientationInfo.addDouble(o[0]); orientationInfo.addDouble(o[1]); orientationInfo.addDouble(o[2]); orientationInfo.addDouble(o[3]);
        Bottle &handb=poseInfo.addList();
        handb.addString("hand");
        handb.addString(left_or_right);

        return poseInfo;
    }

    /***********************************************************************/
    bool save_pose()
    {
        icart_arm->getPose(x,o);

        ofstream fout;
        stringstream fileName;
        fileName<<homeContextPath + "/" + poseOutFileName<<"_"+savename<<"_"<<fileCount;
        string fileNameFormat;
        fileNameFormat=fileName.str()+".txt";
        fout.open(fileNameFormat.c_str());
        if (fout.is_open())
        {
            fout<<left_or_right+" hand pose"<<endl<<endl;
            fout<<x.toString()<<endl;
            fout<<o.toString()<<endl;

            cout<<"Pose saved in "<<fileNameFormat<<endl;
            return true;
        }
        else
        {
            cout<<"Problems in opening pose out file!"<<endl;
            return false;
        }
    }

    /***********************************************************************/
    Bottle get_tactile_data()
    {
        giveContactPoint(contactPoint_thumb, "thumb");
        giveContactPoint(contactPoint_index, "index");
        giveContactPoint(contactPoint_middle, "middle");

        if (frame == "hand")
        {
            Vector aux(4,1.0);
            aux.setSubvector(0,contactPoint_thumb);
            aux=H*aux;
            contactPoint_thumb=aux.subVector(0,2);
            aux.setSubvector(0,contactPoint_index);
            aux=H*aux;
            contactPoint_index=aux.subVector(0,2);
            aux.setSubvector(0,contactPoint_middle);
            aux=H*aux;
            contactPoint_middle=aux.subVector(0,2);
        }

        Bottle tactileData;
        Bottle &thumbData=tactileData.addList();
        thumbData.addString("thumb");
        thumbData.addDouble(contactPoint_thumb[0]); thumbData.addDouble(contactPoint_thumb[1]); thumbData.addDouble(contactPoint_thumb[2]);
        Bottle &indexData=tactileData.addList();
        indexData.addString("index");
        indexData.addDouble(contactPoint_index[0]); indexData.addDouble(contactPoint_index[1]); indexData.addDouble(contactPoint_index[2]);
        Bottle &middleData=tactileData.addList();
        middleData.addString("middle");
        middleData.addDouble(contactPoint_middle[0]); middleData.addDouble(contactPoint_middle[1]); middleData.addDouble(contactPoint_middle[2]);
        Bottle &frameb=tactileData.addList();
        frameb.addString("frame");
        frameb.addString(frame);
        Bottle &handb=tactileData.addList();
        handb.addString("hand");
        handb.addString(left_or_right);


        return tactileData;
    }


    /***********************************************************************/
    bool save_tactile_data()
    {
        giveContactPoint(contactPoint_thumb, "thumb");
        giveContactPoint(contactPoint_index, "index");
        giveContactPoint(contactPoint_middle, "middle");

        if (frame == "hand")
        {
            Vector aux(4,1.0);
            aux.setSubvector(0,contactPoint_thumb);
            aux=H*aux;
            contactPoint_thumb=aux.subVector(0,2);
            aux.setSubvector(0,contactPoint_index);
            aux=H*aux;
            contactPoint_index=aux.subVector(0,2);
            aux.setSubvector(0,contactPoint_middle);
            aux=H*aux;
            contactPoint_middle=aux.subVector(0,2);
        }

        ofstream fout;
        stringstream fileName;
        string fileNameFormat;
        fileName<<homeContextPath + "/" + tactOutFileName.c_str() <<"_"+savename<< "_"+frame<<"_"+left_or_right+"_hand_"<<fileCount;
        fileNameFormat= fileName.str()+".off";
        fout.open(fileNameFormat.c_str());
        if (fout.is_open())
        {
            fout<<"COFF"<<endl;
            fout<<"3 0 0 "<<endl;
            fout<<contactPoint_thumb.toString()<<" 255 0 0 "<<endl;
            fout<<contactPoint_index.toString()<<" 255 0 0 "<<endl;
            fout<<contactPoint_middle.toString()<<" 255 0 0 "<<endl;

            cout<<"Tactile data saved in "<<fileNameFormat<<endl;

            return true;
        }
        else
        {
            cout<<"Problems in opening pose out file!"<<endl;
            return false;
        }
    }

    /*******************************************************************************/
    bool set_filename(const string &entry)
    {
        savename=entry;
        return true;
    }


    /***********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        homeContextPath=rf.getHomeContextPath().c_str();
        module_name=rf.check("module_name", Value("kin-tact"), "Getting module name").asString();
        poseOutFileName=rf.check("pose_file_name", Value("hand_pose"), "Getting pose file name").asString();
        tactOutFileName=rf.check("tact_file_name", Value("tactile_data"), "Getting tactile file name").asString();
        frame=rf.check("frame", Value("hand"), "Getting reference frame").asString();
        savename=rf.check("savename", Value("test"), "Default file savename").asString();

        cout<<"Files will be saved in "<<homeContextPath<<" folder, as "<<poseOutFileName<<"N."<<" and "<<tactOutFileName<<"N"<<", with increasing numeration N"<< endl;
        fileCount=0;

        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
            robot="icub";

        left_or_right=rf.find("which_hand").asString().c_str();
        if(rf.find("which_hand").isNull())
            left_or_right="right";

        Property option_arm("(device cartesiancontrollerclient)");
        option_arm.put("remote","/"+robot+"/cartesianController/"+left_or_right+"_arm");
        option_arm.put("local","/"+module_name+"/cartesian/"+left_or_right+"_arm");

        robotDevice.open(option_arm);
        if (!robotDevice.isValid())
        {
            yError("Device index not available!");
            return false;
        }

        robotDevice.view(icart_arm);

        Property option_arm2("(device remote_controlboard)");
        option_arm2.put("remote","/"+robot+"/"+left_or_right+"_arm");
        option_arm2.put("local","/"+module_name+"/joint/"+left_or_right+"_arm");

        robotDevice2.open(option_arm2);
        if (!robotDevice2.isValid())
        {
            yError("Device 2 not available!");
            return false;
        }

        robotDevice2.view(enc);

        robotDevice2.view(lim);
        deque<IControlLimits*> lim_deque;
        lim_deque.push_back(lim);

        finger_thumb("thumb");
        finger_thumb.alignJointsBounds(lim_deque);

        finger_index("index");
        finger_index.alignJointsBounds(lim_deque);

        finger_middle("middle");
        finger_middle.alignJointsBounds(lim_deque);

        int jnts;
        enc->getAxes(&jnts);
        encoders.resize(jnts);
        enc->getEncoders(encoders.data());
        icart_arm->storeContext(&startup_context_id);
        icart_arm->getTipFrame(tip_x_init, tip_o_init);

        cout<<"tip_x_init "<<tip_x_init.toString()<<endl;
        cout<<"tip_o_init "<<tip_o_init.toString()<<endl;

        icart_arm->getPose(x,o);

        H.resize(4,0.0);

        H=axis2dcm(o);
        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,x);
        H.setCol(3,x_aux);
        H=SE3inv(H);

        Property option_analog("(device analogsensorclient)");
        option_analog.put("remote","/"+robot+"/"+left_or_right+"_hand"+"/analog:o");
        option_analog.put("local","/"+module_name+"/analogsensorclient/"+left_or_right+"_hand");

        analogDevice.open(option_analog);
        if (!analogDevice.isValid())
        {
            yError("Device index not available!");
            return false;
        }

        analogDevice.view(analog);

        x.resize(3,0.0);
        o.resize(4,0.0);
        contactPoint_index.resize(3,0.0);
        contactPoint_middle.resize(3,0.0);
        contactPoint_thumb.resize(3,0.0);

        portRpc.open("/"+module_name+"/rpc");

        attach(portRpc);

        return true;
    }

    /***********************************************************************/
    bool close()
    {
        robotDevice.close();
        robotDevice2.close();

        portRpc.close();
    }

    /***********************************************************************/
    bool updateModule()
    {
        return true;
    }

    /***********************************************************************/
    void giveContactPoint(Vector &contactPoint, const string &finger_string)
    {
        contactPoint.resize(3,0.0);
        Vector joints, enc_from_port;
        enc_from_port.resize(3,0.0);
        //string finger_str=left_or_right+"_"+finger_string;

        iCubfinger finger;
        if (finger_string=="thumb")
            finger=finger_thumb;
        if (finger_string=="index")
            finger=finger_index;
        if (finger_string=="middle")
            finger=finger_middle;

        enc->getEncoders(encoders.data());
        analog->read(enc_from_port);
        finger.getChainJoints(encoders,enc_from_port,joints);

        Matrix tipFrame=finger.getH((M_PI/180.0)*joints);
        Vector tip_x=tipFrame.getCol(3);
        Vector tip_o=dcm2axis(tipFrame);

        icart_arm->attachTipFrame(tip_x.subVector(0,2),tip_o);

        Time::delay(0.1);
        icart_arm->getPose(contactPoint, o);
        icart_arm->removeTipFrame();
        Time::delay(0.1);
    }
};
