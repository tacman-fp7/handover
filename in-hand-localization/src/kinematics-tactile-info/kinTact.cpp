
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

    double minDistal, maxDistal, minMiddle, maxMiddle;

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
        fileName<<homeContextPath + "/" + tactOutFileName.c_str() <<"_"+savename<< "_"+frame<<"_"<<fileCount;
        fileNameFormat= fileName.str()+".off";
        fout.open(fileNameFormat.c_str());
        if (fout.is_open())
        {
            fout<<"OFF"<<endl;
            fout<<"3 0 0 "<<endl;
            fout<<contactPoint_thumb.toString()<<endl;
            fout<<contactPoint_index.toString()<<endl;
            fout<<contactPoint_middle.toString()<<endl;

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
        frame=rf.check("frame", Value("robot"), "Getting reference frame").asString();
        savename=rf.check("savename", Value("test"), "Default file savename").asString();

        Bottle &bthumb=limits.addList();
        bthumb.addString("thumb");
        Bottle &blimitp_thumb=bthumb.addList();
        blimitp_thumb.addString("proximal"); blimitp_thumb.addDouble(rf.check("thumb_middle_min", Value(14)).asDouble()); blimitp_thumb.addDouble(rf.check("thumb_middle_max", Value(235)).asDouble());
        Bottle &blimitm_thumb=bthumb.addList();
        blimitm_thumb.addString("middle"); blimitm_thumb.addDouble(rf.check("thumb_middle_min", Value(20)).asDouble()); blimitm_thumb.addDouble(rf.check("thumb_middle_max", Value(215)).asDouble());
        Bottle &blimitd_thumb=bthumb.addList();
        blimitd_thumb.addString("distal"); blimitd_thumb.addDouble(rf.check("thumb_distal_min", Value(24)).asDouble()); blimitd_thumb.addDouble(rf.check("thumb_distal_max", Value(250)).asDouble());

        Bottle &bindex=limits.addList();
        bindex.addString("index");
        Bottle &blimitp_index=bindex.addList();
        blimitp_index.addString("proximal"); blimitp_index.addDouble(rf.check("index_proximal_min", Value(15)).asDouble()); blimitp_index.addDouble(rf.check("index_proximal_max", Value(240)).asDouble());
        Bottle &blimitm_index=bindex.addList();
        blimitm_index.addString("middle"); blimitm_index.addDouble(rf.check("index_middle_min", Value(50)).asDouble()); blimitm_index.addDouble(rf.check("index_middle_max", Value(225)).asDouble());
        Bottle &blimitd_index=bindex.addList();
        blimitd_index.addString("distal"); blimitd_index.addDouble(rf.check("index_distal_min", Value(0)).asDouble()); blimitd_index.addDouble(rf.check("index_distal_max", Value(239)).asDouble());

        Bottle &bmiddle=limits.addList();
        bmiddle.addString("middle");
        Bottle &blimitp_middle=bmiddle.addList();
        blimitp_middle.addString("proximal"); blimitp_middle.addDouble(rf.check("middle_proximal_min", Value(0)).asDouble()); blimitp_middle.addDouble(rf.check("middle_proximal_max", Value(236)).asDouble());
        Bottle &blimitm_middle=bmiddle.addList();
        blimitm_middle.addString("middle"); blimitm_middle.addDouble(rf.check("middle_middle_min", Value(3)).asDouble()); blimitm_middle.addDouble(rf.check("middle_middle_max", Value(234)).asDouble());
        Bottle &blimitd_middle=bmiddle.addList();
        blimitd_middle.addString("distal"); blimitd_middle.addDouble(rf.check("middle_distal_min", Value(19)).asDouble()); blimitd_middle.addDouble(rf.check("middle_distal_max", Value(255)).asDouble());

        cout<<"Bottle limits "<<limits.toString()<<endl;

        cout<<"Files will be saved in "<<homeContextPath<<" folder, as "<<poseOutFileName<<"N."<<" and "<<tactOutFileName<<"N"<<", with increasing numeration N"<< endl;
        fileCount=0;

        robot=rf.find("robot").asString().c_str();
        if(rf.find("robot").isNull())
            robot="icubSim";

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
        string finger_str=left_or_right+"_"+finger_string;

        iCubFinger finger(finger_str);

        enc->getEncoders(encoders.data());
        finger.getChainJoints(encoders,joints);

        analog->read(enc_from_port);

        getLimits(finger_str);

        if (finger_string=="thumb")
        {            
            joints[2]=90 * (1- (enc_from_port[1] - minMiddle)/(maxMiddle - minMiddle));
            joints[3]=90 * (1- (enc_from_port[2] - minDistal)/(maxDistal - minDistal));
        }
        else if (finger_string=="index")
        {
            joints[2]=90 * (1- (enc_from_port[4] - minMiddle)/(maxMiddle - minMiddle));
            joints[3]=90 * (1- (enc_from_port[5] - minDistal)/(maxDistal - minDistal));
        }
        else if (finger_string=="middle")
        {
            joints[1]=90 * (1- (enc_from_port[7] - minMiddle)/(maxMiddle - minMiddle));
            joints[2]=90 * (1- (enc_from_port[8] - minDistal)/(maxDistal - minDistal));
        }

        Matrix tipFrame=finger.getH((M_PI/180.0)*joints);
        Vector tip_x=tipFrame.getCol(3);
        Vector tip_o=dcm2axis(tipFrame);

        icart_arm->attachTipFrame(tip_x.subVector(0,2),tip_o);

        Time::delay(0.1);
        icart_arm->getPose(contactPoint, o);
        icart_arm->removeTipFrame();
        Time::delay(0.1);
    }

    /***********************************************************************/
    void getLimits(string finger)
    {
        for (size_t i=0; i<3; i++)
        {
            Bottle *fing=limits.get(i).asList();

            if (fing->get(0).asString()==finger)
            {
                Bottle *middle=fing->get(2).asList();
                minMiddle=middle->get(1).asDouble();
                maxMiddle=middle->get(2).asDouble();

                Bottle *distal=fing->get(3).asList();
                minDistal=distal->get(1).asDouble();
                maxDistal=distal->get(2).asDouble();

                cout<<"finger "<<finger << endl<<"min - max middle "<<minMiddle<<" "<<maxMiddle<<endl<<" min - max distal "<<minDistal<<" "<<maxDistal<<endl;
           }
        }
    }
};
