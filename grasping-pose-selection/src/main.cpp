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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class poseSelection : public RFModule
{
    Matrix H_object;
    vector<Vector> positions;
    vector<Vector> positions_rotated;
    vector<Vector> orientations;
    vector<Vector> x_axis, y_axis, z_axis;
    vector<Vector> x_axis_rotated, y_axis_rotated, z_axis_rotated;

    string orientationFileName;
    string objectPoseFileName;
    string positionFileName;
    string homeContextPath;
    string module_name;
    string frame;

    bool change_frame;
    bool online;

    RpcClient portPoseIn;

    /*********************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("pose-selection"), "Getting module name").asString();
        positionFileName=rf.check("positionFileName", Value("positions.off"), "Default positions file name").asString();
        orientationFileName=rf.check("orientationFileName", Value("orientations-left.txt"), "Default orientations file name").asString();
        objectPoseFileName=rf.check("objectPoseFileName", Value("object-pose.txt"), "Default orientations file name").asString();
        online=(rf.check("online", Value("yes"), "online or offline processing").asString()== "yes");
        homeContextPath=rf.getHomeContextPath().c_str();

        readPoses(positionFileName, orientationFileName);
        if (online)
            portPoseIn.open("/"+module_name+"ps:rpc");
        else
            H_object=readObjectPose();

        return true;
    }

    /*********************************************************/
    bool updateModule()
    {
        changeFrame();
        cout<<"positions"<<endl;
        for (int i=0; i<positions.size(); i++)
            cout<<positions[i].toString()<<endl;

        cout<<"orientations"<<endl;
        for (int i=0; i<orientations.size(); i++)
            cout<<orientations[i].toString()<<endl;

        if (online)
        {
            askForObjectPose();
            //showPoses();
        }

        return false;
    }

    /*********************************************************/
    bool close()
    {
        portPoseIn.close();
        return true;
    }

    /*********************************************************/
    bool interrupt()
    {
        portPoseIn.interrupt();
        return true;
        return true;
    }

    /*********************************************************/
    bool readPoses(string &positionFileName, string &orientationFileName)
    {
        read(positionFileName, "positions");
        read(orientationFileName, "orientations");

        return true;
    }

    /*******************************************************************************/
    bool read(string &filename, const string &tag)
    {
        int state=0;
        int nPoints;
        char line[255];
        vector<Vector> points_tmp;
        if (tag=="positions")
            positions.clear();
        else if (tag=="orientations")
            orientations.clear();
        Vector point_tmp;
        if (tag=="positions")
            point_tmp.resize(6,0.0);
        else if (tag=="orientations")
            point_tmp.resize(9,0.0);

        cout<< "In cloud file "<<homeContextPath+"/"+filename<<endl;

        ifstream cloudFile((homeContextPath+"/"+filename).c_str());
        if (!cloudFile.is_open())
        {
            yError()<<"problem opening cloud file!";
            return false;
        }

        while (!cloudFile.eof())
        {
            cloudFile.getline(line,sizeof(line),'\n');
            Bottle b(line);
            Value firstItem=b.get(0);
            bool isNumber=firstItem.isInt() || firstItem.isDouble();

            if (state==0)
            {
                string tmp=firstItem.asString().c_str();
                std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
                if (tmp=="COFF" || tmp=="OFF")
                    state++;
            }
            else if (state==1)
            {
                if (isNumber)
                {
                    nPoints=firstItem.asInt();
                    state++;
                }
            }
            else if (state==2)
            {
                if (isNumber && (b.size()>=3))
                {
                    point_tmp[0]=b.get(0).asDouble();
                    point_tmp[1]=b.get(1).asDouble();
                    point_tmp[2]=b.get(2).asDouble();
                    point_tmp[3]=b.get(3).asDouble();
                    point_tmp[4]=b.get(4).asDouble();
                    point_tmp[5]=b.get(5).asDouble();

                    if (tag=="orientations")
                    {
                        point_tmp[6]=b.get(6).asDouble();
                        point_tmp[7]=b.get(7).asDouble();
                        point_tmp[8]=b.get(8).asDouble();
                    }
                    points_tmp.push_back(point_tmp);

                    if (--nPoints<=0)
                    {
                        for( size_t i=0; i<points_tmp.size();i++)
                        {
                            point_tmp=points_tmp[i];
                            if (tag =="positions")
                                positions.push_back(point_tmp);
                            else if (tag=="orientations")
                                orientations.push_back(point_tmp);
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /*******************************************************************************/
    bool changeFrame()
    {
        Vector tmp(4,1.0);
        for (size_t i=0; i<positions.size(); i++)
        {
            positions_rotated.push_back(tmp.setSubvector(0,positions[i]));
        }

        // compute axis of frame and rotate them

        return true;
    }

    /*******************************************************************************/
    Matrix readObjectPose()
    {
        Matrix H;
        int state=0;
        char line[255];

        cout<< "In pose file "<<homeContextPath+"/"+objectPoseFileName<<endl;

        ifstream poseFile((homeContextPath+"/"+objectPoseFileName).c_str());
        if (!poseFile.is_open())
        {
            yError()<<"problem opening pose file!";
        }

        while (!poseFile.eof())
        {
            poseFile.getline(line,sizeof(line),'\n');
            Bottle b(line);
            Value firstItem=b.get(0);
            bool isNumber=firstItem.isInt() || firstItem.isDouble();

            if (state==0)
            {
                string tmp=firstItem.asString().c_str();
                std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
                if (tmp=="position" || tmp=="orientation")
                    state++;
            }
            else if (state==1)
            {
                if (isNumber && (b.size()==3))
                {
                    pos[0]=b.get(0).asDouble();
                    pos[1]=b.get(1).asDouble();
                    pos[2]=b.get(2).asDouble();
                }
            }
            else if (state==2)
            {
                else if (isNumber && (b.size()==3))
                {
                    euler_angles[0]=b.get(0).asDouble();
                    euler_angles[1]=b.get(1).asDouble();
                    euler_angles[2]=b.get(2).asDouble();
                }
            }
        }

        H.resize(4,0.0);

        H=euler2dcm(euler_angles);
        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,position);
        H.setCol(3,x_aux);
        H=SE3inv(H);

        return H;
    }

    /*******************************************************************************/
    bool askForObjectPose()
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

    poseSelection mod;
    ResourceFinder rf;
    rf.setDefaultContext("poseSelection");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
