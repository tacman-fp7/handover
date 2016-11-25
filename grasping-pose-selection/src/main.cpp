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
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class poseSelection : public RFModule
{
    Matrix H_object;
    Matrix H_hand;
    Vector pos, euler_angles, axis;
    vector<Vector> positions;
    vector<Vector> positions_rotated;
    vector<Vector> orientations;
    vector<Vector> x_axis, y_axis, z_axis;
    vector<Vector> x_axis_rotated, y_axis_rotated, z_axis_rotated;

    string orientationFileName;
    string objectPoseFileName;
    string handPoseFileName;
    string positionFileName;
    string homeContextPath;
    string module_name;
    string frame;

    bool change_frame;
    bool online;
    bool euler;

    double length;
    int camera;

    IGazeControl *igaze;
    PolyDriver clientGazeCtrl;

    RpcClient portPoseIn;
    RpcClient portHandIn;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;

    /*********************************************************/
    bool configure(ResourceFinder &rf)
    {
        module_name=rf.check("module_name", Value("pose-selection"), "Getting module name").asString();
        homeContextPath=rf.getHomeContextPath().c_str();
        positionFileName=rf.check("positionFileName", Value("positions.off"), "Default positions file name").asString();
        orientationFileName=rf.check("orientationFileName", Value("orientations-right.txt"), "Default orientations file name").asString();
        objectPoseFileName=rf.check("objectPoseFileName", Value("object-pose.txt"), "Default orientations file name").asString();
        handPoseFileName=rf.check("handPoseFileName", Value("hand-pose.txt"), "Hand pose").asString();
        online=(rf.check("online", Value("no"), "online or offline processing").asString()== "yes");
        camera=(rf.check("camera", Value(0), "online or offline processing").asInt());

        H_object.resize(4,4);
        H_hand.resize(4,4);
        pos.resize(3,0.0);
        euler_angles.resize(3,0.0);
        axis.resize(4,0.0);

        readPoses(positionFileName, orientationFileName);

        if (online)
        {
            portPoseIn.open("/"+module_name+"/ps:rpc");
            portHandIn.open("/"+module_name+"/hn:rpc");
            // temporaneo
            Vector tmp(3,0.0);
            H_object.setCol(3,tmp);
            H_hand=readPoseObjectAndHand(handPoseFileName);
        }
        else
        {
            //H_object=readPoseObjectAndHand(objectPoseFileName);
            H_hand=readPoseObjectAndHand(handPoseFileName);
        }

        length=0.06;        

        portImgIn.open("/" + module_name + "/img:i");
        portImgOut.open("/" + module_name + "/img:o");

        Property optionG;
        optionG.put("device","gazecontrollerclient");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local","/client/gaze");

        clientGazeCtrl.open(optionG);
        igaze=NULL;
        if (clientGazeCtrl.isValid())
        {
            clientGazeCtrl.view(igaze);
        }

        return true;
    }

    /*********************************************************/
    bool updateModule()
    {
        /**cout<<"positions"<<endl;
        for (int i=0; i<positions.size(); i++)
            cout<<positions[i].toString()<<endl;

        cout<<"orientations"<<endl;
        for (int i=0; i<orientations.size(); i++)
            cout<<orientations[i].toString()<<endl;*/

        if (online)
        {
            H_object=askForObjectPose();
            askForHandPose();
        }

        changeFrame();

        /**cout<<"rotated positions"<<endl;
        for (int i=0; i<positions_rotated.size(); i++)
            cout<<positions_rotated[i].toString()<<endl;

        /**cout<<"rotated orientations"<<endl;
        for (int i=0; i<orientations.size(); i++)
        {
            cout<<x_axis_rotated[i].toString()<<endl;
            cout<<y_axis_rotated[i].toString()<<endl;
            cout<<z_axis_rotated[i].toString()<<endl;
        }*/

        showPoses();

        if (online)
            return true;
        else
            return false;
    }

    /*********************************************************/
    bool close()
    {
        portPoseIn.close();
        portHandIn.close();
        portImgIn.close();
        portImgOut.close();
        return true;
    }

    /*********************************************************/
    bool interrupt()
    {
        portPoseIn.interrupt();
        portHandIn.interrupt();
        portImgIn.close();
        portImgOut.close();
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
        Vector tmp2(3,0.0);
        H_object.setCol(3,tmp2);
        /**cout<<"H hand "<<H_hand.toString()<<endl;
        cout<<"H object "<<H_object.toString()<<endl;
        cout<<"H hadn + object "<<(H_hand*H_object).toString()<<endl;*/
        for (size_t i=0; i<positions.size(); i++)
        {
            tmp.setSubvector(0,positions[i].subVector(0,2));
            tmp=H_hand*H_object*(tmp);
            positions_rotated.push_back(tmp.subVector(0,2));
        }

        Vector x(3,0.0), y(3,0.0), z(3,0.0);
        Vector matrix(9,0.0);

        for (size_t i=0; i<orientations.size(); i++)
        {
            matrix=orientations[i];
            x[0]=matrix[0]; x[1]=matrix[3]; x[2]=matrix[6];
            y[0]=matrix[1]; y[1]=matrix[4]; y[2]=matrix[7];
            z[0]=matrix[2]; z[1]=matrix[5]; z[2]=matrix[8];
            x_axis.push_back(positions[i].subVector(0,2)+length*x);
            y_axis.push_back(positions[i].subVector(0,2)+length*y);
            z_axis.push_back(positions[i].subVector(0,2)+length*z);

            tmp.setSubvector(0,x_axis[i]);
            tmp=H_hand*H_object*tmp;
            x_axis_rotated.push_back(tmp.subVector(0,2));

            tmp.setSubvector(0,y_axis[i]);
            tmp=H_hand*H_object*tmp;
            y_axis_rotated.push_back(tmp.subVector(0,2));

            tmp.setSubvector(0,z_axis[i]);
            tmp=H_hand*H_object*tmp;
            z_axis_rotated.push_back(tmp.subVector(0,2));
        }


        return true;
    }

    /*******************************************************************************/
    Matrix readPoseObjectAndHand(string &fileName)
    {
        Matrix H;
        int state=0;
        char line[255];

        cout<< "In pose file "<<homeContextPath+"/"+fileName<<endl;

        ifstream poseFile((homeContextPath+"/"+fileName).c_str());

        if (!poseFile.is_open())
        {
            yError()<<"problem opening pose file!";
        }

        while (!poseFile.eof() || state<2)
        {
            poseFile.getline(line,sizeof(line),'\n');
            Bottle b(line);
            Value firstItem=b.get(0);
            bool isNumber=firstItem.isInt() || firstItem.isDouble();

            if (state==0)
            {
                string tmp=firstItem.asString().c_str();
                if (tmp=="position")
                    state++;
                if (tmp=="orientation")
                    state+=2;
            }
            else if (state==1)
            {
                if (isNumber && (b.size()==3))
                {
                    pos[0]=b.get(0).asDouble();
                    pos[1]=b.get(1).asDouble();
                    pos[2]=b.get(2).asDouble();
                }
                state=0;
            }
            else if (state==2)
            {
                if (isNumber && (b.size()==3))
                {
                    euler_angles[0]=b.get(0).asDouble();
                    euler_angles[1]=b.get(1).asDouble();
                    euler_angles[2]=b.get(2).asDouble();
                    euler=true;
                }
                else if (isNumber && (b.size()==4))
                {
                    axis[0]=b.get(0).asDouble();
                    axis[1]=b.get(0).asDouble();
                    axis[2]=b.get(0).asDouble();
                    axis[3]=b.get(0).asDouble();
                    euler=false;
                }
                state=3;
            }
        }
        H.resize(4,4);

        cout<<"pose "<<pos.toString()<<endl;
        if (euler)
            H=euler2dcm(euler_angles);
        else
            H=axis2dcm(axis);
        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,pos);
        H.setCol(3,x_aux);
        //H=SE3inv(H);

        return H;
    }

    /*******************************************************************************/
    Matrix askForObjectPose()
    {
        Matrix H;
        Bottle cmd,reply;
        cmd.addString("get_estimated_pose");

        if (portPoseIn.write(cmd, reply))
        {
            //portPoseIn.write(cmd, reply);
            Bottle *rec=reply.get(0).asList();
            pos[0]=rec->get(0).asDouble();
            pos[1]=rec->get(1).asDouble();
            pos[2]=rec->get(2).asDouble();

            euler_angles[0]=rec->get(3).asDouble();
            euler_angles[1]=rec->get(4).asDouble();
            euler_angles[2]=rec->get(5).asDouble();
        }
        else
        {
            pos[0]=pos[1]=pos[2]=euler_angles[0]=euler_angles[1]=euler_angles[2];
        }
        cout<<"pose"<<pos.toString()<<" "<<euler_angles.toString()<<endl;

        H.resize(4,4);

        //cout<<"pos "<<pos.toString()<<endl;
        H=euler2dcm(euler_angles);

        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,pos);
        H.setCol(3,x_aux);
        return H;
    }

    /*******************************************************************************/
    bool showPoses()
    {
        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut.resize(imgIn->width(),imgIn->height());

        cv::Mat imgInMat=cv::cvarrToMat((IplImage*)imgIn->getIplImage());
        cv::Mat imgOutMat=cv::cvarrToMat((IplImage*)imgOut.getIplImage());
        imgInMat.copyTo(imgOutMat);

        Vector position_2D(2,0.0);
        Vector axis_2D(2,0.0);

        for (size_t i=0; i<positions.size(); i++)
        {
            igaze->get2DPixel(camera, positions_rotated[i],position_2D);
            cv::Point pixel2D(position_2D[0],position_2D[1]);

            igaze->get2DPixel(camera, x_axis_rotated[i],axis_2D);
            cv::Point pixel_axis_x2D(axis_2D[0],axis_2D[1]);

            cv::line(imgOutMat,pixel2D,pixel_axis_x2D,cv::Scalar(255,0,0));

            igaze->get2DPixel(camera, y_axis_rotated[i],axis_2D);
            cv::Point pixel_axis_y2D(axis_2D[0],axis_2D[1]);
            cv::line(imgOutMat,pixel2D,pixel_axis_y2D,cv::Scalar(0,255,0));

            igaze->get2DPixel(camera, z_axis_rotated[i],axis_2D);
            cv::Point pixel_axis_z2D(axis_2D[0],axis_2D[1]);
            cv::line(imgOutMat,pixel2D,pixel_axis_z2D,cv::Scalar(0,0,255));
        }

        portImgOut.write();

        return true;
    }
    /*******************************************************************************/
    bool askForHandPose()
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

    poseSelection mod;
    ResourceFinder rf;
    rf.setDefaultContext("poseSelection");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
