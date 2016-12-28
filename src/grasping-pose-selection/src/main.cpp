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

#include <iCub/iKin/iKinFwd.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

#include "src/poseSelection_IDL.h"

class poseSelection : public RFModule,
                      public poseSelection_IDL
{
    Matrix H_object, H_hand;
    Vector xd_h, od_h;
    Vector index_poses;
    Vector pos_hand, axis_hand;
    Vector pos, euler_angles, axis;
    Vector x_init_moving_arm, o_init_moving_arm;
    Vector x_init_resting_arm, o_init_resting_arm;

    vector<Vector> od;
    vector<Vector> positions;
    vector<Vector> orientations;
    vector<Vector> positions_rotated;    
    vector<Vector> x_axis, y_axis, z_axis;
    vector<Vector> x_axis_rotated, y_axis_rotated, z_axis_rotated;

    string orientationFileName;
    string objectPoseFileName;
    string handPoseFileName;
    string positionFileName;
    string homeContextPath;
    string left_or_right;
    string module_name;
    string frame;
    string robot;

    bool select_new_pose;
    bool torso_enabled;
    bool change_frame;
    bool closed_chain;
    bool update_pose;
    bool go_to_home;
    bool online;
    bool euler;
    bool move;

    double length;

    int index;
    int camera;
    int startup_context_id;

    deque<IControlLimits*> lim_deque;

    iCubArm ikin_arm;
    iCubTorso ikin_torso;
    iKinChain *chain;
    IGazeControl *igaze;    
    IControlLimits* lim_arm;
    IControlLimits* lim_torso;
    ICartesianControl *icart_arm_move;

    PolyDriver clientGazeCtrl;
    PolyDriver robotDevice_move;
    PolyDriver robotDevice2;
    PolyDriver robotDevice3;

    RpcClient portPoseIn;
    RpcClient portHandIn;

    RpcServer portRpc;

    ImageOf<PixelRgb> *imgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    bool ask_new_pose()
    {
        update_pose=true;
        select_new_pose=true;
        return true;
    }

    /************************************************************************/
    bool reach_pose()
    {
        move=true;
        return true;
    }

    /************************************************************************/
    bool choose_new_pose()
    {
        select_new_pose=true;
        return true;
    }

    /************************************************************************/
    bool go_home()
   {
        go_to_home=true;
        return true;
    }

    /************************************************************************/
    bool select_pose(int entry)
    {
        if (entry>=0 && entry<positions.size())
        {
            Vector qdhat(10,0.0);
            index=entry;
            icart_arm_move->askForPose(positions_rotated[index], od[index], xd_h, od_h, qdhat);
            select_new_pose=false;
            return true;
        }
        else
            return false;
    }

    /*********************************************************/
    bool configure(ResourceFinder &rf)
    {
        homeContextPath=rf.getHomeContextPath().c_str();
        module_name=rf.check("module_name", Value("pose-selection"), "Getting module name").asString();

        handPoseFileName=rf.check("handPoseFileName", Value("hand-pose.txt"), "Hand pose").asString();
        positionFileName=rf.check("positionFileName", Value("positions.off"), "Default positions file name").asString();        
        objectPoseFileName=rf.check("objectPoseFileName", Value("object-pose.txt"), "Default object pose file name").asString();
        orientationFileName=rf.check("orientationFileName", Value("orientations-right.txt"), "Default orientations file name").asString();

        online=(rf.check("online", Value("yes"), "online or offline processing").asString()== "yes");
        camera=(rf.check("camera", Value(0), "online or offline processing").asInt());
        torso_enabled=(rf.check("torso_enabled", Value("no")).asString()== "yes");
        closed_chain=(rf.check("closed_chain", Value("no")).asString()== "yes");

        robot=rf.check("robot", Value("icubSim")).asString();
        left_or_right=rf.check("which_hand", Value("left")).asString();

        pos.resize(3,0.0);
        xd_h.resize(3,0.0);
        od_h.resize(4,0.0);
        axis.resize(4,0.0);
        H_hand.resize(4,4);
        H_object.resize(4,4);
        pos_hand.resize(3,0.0);
        axis_hand.resize(4,0.0);
        euler_angles.resize(3,0.0);       
        x_init_moving_arm.resize(3,0.0);
        o_init_moving_arm.resize(4,0.0);
        x_init_resting_arm.resize(3,0.0);
        o_init_resting_arm.resize(4,0.0);

        readPoses(positionFileName, orientationFileName);

        index_poses.resize(positions.size(), 0.0);

        index=-1;
        length=0.06;

        select_new_pose=true;
        update_pose=false;
        move=false;

        if (online)
        {
            portPoseIn.open("/"+module_name+"/ps:rpc");
            portHandIn.open("/"+module_name+"/hn:rpc");

            Vector tmp(3,0.0);
            H_object.setCol(3,tmp);
        }
        else
        {
            H_object=readPoseObjectAndHand(objectPoseFileName);
            H_hand=readPoseObjectAndHand(handPoseFileName);
        }

        portRpc.open("/"+module_name+"/rpc");
        portImgIn.open("/" + module_name + "/img:i");
        portImgOut.open("/" + module_name + "/img:o");

        attach(portRpc);

        Property optionG;
        optionG.put("device","gazecontrollerclient");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local","/"+module_name+"/gaze");

        clientGazeCtrl.open(optionG);
        igaze=NULL;

        if (clientGazeCtrl.isValid())
        {
            clientGazeCtrl.view(igaze);
        }
        else
            yError(" Gaze NOT OPENED!");

        Property option_arm_move("(device cartesiancontrollerclient)");
        option_arm_move.put("remote","/"+robot+"/cartesianController/"+left_or_right+"_arm");
        option_arm_move.put("local","/"+module_name+"/cartesian/"+left_or_right+"_arm");

        robotDevice_move.open(option_arm_move);
        if (!robotDevice_move.isValid())
        {
            yError(" Device index not available!");
            return false;
        }

        robotDevice_move.view(icart_arm_move);
        icart_arm_move->storeContext(&startup_context_id);
        icart_arm_move->getPose(x_init_moving_arm, o_init_moving_arm);

        Vector curDof;
        icart_arm_move->getDOF(curDof);
        Vector newDof(3);
        newDof.resize(3,0);
        if (torso_enabled)
        {
            newDof[0]=1;
            newDof[1]=1;
            newDof[2]=1;
        }
        icart_arm_move->setDOF(newDof,curDof);

        if (left_or_right=="right")
            ikin_arm=iCubArm("right_v2");
        else
            ikin_arm=iCubArm("left_v2");

        ikin_torso=iCubTorso();

        Property option_arm2("(device remote_controlboard)");
        option_arm2.put("remote","/"+robot+"/"+left_or_right+"_arm");
        option_arm2.put("local","/"+module_name+"/joint/"+left_or_right+"_arm");

        robotDevice2.open(option_arm2);
        if (!robotDevice2.isValid())
        {
            yError(" Device 2 not available!");
            return false;
        }

        robotDevice2.view(lim_arm);

        Property option_arm3("(device remote_controlboard)");
        option_arm3.put("remote","/"+robot+"/torso");
        option_arm3.put("local","/"+module_name+"/joint/torso");

        robotDevice3.open(option_arm3);
        if (!robotDevice3.isValid())
        {
            yError(" Device 2 not available!");
            return false;
        }

        robotDevice3.view(lim_torso);

        lim_deque.push_back(lim_torso);
        lim_deque.push_back(lim_arm);

        if (!ikin_arm.alignJointsBounds(lim_deque))
            yError(" Problems in alignJointsBounds()");

        bool rel=ikin_torso.releaseLink(0);
        rel=rel && ikin_torso.releaseLink(1);
        rel=rel && ikin_torso.releaseLink(2);

        imgIn=NULL;

        return true;
    }

    /*********************************************************/
    bool updateModule()
    {
        if (online)
        {
            if (update_pose)
            {
                H_object=askForObjectPose();
                H_hand=askForHandPose();
            }
        }

        if (select_new_pose)
        {
            changeFrame();

            distanceFromHand();

            manipulability();

            showPoses();

            choosePose();
        }

        showPoses();

        if (online)
            return true;
        else
            return true;
    }

    /*********************************************************/
    bool close()
    {
        if (clientGazeCtrl.isValid())
            clientGazeCtrl.close();

        if (robotDevice_move.isValid())
            robotDevice_move.close();

        if (robotDevice2.isValid())
            robotDevice2.close();

        if (robotDevice3.isValid())
            robotDevice3.close();

        if (portPoseIn.asPort().isOpen())
            portPoseIn.close();

        if (portHandIn.asPort().isOpen())
            portHandIn.close();

        if (portRpc.asPort().isOpen())
            portRpc.close();

        if (!portImgIn.isClosed())
            portImgIn.close();

        if (!portImgOut.isClosed())
            portImgOut.close();
        return true;
    }

    /*********************************************************/
    bool interrupt()
    {
        if (portPoseIn.asPort().isOpen())
            portPoseIn.interrupt();

        if (portHandIn.asPort().isOpen())
            portHandIn.interrupt();

        portRpc.interrupt();

        if (!portImgIn.isClosed())
            portImgIn.interrupt();

        if (!portImgOut.isClosed())
            portImgOut.interrupt();

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

        cout<< " In cloud file "<<homeContextPath+"/"+filename<<endl;

        ifstream cloudFile((homeContextPath+"/"+filename).c_str());
        if (!cloudFile.is_open())
        {
            yError()<<" Problem opening cloud file!";
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

        if (norm(H_object.getCol(3).subVector(0,2))>0.0)
        {            
            x_axis_rotated.clear();
            y_axis_rotated.clear();
            z_axis_rotated.clear();
            positions_rotated.clear();

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
        }

        return true;
    }

    /*******************************************************************************/
    Matrix readPoseObjectAndHand(string &fileName)
    {
        Matrix H;
        int state=0;
        char line[255];

        cout<< " In pose file "<<homeContextPath+"/"+fileName<<endl;

        ifstream poseFile((homeContextPath+"/"+fileName).c_str());

        if (!poseFile.is_open())
        {
            yError()<<" Problem opening pose file!";
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

        cout<<" Pose read in "<<homeContextPath+"/"+fileName<<": "<<pos.toString()<<endl;

        if (euler)
            H=euler2dcm(euler_angles);
        else
            H=axis2dcm(axis);

        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,pos);
        H.setCol(3,x_aux);

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

        cout<<" Received pose: "<<pos.toString()<<" "<<euler_angles.toString()<<endl;

        H.resize(4,4);

        H=euler2dcm(euler_angles);

        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,pos);
        H.setCol(3,x_aux);

        if (norm(pos)>0.0)
            update_pose=false;

        cout<<" Homogeneous matrix of object pose: "<<H.toString()<<endl;
        return H;
    }

    /*******************************************************************************/
    bool showPoses()
    {
        if (imgIn==NULL)
        {
            imgIn=portImgIn.read(false);
            if (imgIn==NULL)
            {
                yError()<<" Please, connect the cameras!";
                return true;
            }
        }
        else
            imgIn=portImgIn.read();

        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut.resize(imgIn->width(),imgIn->height());

        cv::Mat imgInMat=cv::cvarrToMat((IplImage*)imgIn->getIplImage());
        cv::Mat imgOutMat=cv::cvarrToMat((IplImage*)imgOut.getIplImage());

        imgInMat.copyTo(imgOutMat);

        Vector position_2D(2,0.0);
        Vector axis_2D(2,0.0);

        int thickness=2;
        int font=0;
        double fontScale=0.5;
        int x_shift, y_shift;
        Vector num_position(3,0.0);


        if ( norm(pos)>0.0)
        {
            for (size_t i=0; i<positions_rotated.size(); i++)
            {
                cv::Scalar color(0,255,0);
                
                if (norm(index_poses)>0.0)
                {
                    color[0]+=-50*index_poses[i];
                    color[1]+= 10*index_poses[i];
                    if (index_poses[i]!=0.0)
                        color[2]=0;
                }

                stringstream i_string;
                i_string<<i;

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

                if (left_or_right=="right")
                    num_position=positions_rotated[i]+0.60*(y_axis_rotated[i]-positions_rotated[i])-0.40*(z_axis_rotated[i]-positions_rotated[i]);
                else
                    num_position=positions_rotated[i]+0.60*(y_axis_rotated[i]-positions_rotated[i])+0.40*(z_axis_rotated[i]-positions_rotated[i]);
                Vector num_position2D(2,0.0);
                igaze->get2DPixel(camera, num_position,num_position2D);
                cv::putText(imgOutMat, i_string.str(), cv::Point(num_position2D[0], num_position2D[1]), font, fontScale, color, thickness);
            }
        }

        if (index>=0 && norm(index_poses)>0.0)
        {
            cv::Scalar color(255,0,0);
            color[0]=-10*index_poses[index];
            color[1]=255 + 20*index_poses[index];
            if (index_poses[index]!=0.0)
                color[2]=0;

            stringstream i_string;
            i_string<<index;

            igaze->get2DPixel(camera, positions_rotated[index],position_2D);
            cv::Point pixel2D(position_2D[0],position_2D[1]);

            igaze->get2DPixel(camera, x_axis_rotated[index],axis_2D);
            cv::Point pixel_axis_x2D(axis_2D[0],axis_2D[1]);

            cv::line(imgOutMat,pixel2D,pixel_axis_x2D,cv::Scalar(255,0,0), 2);

            igaze->get2DPixel(camera, y_axis_rotated[index],axis_2D);
            cv::Point pixel_axis_y2D(axis_2D[0],axis_2D[1]);
            cv::line(imgOutMat,pixel2D,pixel_axis_y2D,cv::Scalar(0,255,0), 2);

            igaze->get2DPixel(camera, z_axis_rotated[index],axis_2D);
            cv::Point pixel_axis_z2D(axis_2D[0],axis_2D[1]);
            cv::line(imgOutMat,pixel2D,pixel_axis_z2D,cv::Scalar(0,0,255), 2);

            if (left_or_right=="right")
                num_position=positions_rotated[index]+0.60*(y_axis_rotated[index]-positions_rotated[index])-0.40*(z_axis_rotated[index]-positions_rotated[index]);
            else
                num_position=positions_rotated[index]+0.60*(y_axis_rotated[index]-positions_rotated[index])+0.40*(z_axis_rotated[index]-positions_rotated[index]);

            Vector num_position2D(2,0.0);
            igaze->get2DPixel(camera, num_position,num_position2D);
            cv::putText(imgOutMat, i_string.str(), cv::Point(num_position2D[0], num_position2D[1]), font, fontScale, color, thickness);

            color[0]=0;
            color[1]=0;
            color[2]=255;
            Vector center_bb(2,0.0);

            igaze->get2DPixel(camera, num_position,center_bb);
            cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

            igaze->get2DPixel(camera, xd_h, position_2D);
            cv::Point real_pixel2D(position_2D[0],position_2D[1]);

            Matrix orient(4,4);
            orient=axis2dcm(od_h);

            igaze->get2DPixel(camera, xd_h + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
            cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

            cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

            igaze->get2DPixel(camera, xd_h + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
            cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
            cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

            igaze->get2DPixel(camera,xd_h + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
            cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
            cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);
        }

        portImgOut.write();

        return true;
    }

    /*******************************************************************************/
    Matrix askForHandPose()
    {
        Matrix H;
        Bottle cmd,reply;
        cmd.addString("get_pose");

        if (portHandIn.write(cmd, reply))
        {         
            Bottle *bpos0=reply.get(0).asList();  
            for (size_t i=0; i<bpos0->size();i++)
            {               
                Bottle *bpos=bpos0->get(i).asList(); 

                if (bpos->get(0)=="position")
                {
                    pos_hand[0]=bpos->get(1).asDouble();
                    pos_hand[1]=bpos->get(2).asDouble();
                    pos_hand[2]=bpos->get(3).asDouble();
                }
                else if (bpos->get(0)=="orientation")
                {
                    axis_hand[0]=bpos->get(1).asDouble();
                    axis_hand[1]=bpos->get(2).asDouble();
                    axis_hand[2]=bpos->get(3).asDouble();
                    axis_hand[3]=bpos->get(4).asDouble();
                }    
            }
        }
        else
        {
            pos_hand[0]=pos_hand[1]=pos_hand[2]=axis_hand[0]=axis_hand[1]=axis_hand[2]=axis_hand[3];
        }

        yDebug()<<" Received hand pose: "<<pos_hand.toString()<<" "<<axis_hand.toString();
        H.resize(4,4);

        H=axis2dcm(axis_hand);

        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,pos_hand);
        H.setCol(3,x_aux);
        return H;
    }

    /*******************************************************************************/
    void distanceFromHand()
    {
        vector<double> distances(positions_rotated.size(), 0.0);

        for (size_t i=0; i<positions_rotated.size(); i++)
        {
             distances[i]=norm(positions_rotated[i] - pos_hand);
        }

        sort(distances.begin(), distances.end());

        for (size_t i=0; i<distances.size(); i++)
        {
            double count=-1;
            for (vector<double>::iterator it=distances.begin(); it!=distances.end(); ++it)
            {
                count++;
                if (norm(positions_rotated[i] - pos_hand)==*it)
                {
                    index_poses[i]= (count - 8);                    
                }
            }
        }

        yDebug()<<" Index poses: "<<index_poses.toString();
    }

    /*******************************************************************************/
    void manipulability()
    {
        Vector xdhat(3,0.0);
        Vector odhat(4,0.0);
        Vector qdhat(10,0.0);
        Vector err_orient;
        Vector err_pos;
        bool first_time=true;
        vector<double> manip;
        vector<double> manip_notordered;

        cout<<endl<<" Computing manipulability..."<<endl<<endl;

        for (size_t i=0; i<positions_rotated.size(); i++)
        {
            Matrix orient(3,3);
            orient.setCol(0,(x_axis_rotated[i]-positions_rotated[i])/norm(x_axis_rotated[i]-positions_rotated[i]));
            orient.setCol(1,(y_axis_rotated[i]-positions_rotated[i])/norm(y_axis_rotated[i]-positions_rotated[i]));
            orient.setCol(2,(z_axis_rotated[i]-positions_rotated[i])/norm(z_axis_rotated[i]-positions_rotated[i]));

            yDebug()<<" Desired rotation matrix";
            cout<<orient.toString()<<endl<<endl;

            od.push_back(dcm2axis(orient));

            icart_arm_move->askForPose(positions_rotated[i], od[i], xdhat, odhat, qdhat);

            Matrix orient_hat(4,4);
            orient_hat=axis2dcm(odhat);

            yDebug()<<" Computed rotation matrix ";
            cout<<orient_hat.toString()<<endl<<endl;

            //err_orient.push_back(norm(od[i].subVector(0,2)-odhat.subVector(0,2)+ abs(fmod(od[i][3]-odhat[3], 2*M_PI))));
            err_orient.push_back(norm(orient_hat.getCol(0).subVector(0,2)-orient.getCol(0).subVector(0,2))+
                                      norm(orient_hat.getCol(1).subVector(0,2)-orient.getCol(1).subVector(0,2))+
                                           norm(orient_hat.getCol(2).subVector(0,2)-orient.getCol(2).subVector(0,2)));

            err_pos.push_back(norm(positions_rotated[i]-xdhat));

            yDebug()<<" Error in orientation for pose "<<i<<": "<<err_orient[i];
            yDebug()<<" Error in position "<<i<<": "<<err_pos[i];
            yDebug()<<" Qd for pose "<<i<<": "<<qdhat.toString();
            cout<<endl;

            Matrix J=ikin_arm.GeoJacobian(qdhat);

            manip.push_back(sqrt(det(J*J.transposed())));
        }

        manip_notordered=manip;
        sort(manip.begin(), manip.end());

        for (size_t i=0; i<manip_notordered.size(); i++)
        {
            int count=0;
            yDebug()<<" Manipulability for pose "<<i<<": "<<manip_notordered[i];
            first_time=true;
            for (std::vector<double>::iterator it=manip.begin(); it!=manip.end(); ++it)
            {                
                if (manip_notordered[i]==*it && first_time)
                {                   
                    index_poses[i] += (count-8)*(err_orient[i]+err_pos[i]);
                    first_time=false;
                }
                count++;
            }

            cout<<endl;
        }

        cout<<" Index poses after manipulability: "<<index_poses.toString()<<endl<<endl;
    }

    /*******************************************************************************/
    bool choosePose()
    {
        double tmp=-100.0;
        int count=0;
        Vector qdhat(10,0.0);

        for(size_t i=0; i<index_poses.size(); i++)
        {
            if (tmp<index_poses[i])
            {
                tmp=index_poses[i];
                index=count;
            }
            count++;
        }
        yDebug()<<" Selected pose: "<<index;

        if (norm(pos)>0.0)
        {
            icart_arm_move->askForPose(positions_rotated[index], od[index], xd_h, od_h, qdhat);

            select_new_pose=false;
        }
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
