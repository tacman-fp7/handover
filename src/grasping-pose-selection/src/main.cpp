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
#include <iCub/ctrl/minJerkCtrl.h>
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
using namespace iCub::ctrl;

#include "src/poseSelection_IDL.h"

class poseSelection : public RFModule,
                      public poseSelection_IDL
{  
    Matrix H_object, H_hand;

    vector<Vector> correction_matrix;

    Vector xd_h, od_h;
    Vector index_poses;
    Vector pose_second;
    Vector position_new;
    Vector position_old;
    Vector pose_second_corr;
    Vector pos_hand, axis_hand;
    Vector pos, euler_angles, axis;
    Vector x_init_moving_arm, o_init_moving_arm;
    Vector x_init_resting_arm, o_init_resting_arm;

    vector<Vector> od;
    vector<Vector> odhat;
    vector<Vector> xdhat;
    vector<Vector> qdhat;
    vector<double> manip;
    vector<Vector> odhat_wp;
    vector<Vector> xdhat_wp;
    vector<Vector> positions;
    vector<Vector> waypoints;
    vector<Vector> example_poses;
    vector<Vector> orientations;
    vector<Vector> first_arm_pose;
    vector<Vector> positions_rotated;    
    vector<Vector> x_axis, y_axis, z_axis;
    vector<Vector> x_axis_wp, y_axis_wp, z_axis_wp;
    vector<Vector> x_axis_rotated, y_axis_rotated, z_axis_rotated;

    string orientationFileName;
    string computedPosesFileName;
    string correctionMatrixFileName;
    string objectPoseFileName;
    string handPoseFileName;
    string positionFileName;
    string homeContextPath;
    string left_or_right;
    string module_name;
    string frame;
    string robot;

    bool select_defined_pose;
    bool reach_final_pose;
    bool update_hand_pose;
    bool reached_waypoint;
    bool select_new_pose;
    bool reach_waypoint;
    bool torso_enabled;
    bool correct_pose;
    bool reached_final;
    bool change_frame;
    bool closed_chain;
    bool update_pose;    
    bool twist_wrist;
    bool to_be_sent;
    bool use_matrix;
    bool new_angle;
    bool waypoint;
    bool manip_ok;
    bool correct;
    bool online;
    bool euler;

    double theta;
    double length;
    double tolerance;
    double y_corr;
    double offset_z_final;
    double offset_x_approach;
    double offset_x_final;
    double offset_y_final;
    double offset_z_approach;

    int index;
    int camera;
    int num_obj;
    int num_objs;
    int n_waypoint;
    int init_num_pos;
    int current_waypoint;
    int startup_context_id;

    deque<IControlLimits*> lim_deque1;
    deque<IControlLimits*> lim_deque2;

    iCubArm ikin_second_arm;
    iCubArm ikin_first_arm;
    iCubTorso ikin_torso;
    iKinChain *chain;
    IGazeControl *igaze;    
    IControlLimits* lim_torso;
    IControlLimits* lim_first_arm;
    IControlLimits* lim_second_arm;
    ICartesianControl *icart_arm_move;
    IControlMode2     *ctrlmode;

    PolyDriver clientGazeCtrl;
    PolyDriver robotDevice_move;
    PolyDriver robotDevice2;
    PolyDriver robotDevice3;
    PolyDriver robotDevice4;

    RpcClient portPoseIn;
    RpcClient portHandIn;
    RpcClient portClosedChain;

    RpcServer portRpc;

    Bottle reply;

    ResourceFinder *rf;

    ImageOf<PixelRgb> *imgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    BufferedPort<ImageOf<PixelRgb> > portImgOutReviewer;

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
        Time::delay(2.0);
        if (index>=0)
            return true;
        else
            return false;
    }

    /************************************************************************/
    bool choose_new_pose()
    {
        select_new_pose=true;
        return true;
    }

    /************************************************************************/
    bool select_pose(int entry)
    {
        if (entry>=0 && entry<positions.size())
        {
            Vector qdhat(10,0.0);
            index=entry;

            if (!closed_chain)
                icart_arm_move->askForPose(positions_rotated[index], od[index], xd_h, od_h, qdhat);

            select_new_pose=false;
            select_defined_pose=true;
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    Bottle get_pose()
    {
        reply.clear();

        if (to_be_sent=true)
        {
            Vector pos_in_hand(4,1.0);

            pos_in_hand.setSubvector(0,positions_rotated[index]);
            pos_in_hand=SE3inv(H_hand)*pos_in_hand;
            reply.addDouble(pos_in_hand[0]); reply.addDouble(pos_in_hand[1]); reply.addDouble(pos_in_hand[2]);

            Vector od_in_hand(4,0.0);
            Matrix aux(4,4);
            aux.zero();
            aux(2,0)=aux(1,2)=-1.0;
            aux(0,1)=1.0;
            od_in_hand=dcm2axis(aux*SE3inv(H_hand)*axis2dcm(odhat[index]));

            reply.addDouble(od_in_hand[0]); reply.addDouble(od_in_hand[1]); reply.addDouble(od_in_hand[2]); reply.addDouble(od_in_hand[3]);
        }

        return reply;
    }

    /************************************************************************/
    int get_index()
    {
        if (norm(first_arm_pose[index].subVector(0,2))>0.0 && norm(xdhat[index])>0.0)
            return index;
        else
            return 1000;
    }

    /************************************************************************/
    int get_n_waypoint()
    {
        return n_waypoint;
    }

    /************************************************************************/
    bool set_n_waypoint(const int entry)
    {
        n_waypoint=entry;
    }

    /************************************************************************/
    Bottle get_Hhand()
    {
        Bottle replyh;

        replyh.addDouble(H_hand(0,0)); replyh.addDouble(H_hand(0,1)); replyh.addDouble(H_hand(0,2)); replyh.addDouble(H_hand(0,3));
        replyh.addDouble(H_hand(1,0)); replyh.addDouble(H_hand(1,1)); replyh.addDouble(H_hand(1,2)); replyh.addDouble(H_hand(1,3));
        replyh.addDouble(H_hand(2,0)); replyh.addDouble(H_hand(2,1)); replyh.addDouble(H_hand(2,2)); replyh.addDouble(H_hand(2,3));

        return replyh;
    }

    /************************************************************************/
    Bottle get_pose_moving_arm()
    {
        Bottle reply;
        Vector pos(4,1.0);
        Vector pos_aux(3,0.0);
        Vector orie(4,0.0);

        icart_arm_move->getPose(pos_aux,orie);
        pos.setSubvector(0,pos_aux);

        reply.addDouble(pos[0]); reply.addDouble(pos[1]); reply.addDouble(pos[2]);
        reply.addDouble(orie[0]); reply.addDouble(orie[1]); reply.addDouble(orie[2]); reply.addDouble(orie[3]);

        return reply;
    }

    /************************************************************************/
    string get_moving_arm()
    {
        return left_or_right;
    }

    /************************************************************************/
    bool update_pose_hand()
    {
        update_hand_pose=true;
        to_be_sent=false;
        return true;
    }

    /************************************************************************/
    bool set_offset_z_final(double entry)
    {
        cout<<endl<< "  New offset_z_final set: "<<entry<<endl<<endl;
        offset_z_final=entry;
        return true;
    }

    /************************************************************************/
    double get_offset_z_final()
    {
        return offset_z_final;
    }

    /************************************************************************/
    bool set_offset_x_approach(double entry)
    {
        cout<<endl<< "  New offset_x_approach set: "<<entry<<endl<<endl;
        offset_x_approach=entry;
        return true;
    }

    /************************************************************************/
    double get_offset_x_approach()
    {
        return offset_x_approach;
    }

    /************************************************************************/
    bool set_offset_x_final(double entry)
    {
        cout<<endl<< "  New offset_x_final set: "<<entry<<endl<<endl;
        offset_x_final=entry;
        return true;
    }

    /************************************************************************/
    double get_offset_x_final()
    {
        return offset_x_final;
    }

    /************************************************************************/
    bool set_offset_y_final(double entry)
    {
        cout<<endl<< "  New offset_x_final set: "<<entry<<endl<<endl;
        offset_y_final=entry;
        return true;
    }

    /************************************************************************/
    double get_offset_y_final()
    {
        return offset_y_final;
    }

    /************************************************************************/
    bool set_offset_z_approach(double entry)
    {
        cout<<endl<< "  New offset_z_approach set: "<<entry<<endl<<endl;
       offset_z_approach=entry;
       return true;
    }

    /************************************************************************/
    double get_offset_z_approach()
    {
        return offset_z_approach;
    }

    /************************************************************************/
    bool set_angle(double entry)
    {
        cout<<endl<< "  New angle for wrist set: "<<entry<<endl<<endl;

        theta=entry;

        new_angle=true;

        return true;
    }

    /************************************************************************/
    double get_angle()
    {
        return theta;
    }

    /************************************************************************/
    bool set_waypoint(const int entry)
    {
        if (entry<n_waypoint)
        {
            current_waypoint=entry;
            reach_waypoint=true;
            Time::delay(2.5);
            if (reached_waypoint)
                return true;
            else
                return false;
        }
        else
        {
            reach_waypoint=false;
            return false;
        }
    }

    /************************************************************************/
    bool reach_final()
    {
        reach_final_pose=true;
        Time::delay(2.5);
        if (reached_final)
            return true;
        else
            return false;
    }

    /************************************************************************/
    bool set_tolerance(double entry)
    {
        cout<<endl<< "  New tolerance set: "<<entry<<endl<<endl;

        tolerance=entry;

        return true;
    }

    /************************************************************************/
    double get_tolerance()
    {
        return tolerance;
    }

    /************************************************************************/
    bool set_y_correction(double entry)
    {
        cout<<endl<< "  New y offset: "<<entry<<endl<<endl;

        y_corr=entry;
        correct=true;

        return true;
    }

    /************************************************************************/
    double get_y_correction()
    {
        if (correct)
            return y_corr;
    }

    /*********************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        homeContextPath=rf.getHomeContextPath().c_str();
        module_name=rf.check("module_name", Value("pose-selection"), "Getting module name").asString();

        num_objs=rf.check("num_objs", Value(1)).asInt();

        handPoseFileName=rf.check("handPoseFileName", Value("hand-pose.txt"), "Hand pose").asString();
        positionFileName=rf.check("positionFileName", Value("positions0.off"), "Default positions file name").asString();
        objectPoseFileName=rf.check("objectPoseFileName", Value("object-pose.txt"), "Default object pose file name").asString();
        orientationFileName=rf.check("orientationFileName", Value("orientations-right.txt"), "Default orientations file name").asString();
        correctionMatrixFileName=rf.check("correctionMatrixFileName", Value("correction-matrices.txt"), "Default orientations file name").asString();
        computedPosesFileName=rf.check("computedPosesFileName", Value("computed-poses.txt"), "Default orientations file name").asString();


        online=(rf.check("online", Value("yes"), "online or offline processing").asString()== "yes");
        camera=(rf.check("camera", Value(0), "online or offline processing").asInt());
        torso_enabled=(rf.check("torso_enabled", Value("no")).asString()== "yes");
        twist_wrist=(rf.check("twist_wrist", Value("yes")).asString()== "yes");
        closed_chain=(rf.check("closed_chain", Value("no")).asString()== "yes");
        waypoint=(rf.check("use_waypoint", Value("yes")).asString()== "yes");
        correct=(rf.check("correction", Value("no")).asString()== "yes");
        correct_pose=(rf.check("correct_pose", Value("no")).asString()== "yes");

        robot=rf.check("robot", Value("icubSim")).asString();
        left_or_right=rf.check("which_hand", Value("left")).asString();

        n_waypoint=rf.check("n_waypoint", Value(1)).asInt();

        if (left_or_right=="left")
            theta=rf.check("theta", Value(20.0)).asDouble();
        else
            theta=rf.check("theta", Value(-20.0)).asDouble();

        offset_z_final=rf.check("offset_z_final", Value(0.02)).asDouble();
        y_corr=rf.check("y_corr", Value(0.0)).asDouble();
        offset_x_approach=rf.check("offset_x_approach", Value(0.08)).asDouble();
        offset_x_final=rf.check("offset_x_final", Value(0.02)).asDouble();
        offset_z_approach=rf.check("offset_z_approach", Value(0.04)).asDouble();
        tolerance=rf.check("tolerance", Value(0.02)).asDouble();

        offset_y_final=rf.check("offset_y_final", Value(0.0)).asDouble();


        cout<< " An offset_z_final of "<<offset_z_final<< " will be added in order to shift poses along z-axis of hand frame"<<endl;
        cout<< " An offset_x_approach of "<<offset_x_approach<< " will be added in order to shift poses along x-axis of hand frame during approach"<<endl;
        cout<< " An offset_x_final of "<<offset_x_final<< " will be added in order to shift final poses along x-axis of hand frame "<<endl<<endl;

        length=0.06;

        pos.resize(3,0.0);
        xd_h.resize(3,0.0);
        od_h.resize(4,0.0);
        axis.resize(4,0.0);
        H_hand.resize(4,4);
        H_object.resize(4,4);
        pos_hand.resize(3,0.0);
        axis_hand.resize(4,0.0);
        pose_second.resize(7,0.0);
        euler_angles.resize(3,0.0);
        position_new.resize(3,0.0);
        position_old.resize(3,0.0);
        pose_second_corr.resize(7,0.0);
        x_init_moving_arm.resize(3,0.0);
        o_init_moving_arm.resize(4,0.0);
        x_init_resting_arm.resize(3,0.0);
        o_init_resting_arm.resize(4,0.0);

        readPoses(positionFileName, orientationFileName);
        read(correctionMatrixFileName, "corr_matrix");
        read(computedPosesFileName, "example_poses");

        for (size_t i=0; i<correction_matrix.size(); i++)
        {
            yDebug()<<" Correction matrix "<<correction_matrix[i].toString();
            yDebug()<<" Example poses "<<example_poses[i].toString();
        }

        index_poses.resize(positions.size(), 0.0);

        index=-1;
        num_obj=0;

        update_hand_pose=false;
        to_be_sent=true;
        manip_ok=false;
        select_new_pose=false;
        update_pose=false;
        reach_waypoint=false;
        reach_final_pose=false;
        select_defined_pose=false;

        reached_waypoint=false;
        reached_final=false;

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
        portImgOutReviewer.open("/" + module_name + "/img:r");

        if (closed_chain)
            portClosedChain.open("/"+module_name+"/cc:rpc");

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

        //igaze->setTrackingMode(true);

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
        {
            ikin_second_arm=iCubArm("right_v2");
            ikin_first_arm=iCubArm("left_v2");

        }
        else
        {
            ikin_second_arm=iCubArm("left_v2");
            ikin_first_arm=iCubArm("right_v2");
        }

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

        robotDevice2.view(lim_second_arm);
        robotDevice2.view(ctrlmode);

        Property option_arm4("(device remote_controlboard)");
        if (left_or_right=="left")
        {
            option_arm4.put("remote","/"+robot+"/right_arm");
            option_arm4.put("local","/"+module_name+"/joint/right_arm");
        }
        else
        {
            option_arm4.put("remote","/"+robot+"/left_arm");
            option_arm4.put("local","/"+module_name+"/joint/left_arm");
        }


        robotDevice4.open(option_arm4);
        if (!robotDevice4.isValid())
        {
            yError(" Device 4 not available!");
            return false;
        }

        robotDevice4.view(lim_first_arm);

        Property option_arm3("(device remote_controlboard)");
        option_arm3.put("remote","/"+robot+"/torso");
        option_arm3.put("local","/"+module_name+"/joint/torso");

        robotDevice3.open(option_arm3);
        if (!robotDevice3.isValid())
        {
            yError(" Device 3 not available!");
            return false;
        }

        robotDevice3.view(lim_torso);

        lim_deque1.push_back(lim_torso);
        lim_deque1.push_back(lim_second_arm);

        lim_deque2.push_back(lim_torso);
        lim_deque2.push_back(lim_first_arm);

        if (!ikin_second_arm.alignJointsBounds(lim_deque1))
            yError(" Problems in alignJointsBounds()");

        if (!ikin_first_arm.alignJointsBounds(lim_deque2))
            yError(" Problems in alignJointsBounds()");

        imgIn=NULL;

        return true;
    }

    /*********************************************************/
    bool updateModule()
    {
        if (select_new_pose == true || update_hand_pose ==true)
        {
            cout<< endl<< " =================================="
                          "==================================="
                          "==================================="
                          "=================================== "<<endl<<endl;
        }

        if (online)
        {
            if (update_pose)
            {
                H_object=askForObjectPose();
                H_hand=askForHandPose();
            }

            //if (update_hand_pose)
                H_hand=askForHandPose();
        }

        if (select_new_pose)
        {
            H_hand=askForHandPose();

            changeFrame();

            distanceFromHand();

            if (!closed_chain)
                manipulability();
            else
                manip_ok=manipulabilityClosedChain();

            if (manip_ok)
                choosePose();

            if (waypoint && !correct_pose)
                addWaypoint(n_waypoint, index, pose_second);
            else if (waypoint && correct_pose)
                addWaypoint(n_waypoint, index, pose_second_corr);
        }

        if (select_defined_pose)
        {
            H_hand=askForHandPose();

            askXdOdHat();

            chooseSecondHandPose();

            if (waypoint && !correct_pose)
                addWaypoint(n_waypoint, index, pose_second);
            else if (waypoint && correct_pose)
                addWaypoint(n_waypoint, index, pose_second_corr);
        }

        if (update_hand_pose)
        {
            changeFrame();

            if (manip_ok)
                choosePose();

            if (waypoint && !correct_pose)
                addWaypoint(n_waypoint, index, pose_second);
            else if (waypoint && correct_pose)
                addWaypoint(n_waypoint, index, pose_second_corr);

            showPoses();
        }

        if (reach_waypoint)
            reachWaypointVel(current_waypoint);

        if (reach_final_pose)
            reachFinalPoint();

        showPoses();


        showPosesReviewer();

        if (online)
            return true;
        else
            return true;

        cout<< "CIAOOOOO"<<endl;
    }

//    /*******************************************************************************/
//    double getPeriod()
//    {
//        return 0.1;
//    }


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

        if (robotDevice4.isValid())
            robotDevice4.close();

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

        if (!portImgOutReviewer.isClosed())
            portImgOutReviewer.close();

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

        init_num_pos=positions.size();

        if (twist_wrist)
            rotatePoses(theta*iCub::ctrl::CTRL_DEG2RAD);

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
        else if (tag=="corr_matrix")
            correction_matrix.clear();
        else if (tag=="example_poses")
            example_poses.clear();

        Vector point_tmp;

        if (tag=="positions")
            point_tmp.resize(6,0.0);
        else if (tag=="orientations")
            point_tmp.resize(9,0.0);
        else if (tag=="corr_matrix")
            point_tmp.resize(16,0.0);
        else if (tag=="example_poses")
            point_tmp.resize(7,0.0);

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
                    cout<<"n Points "<<nPoints<<endl;
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

                    if (tag=="example_poses")
                        point_tmp[6]=b.get(6).asDouble();

                    if (tag=="orientations" || tag=="corr_matrix")
                    {
                        point_tmp[6]=b.get(6).asDouble();
                        point_tmp[7]=b.get(7).asDouble();
                        point_tmp[8]=b.get(8).asDouble();
                    }
                    if (tag=="corr_matrix")
                    {
                        point_tmp[9]=b.get(9).asDouble();
                        point_tmp[10]=b.get(10).asDouble();
                        point_tmp[11]=b.get(11).asDouble();
                        point_tmp[12]=b.get(12).asDouble();
                        point_tmp[13]=b.get(13).asDouble();
                        point_tmp[14]=b.get(14).asDouble();
                        point_tmp[15]=b.get(15).asDouble();
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
                            else if (tag=="corr_matrix")
                                correction_matrix.push_back(point_tmp);
                            else if (tag=="example_poses")
                                example_poses.push_back(point_tmp);
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

        if (new_angle)
        {
            rotatePoses(theta*iCub::ctrl::CTRL_DEG2RAD);
        }

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

            for (size_t i=0; i<positions.size(); i++)
            {
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

            //if (offset_z_final > 0.0)
            //{
                addOffset(positions_rotated,z_axis_rotated, positions_rotated,offset_z_final, "z");

                addOffset(z_axis_rotated, z_axis_rotated,positions_rotated,offset_z_final, "z");
                addOffset(x_axis_rotated, z_axis_rotated,positions_rotated, offset_z_final, "z");
                addOffset(y_axis_rotated, z_axis_rotated,positions_rotated, offset_z_final, "z");
            //}

            //if (offset_x_final > 0.0)
            //{
                addOffset(positions_rotated,x_axis_rotated,positions_rotated, offset_x_final, "x");

                addOffset(x_axis_rotated, x_axis_rotated,positions_rotated,offset_x_final, "x");
                addOffset(z_axis_rotated, x_axis_rotated,positions_rotated, offset_x_final, "x");
                addOffset(y_axis_rotated, x_axis_rotated,positions_rotated, offset_x_final, "x");


                addOffset(positions_rotated,y_axis_rotated, positions_rotated,offset_y_final, "y");

                addOffset(y_axis_rotated, y_axis_rotated,positions_rotated,offset_y_final, "y");
                addOffset(z_axis_rotated, y_axis_rotated,positions_rotated, offset_y_final, "y");
                addOffset(x_axis_rotated, y_axis_rotated,positions_rotated, offset_y_final, "y");
            //}

        }

        update_hand_pose=false;

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

        yDebug()<<" Pose read in "<<homeContextPath+"/"+fileName<<": "<<pos.toString(3,3);

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
            num_obj=rec->get(6).asInt();
        }
        else
        {
            pos[0]=pos[1]=pos[2]=euler_angles[0]=euler_angles[1]=euler_angles[2]=0.0;
        }

        yDebug()<<" Received pose: "<<pos.toString(3,3)<<" "<<euler_angles.toString(3,3);

        H.resize(4,4);

        H=euler2dcm(euler_angles);

        Vector x_aux(4,1.0);
        x_aux.setSubvector(0,pos);
        H.setCol(3,x_aux);

        if (norm(pos)>0.0)
            update_pose=false;

//        if (num_obj<num_objs)
//        {
//            stringstream ss2;
//            ss2 << "positions"<<num_obj<<".off";
//            string str_num_obj = ss2.str();

//            cout<<" Object selected "<< num_obj<<endl;

//            readPoses(str_num_obj, orientationFileName);
//        }

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

        if (closed_chain && norm(index_poses)>0.0)
        {
            H_hand=askForHandPose();

            changeFrame();
        }

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

            if (!closed_chain)
            {
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
            else if (xdhat.size()>0)
            {
                igaze->get2DPixel(camera, num_position,center_bb);
                cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

                igaze->get2DPixel(camera, xdhat[index], position_2D);
                cv::Point real_pixel2D(position_2D[0],position_2D[1]);
                Matrix orient(4,4);
                orient=axis2dcm(odhat[index]);


                igaze->get2DPixel(camera, xdhat[index] + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
                cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(200,0,0), 2);

                igaze->get2DPixel(camera, xdhat[index] + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
                cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,200,0), 2);

                igaze->get2DPixel(camera,xdhat[index] + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
                cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,200), 2);

                /*for (size_t i=0; i<n_waypoint;i++)
                {

                    igaze->get2DPixel(camera, waypoints[i], position_2D);
                    cv::Point real_pixel2D(position_2D[0],position_2D[1]);
                    Matrix orient(3,3);
                    orient.setCol(0,(x_axis_wp[i]-waypoints[i])/norm(x_axis_wp[i]-waypoints[i]));
                    orient.setCol(1,(y_axis_wp[i]-waypoints[i])/norm(y_axis_wp[i]-waypoints[i]));
                    orient.setCol(2,(z_axis_wp[i]-waypoints[i])/norm(z_axis_wp[i]-waypoints[i]));

                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *orient.getCol(0),axis_2D);
                    cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *orient.getCol(1),axis_2D);
                    cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

                    igaze->get2DPixel(camera,waypoints[i] + 0.05 *orient.getCol(2),axis_2D);
                    cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);
                }*/
            }
//            else
//            {
//                igaze->get2DPixel(camera, num_position,center_bb);
//                cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

//                igaze->get2DPixel(camera, pose_second.subVector(0,2), position_2D);
//                cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                Matrix orient(4,4);
//                orient=axis2dcm(pose_second.subVector(3,6));

//                igaze->get2DPixel(camera, pose_second.subVector(0,2) + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                igaze->get2DPixel(camera, pose_second.subVector(0,2) + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                igaze->get2DPixel(camera,pose_second.subVector(0,2) + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);

//                for (size_t i=0; i<n_waypoint;i++)
//                {

//                    igaze->get2DPixel(camera, waypoints[i], position_2D);
//                    cv::Point real_pixel2D(position_2D[0],position_2D[1]);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *x_axis_wp[i],axis_2D);
//                    cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *y_axis_wp[i],axis_2D);
//                    cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *z_axis_wp[i],axis_2D);
//                    cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);
//                }

//                if (correct_pose)
//                {
//                    igaze->get2DPixel(camera, num_position,center_bb);
//                    cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

//                    igaze->get2DPixel(camera, pose_second_corr.subVector(0,2), position_2D);
//                    cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                    Matrix orient(4,4);
//                    orient=axis2dcm(pose_second_corr.subVector(3,6));

//                    igaze->get2DPixel(camera, pose_second_corr.subVector(0,2) + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
//                    cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(200,0,0), 2);

//                    igaze->get2DPixel(camera, pose_second_corr.subVector(0,2) + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
//                    cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,200,0), 2);

//                    igaze->get2DPixel(camera,pose_second_corr.subVector(0,2) + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
//                    cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,200), 2);
//                }
//            }
        }

        portImgOut.write();

        return true;
    }

    /*******************************************************************************/
    bool showPosesReviewer()
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

        ImageOf<PixelRgb> &imgOutR=portImgOutReviewer.prepare();
        imgOutR.resize(imgIn->width(),imgIn->height());

        cv::Mat imgInMat=cv::cvarrToMat((IplImage*)imgIn->getIplImage());
        cv::Mat imgOutMat=cv::cvarrToMat((IplImage*)imgOutR.getIplImage());

        imgInMat.copyTo(imgOutMat);

        Vector position_2D(2,0.0);
        Vector axis_2D(2,0.0);

        int thickness=2;
        int font=0;
        double fontScale=0.5;
        int x_shift, y_shift;
        Vector num_position(3,0.0);

        if (closed_chain && norm(index_poses)>0.0)
        {
            H_hand=askForHandPose();

            changeFrame();
        }

        if ( norm(pos)>0.0 && norm(position_new - position_old)<=0.005)
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

        if (index>=0 && norm(index_poses)>0.0 && norm(position_new - position_old)<=0.005)
        {
            //position_new=position_old;
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

//            if (!closed_chain)
//            {
//                igaze->get2DPixel(camera, num_position,center_bb);
//                cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

//                igaze->get2DPixel(camera, xd_h, position_2D);
//                cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                Matrix orient(4,4);
//                orient=axis2dcm(od_h);

//                igaze->get2DPixel(camera, xd_h + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                igaze->get2DPixel(camera, xd_h + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                igaze->get2DPixel(camera,xd_h + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);
//            }
//            else if (xdhat.size()>0)
//            {
//                igaze->get2DPixel(camera, num_position,center_bb);
//                cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

//                igaze->get2DPixel(camera, xdhat[index], position_2D);
//                cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                Matrix orient(4,4);
//                orient=axis2dcm(odhat[index]);


//                igaze->get2DPixel(camera, xdhat[index] + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                igaze->get2DPixel(camera, xdhat[index] + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                igaze->get2DPixel(camera,xdhat[index] + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);

//                for (size_t i=0; i<n_waypoint;i++)
//                {

//                    igaze->get2DPixel(camera, waypoints[i], position_2D);
//                    cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                    Matrix orient(3,3);
//                    orient.setCol(0,(x_axis_wp[i]-waypoints[i])/norm(x_axis_wp[i]-waypoints[i]));
//                    orient.setCol(1,(y_axis_wp[i]-waypoints[i])/norm(y_axis_wp[i]-waypoints[i]));
//                    orient.setCol(2,(z_axis_wp[i]-waypoints[i])/norm(z_axis_wp[i]-waypoints[i]));

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *orient.getCol(0),axis_2D);
//                    cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *orient.getCol(1),axis_2D);
//                    cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                    igaze->get2DPixel(camera,waypoints[i] + 0.05 *orient.getCol(2),axis_2D);
//                    cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);
//                }
//            }
//            else
//            {
//                igaze->get2DPixel(camera, num_position,center_bb);
//                cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

//                igaze->get2DPixel(camera, pose_second.subVector(0,2), position_2D);
//                cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                Matrix orient(4,4);
//                orient=axis2dcm(pose_second.subVector(3,6));

//                igaze->get2DPixel(camera, pose_second.subVector(0,2) + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                igaze->get2DPixel(camera, pose_second.subVector(0,2) + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                igaze->get2DPixel(camera,pose_second.subVector(0,2) + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
//                cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);

//                for (size_t i=0; i<n_waypoint;i++)
//                {

//                    igaze->get2DPixel(camera, waypoints[i], position_2D);
//                    cv::Point real_pixel2D(position_2D[0],position_2D[1]);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *x_axis_wp[i],axis_2D);
//                    cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(255,0,0), 2);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *y_axis_wp[i],axis_2D);
//                    cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,255,0), 2);

//                    igaze->get2DPixel(camera, waypoints[i] + 0.05 *z_axis_wp[i],axis_2D);
//                    cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,255), 2);
//                }

//                if (correct_pose)
//                {
//                    igaze->get2DPixel(camera, num_position,center_bb);
//                    cv::rectangle(imgOutMat, cv::Point(center_bb[0]-10, center_bb[1]-20),cv::Point(center_bb[0]+20, center_bb[1]+10), color, 2, 8 );

//                    igaze->get2DPixel(camera, pose_second_corr.subVector(0,2), position_2D);
//                    cv::Point real_pixel2D(position_2D[0],position_2D[1]);
//                    Matrix orient(4,4);
//                    orient=axis2dcm(pose_second_corr.subVector(3,6));

//                    igaze->get2DPixel(camera, pose_second_corr.subVector(0,2) + 0.05 *orient.getCol(0).subVector(0,2),axis_2D);
//                    cv::Point real_pixel_axis_x2D(axis_2D[0],axis_2D[1]);

//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_x2D,cv::Scalar(200,0,0), 2);

//                    igaze->get2DPixel(camera, pose_second_corr.subVector(0,2) + 0.05 *orient.getCol(1).subVector(0,2),axis_2D);
//                    cv::Point real_pixel_axis_y2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_y2D,cv::Scalar(0,200,0), 2);

//                    igaze->get2DPixel(camera,pose_second_corr.subVector(0,2) + 0.05 *orient.getCol(2).subVector(0,2),axis_2D);
//                    cv::Point real_pixel_axis_z2D(axis_2D[0],axis_2D[1]);
//                    cv::line(imgOutMat,real_pixel2D,real_pixel_axis_z2D,cv::Scalar(0,0,200), 2);
//                }
//                }
        }

        portImgOutReviewer.write();

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
            select_new_pose=false;
            yError()<< " Some problems in receiving hand pose";
        }

        //cout<< "New pose "<<pos_hand.toString()<<" "<<axis_hand.toString()<<endl;

        position_new=pos_hand;

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
        if (positions_rotated.size()>0)
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
                        index_poses[i]= (count - positions_rotated.size());
                    }
                }
            }

            yDebug()<<" Index poses: "<<index_poses.toString(3,3);
        }
    }

    /*******************************************************************************/
    void manipulability()
    {
        Vector xdhat(3,0.0);
        Vector odhat_tmp(4,0.0);
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

            od.push_back(dcm2axis(orient));

            icart_arm_move->askForPose(positions_rotated[i], od[i], xdhat, odhat_tmp, qdhat);

            odhat.push_back(odhat_tmp);

            Matrix orient_hat(4,4);
            orient_hat=axis2dcm(odhat[i]);

            //err_orient.push_back(norm(od[i].subVector(0,2)-odhat.subVector(0,2)+ abs(fmod(od[i][3]-odhat[3], 2*M_PI))));
            err_orient.push_back(norm(orient_hat.getCol(0).subVector(0,2)-orient.getCol(0).subVector(0,2))+
                                      norm(orient_hat.getCol(1).subVector(0,2)-orient.getCol(1).subVector(0,2))+
                                           norm(orient_hat.getCol(2).subVector(0,2)-orient.getCol(2).subVector(0,2)));

            err_pos.push_back(norm(positions_rotated[i]-xdhat));

            yDebug()<<" Error in orientation for pose "<<i<<": "<<err_orient[i];
            yDebug()<<" Error in position "<<i<<": "<<err_pos[i];
            yDebug()<<" Qd for pose "<<i<<": "<<qdhat.toString(3,3);
            cout<<endl;

            Matrix J=ikin_second_arm.GeoJacobian(qdhat);

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
                    index_poses[i] += (count-positions_rotated.size())*(err_orient[i]+err_pos[i]);
                    first_time=false;
                }
                count++;
            }

            cout<<endl;
        }

        yInfo()<<" Index poses after manipulability: "<<index_poses.toString(3,3);
    }

    /*******************************************************************************/
    bool manipulabilityClosedChain()
    {
        Vector err_orient;
        Vector err_pos;
        bool first_time=true;
        bool go_on=false;
        vector<double> manip_notordered;
        od.clear();

        cout<<endl<<" Computing manipulability..."<<endl<<endl;

        Matrix orient(3,3);

        for (size_t i=0; i<positions_rotated.size(); i++)
        {
            orient.setCol(0,(x_axis_rotated[i]-positions_rotated[i])/norm(x_axis_rotated[i]-positions_rotated[i]));
            orient.setCol(1,(y_axis_rotated[i]-positions_rotated[i])/norm(y_axis_rotated[i]-positions_rotated[i]));
            orient.setCol(2,(z_axis_rotated[i]-positions_rotated[i])/norm(z_axis_rotated[i]-positions_rotated[i]));

            od.push_back(dcm2axis(orient));
        }

        if (positions_rotated.size()>0)
        {
            go_on=sendToClosedChain(positions_rotated, od);

            go_on=go_on &&askXdOdHat();
        }

        if (go_on)
        {
            for (size_t i=0; i<positions_rotated.size(); i++)
            {
                H_hand=axis2dcm(first_arm_pose[i].subVector(3,6));
                H_hand.setSubcol(first_arm_pose[i].subVector(0,2), 0, 3);

                changeFrame();

                Matrix orient(3,3);
                orient.setCol(0,(x_axis_rotated[i]-positions_rotated[i])/norm(x_axis_rotated[i]-positions_rotated[i]));
                orient.setCol(1,(y_axis_rotated[i]-positions_rotated[i])/norm(y_axis_rotated[i]-positions_rotated[i]));
                orient.setCol(2,(z_axis_rotated[i]-positions_rotated[i])/norm(z_axis_rotated[i]-positions_rotated[i]));

                Matrix orient_hat=axis2dcm(odhat[i]);

                err_orient.push_back(norm(orient_hat.getCol(0).subVector(0,2)-orient.getCol(0).subVector(0,2))+
                                          norm(orient_hat.getCol(1).subVector(0,2)-orient.getCol(1).subVector(0,2))+
                                               norm(orient_hat.getCol(2).subVector(0,2)-orient.getCol(2).subVector(0,2)));

                err_pos.push_back(norm(positions_rotated[i]-xdhat[i]));

                yDebug()<<" Error in orientation for pose "<<i<<": "<<err_orient[i];
                yDebug()<<" Error in position "<<i<<": "<<err_pos[i];
                yDebug()<<" Qd for pose "<<i<<": "<<qdhat[i].toString(3,3);
                cout<<endl;
            }

           int size_manip=manip.size();
           manip_notordered=manip;
           manip.clear();

           for (size_t i=0; i<size_manip; i++)
           {
               manip.push_back(manip_notordered[i]+1/(err_orient[i]));
               //manip.push_back(manip_notordered[i]);
           }


            manip_notordered.clear();
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
                        //index_poses[i] += (count-8)*(err_orient[i]+err_pos[i]);
                       // index_poses[i] += (count-8)*(err_pos[i]*1000);
                        index_poses[i] += (count-(int)manip_notordered.size());
                        cout<<"index poses "<<index_poses[i]<<endl;
                        first_time=false;
                    }
                    count++;
                }

                cout<<endl;
            }

            yInfo()<<" Index poses after manipulability: "<<index_poses.toString(3,3);
            return true;
        }
        else
        {
            yError()<<" Index poses cannot be computed!";
            return false;
        }
    }

    /*******************************************************************************/
    bool choosePose()
    {
        cout<<" choosepose "<<endl;
        double tmp=-100.0;
        int count=0;
        Vector qdhat(10,0.0);

        position_old=position_new;

        if (norm(index_poses)>0.0)
        {
            for(size_t i=0; i<index_poses.size(); i++)
            {
                if (tmp<index_poses[i])
                {
                    tmp=index_poses[i];
                    index=count;
                }
                count++;
            }
            cout<<endl;
            yInfo()<<" Selected pose: "<<index;

            if (first_arm_pose.size()>0)
            {
               chooseSecondHandPose();

            }
            cout<<endl;
        }

        if (norm(pos)>0.0)
        {            
            if (!closed_chain)
            {
                icart_arm_move->askForPose(positions_rotated[index], od[index], xd_h, od_h, qdhat);

                select_new_pose=false;
            }
            else
            {
                if (xdhat.size()>0)
                    select_new_pose=false;
                else
                    select_new_pose=true;
            }
        }
        return true;
    }

     /*******************************************************************************/
    bool sendToClosedChain(vector<Vector> &positions, vector<Vector> & od)
    {
        Bottle cmd, reply;
        cmd.addString("compute_manipulability");
        Bottle &cmd2=cmd.addList();
        Bottle &cmd3=cmd2.addList();
        cmd3.addString("positions");

        for (size_t i=0; i<positions.size(); i++)
        {
            Bottle &cmd4=cmd3.addList();
            cmd4.addDouble(positions[i][0]); cmd4.addDouble(positions[i][1]); cmd4.addDouble(positions[i][2]);
        }

        Bottle &cmd5=cmd2.addList();
        cmd5.addString("orientations");

        for (size_t i=0; i<od.size(); i++)
        {
            Bottle &cmd6=cmd5.addList();
            cmd6.addDouble(od[i][0]); cmd6.addDouble(od[i][1]); cmd6.addDouble(od[i][2]); cmd6.addDouble(od[i][3]);
        }

        if (portClosedChain.write(cmd, reply))
        {
            manip.clear();
            Bottle *rec=reply.get(0).asList();

            if (rec->size()>0)
            {
                for(size_t i=0; i<rec->size();i++)
                {
                    manip.push_back(rec->get(i).asDouble());

                    yDebug()<<" Received manipulability: "<<rec->get(i).asDouble();
                }
            }
            else
            {
                yError()<<" Received manipulability bottle empty!";
                return false;
            }
        }
        else
        {
            yError()<<" No manipulability indices received!!";
            return false;
        }

        return true;

    }

   /*******************************************************************************/
   bool askXdOdHat()
   {
       Bottle cmd, reply;
       Vector tmp(14,0.0);
       Vector second_arm_pose(7,0.0);
       Vector qm(7,0.0), qs(7,0.0);
       cmd.addString("get_solutions");

       if (portClosedChain.write(cmd, reply))
       {
           xdhat.clear();
           odhat.clear();
           qdhat.clear();
           first_arm_pose.clear();

           Bottle *cont=reply.get(0).asList();

           if (cont->size()>0)
           {
               for (size_t i=0; i<cont->size();i++)
               {
                   Bottle *pos=cont->get(i).asList();
                   for (size_t j=0; j<pos->size(); j++)
                   {
                       tmp[j]=pos->get(j).asDouble();
                   }
                   qdhat.push_back(tmp);
               }
           }
           else
           {
               yError()<<" Empty bottle of joints solution received!";
               return false;
           }
       }
       else
       {
          yError()<<" No joints solutions receveid!!";
          return false;
       }

       for (size_t j=0; j<qdhat.size();j++)
       {
           Vector qd_tmp=qdhat[j];

           for (size_t i=0; i<7; i++)
               qs[6-i]=-qd_tmp[i];

            for (size_t i=7; i<14; i++)
                qm[i-7]=qd_tmp[i];

            second_arm_pose=ikin_second_arm.EndEffPose(qm);
            first_arm_pose.push_back(ikin_first_arm.EndEffPose(qs));

            xdhat.push_back(second_arm_pose.subVector(0,2));
            odhat.push_back(second_arm_pose.subVector(3,6));
       }

       return true;
   }

   /*******************************************************************************/
   void addOffset(vector<Vector> &vect, vector<Vector> &axis,  vector<Vector> &center, double offset, string ax)
   {
       int start, end;

       if (left_or_right=="right")
       {
            for (size_t i=0; i<vect.size(); i++)
            {
                if (norm(axis[i]-center[i])>0.0)
                {
                    if (ax=="y")
                        vect[i]=vect[i] + offset*(axis[i]-center[i])/(norm(axis[i]-center[i]));
                    else
                        vect[i]=vect[i] - offset*(axis[i]-center[i])/(norm(axis[i]-center[i]));

                }
                else
                {

                    if (ax=="y")
                        vect[i]=vect[i] + offset*(axis[i])/(norm(axis[i]));
                    else
                        vect[i]=vect[i] - offset*(axis[i])/(norm(axis[i]));
                }

            }
       }
       else
       {
           for (size_t i=0; i<vect.size(); i++)
           {
               if (norm(axis[i]-center[i])>0.0)
               {
                   if (ax=="z")
                       vect[i]=vect[i] + offset*(axis[i]-center[i])/(norm(axis[i]-center[i]));
                   else if (ax=="x")
                       vect[i]=vect[i] - offset*(axis[i]-center[i])/(norm(axis[i]-center[i]));
                   else if (ax=="y")
                       vect[i]=vect[i] - offset*(axis[i]-center[i])/(norm(axis[i]-center[i]));
               }
               else
               {
                   if (ax=="z")
                       vect[i]=vect[i] + offset*(axis[i])/(norm(axis[i]));
                   else if (ax=="x")
                       vect[i]=vect[i] - offset*(axis[i])/(norm(axis[i]));                   
                   else if (ax=="y")
                       vect[i]=vect[i] - offset*(axis[i])/(norm(axis[i]));
               }
           }
       }
   }

   /*******************************************************************************/
   void addWaypoint(int n, int i, Vector &pose_second)
   {
       waypoints.clear();
       x_axis_wp.clear();
       y_axis_wp.clear();
       z_axis_wp.clear();
       vector<Vector> tmp1, tmp2, tmp3, tmp4;

       if (left_or_right=="right")
       {
            for (size_t j=0; j<n; j++)
            {
                Matrix orient_hat=axis2dcm(pose_second.subVector(3,6));

                waypoints.push_back(pose_second.subVector(0,2) - (j+1)*offset_z_approach*(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))));

                z_axis_wp.push_back(orient_hat.getCol(2).subVector(0,2) - (j+1)*offset_z_approach*(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))));
                x_axis_wp.push_back(orient_hat.getCol(0).subVector(0,2) - (j+1)*offset_z_approach*(orient_hat.getCol(0).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(0).subVector(0,2)-pose_second.subVector(0,2))));
                y_axis_wp.push_back(orient_hat.getCol(1).subVector(0,2) - (j+1)*offset_z_approach*(orient_hat.getCol(1).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(1).subVector(0,2)-pose_second.subVector(0,2))));

                if (offset_x_approach > 0.0)
                {
                    tmp1.push_back(waypoints[j]);
                    tmp2.push_back(x_axis_wp[j]);
                    tmp3.push_back(y_axis_wp[j]);
                    tmp4.push_back(z_axis_wp[j]);

                    addOffset(tmp1,tmp2,tmp1, (j+1)*offset_x_approach, "x");
                    addOffset(tmp2, tmp2,tmp1, (j+1)*offset_x_approach, "x");
                    addOffset(tmp4, tmp2,tmp1, (j+1)*offset_x_approach, "x");
                    addOffset(tmp3, tmp2,tmp1, (j+1)* offset_x_approach, "x");

                    waypoints[j]=tmp1[0];
                    x_axis_wp[j]=tmp2[0];
                    y_axis_wp[j]=tmp3[0];
                    z_axis_wp[j]=tmp4[0];

                    tmp1.clear(); tmp2.clear(); tmp3.clear(); tmp4.clear();
                }
            }
       }
       else
       {
           for (size_t j=0; j<n; j++)
           {
               Matrix orient_hat=axis2dcm(pose_second.subVector(3,6));

               waypoints.push_back(pose_second.subVector(0,2) + (j+1)*offset_z_approach*(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))));

               z_axis_wp.push_back(orient_hat.getCol(2).subVector(0,2) + (j+1)*offset_z_approach*(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(2).subVector(0,2)-pose_second.subVector(0,2))));
               x_axis_wp.push_back(orient_hat.getCol(0).subVector(0,2) - (j+1)*offset_z_approach*(orient_hat.getCol(0).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(0).subVector(0,2)-pose_second.subVector(0,2))));
               y_axis_wp.push_back(orient_hat.getCol(1).subVector(0,2) - (j+1)*offset_z_approach*(orient_hat.getCol(1).subVector(0,2)-pose_second.subVector(0,2))/(norm(orient_hat.getCol(1).subVector(0,2)-pose_second.subVector(0,2))));

               if (offset_x_approach > 0.0)
               {
                   tmp1.push_back(waypoints[j]);
                   tmp2.push_back(x_axis_wp[j]);
                   tmp3.push_back(y_axis_wp[j]);
                   tmp4.push_back(z_axis_wp[j]);

                   addOffset(tmp1,tmp2,tmp1, (j+1)*offset_x_approach, "x");
                   addOffset(tmp2, tmp2,tmp1, (j+1)*offset_x_approach, "x");
                   addOffset(tmp4, tmp2,tmp1, (j+1)*offset_x_approach, "x");
                   addOffset(tmp3, tmp2,tmp1, (j+1)* offset_x_approach, "x");

                   waypoints[j]=tmp1[0];
                   x_axis_wp[j]=tmp2[0];
                   y_axis_wp[j]=tmp3[0];
                   z_axis_wp[j]=tmp4[0];

                   tmp1.clear(); tmp2.clear(); tmp3.clear(); tmp4.clear();
               }
           }
       }
   }

   /*******************************************************************************/
   void rotatePoses(double angle)
   {
       Matrix aux(3,3);
       aux.zero();
       aux(0,0)=cos(angle);
       aux(0,2)=sin(angle);
       aux(2,0)=-sin(angle);
       aux(2,2)=cos(angle);
       aux(1,1)=1;
       x_axis.clear();
       y_axis.clear();
       z_axis.clear();

       Vector x(3,0.0), y(3,0.0), z(3,0.0);
       Vector matrix(9,0.0);

       for (size_t i=0; i<orientations.size(); i++)
       {
           matrix=orientations[i];
           Matrix aux2(3,3);
           aux2(0,0)=matrix[0]; aux2(1,0)=matrix[3]; aux2(2,0)=matrix[6];
           aux2(0,1)=matrix[1]; aux2(1,1)=matrix[4]; aux2(2,1)=matrix[7];
           aux2(0,2)=matrix[2]; aux2(1,2)=matrix[5]; aux2(2,2)=matrix[8];

           Matrix orient(3,3);

           cout<<"DET "<<i<<" "<<det(aux2)<<endl;

           orient=aux2*aux;

           x[0]=orient(0,0); x[1]=orient(1,0); x[2]=orient(2,0);
           y[0]=orient(0,1); y[1]=orient(1,1); y[2]=orient(2,1);
           z[0]=orient(0,2); z[1]=orient(1,2); z[2]=orient(2,2);
           x_axis.push_back(positions[i].subVector(0,2)+length*x);
           y_axis.push_back(positions[i].subVector(0,2)+length*y);
           z_axis.push_back(positions[i].subVector(0,2)+length*z);
       }

       new_angle=false;
   }

//   /*******************************************************************************/
//   bool reachWaypoint(int i)
//   {
//       Matrix orient(3,3);
//       orient.setCol(0,(x_axis_wp[i]-waypoints[i])/norm(x_axis_wp[i]-waypoints[i]));
//       orient.setCol(1,(y_axis_wp[i]-waypoints[i])/norm(y_axis_wp[i]-waypoints[i]));
//       orient.setCol(2,(z_axis_wp[i]-waypoints[i])/norm(z_axis_wp[i]-waypoints[i]));

//       Vector odhat_wp=dcm2axis(orient);

//       Vector x_tmp(3,0.0);
//       Vector o_tmp(4,0.0);

//       icart_arm_move->setInTargetTol(tolerance);


//       cout<< " Going to pose: "<<waypoints[i].toString()<<" "<< (odhat_wp).toString()<<endl<<endl;

//       icart_arm_move->goToPoseSync(waypoints[i], odhat_wp);
//       icart_arm_move->waitMotionDone();
//       icart_arm_move->getPose(x_tmp, o_tmp);

//       cout<<" Reached pose with"<<endl<<" position error: "<<norm(x_tmp - waypoints[i])<<endl<< " and orientation error: "<<norm(o_tmp- odhat_wp)<<endl<<endl;

//       icart_arm_move->setTrackingMode(false);
//       icart_arm_move->stopControl();
//       reach_waypoint=false;
//   }

   /*******************************************************************************/
   bool reachWaypointVel(int i)
   {
       for (size_t i=0; i<7;i++)
       {
           ctrlmode->setControlMode(i,VOCAB_CM_POSITION_DIRECT);
       }

       int context;
       icart_arm_move->storeContext(&context);

       Vector dof(10,1.0);
       dof[0]=dof[1]=dof[2]=0.0;

       icart_arm_move->setDOF(dof,dof);
       icart_arm_move->setTrajTime(2.0);

       cout<< " Going to waypoint: "<< waypoints[i].toString(3,3)<<" "<<pose_second.subVector(3,6).toString(3,3)<<endl<<endl;

       icart_arm_move->goToPoseSync(waypoints[i], pose_second.subVector(3,6));
       icart_arm_move->waitMotionDone();

       Vector x_tmp(3,0.0);
       Vector o_tmp(4,0.0);
       icart_arm_move->getPose(x_tmp, o_tmp);

       cout<<" Reached waypoint with position error: "<<norm(waypoints[i] - x_tmp)<< " and orientation error: "<<norm(pose_second.subVector(3,6)- o_tmp)<<endl<<endl;

       icart_arm_move->setTrackingMode(false);

       icart_arm_move->stopControl();

       icart_arm_move->restoreContext(context);
       icart_arm_move->deleteContext(context);

       for (size_t i=0; i<7;i++)
       {
           ctrlmode->setControlMode(i,VOCAB_CM_POSITION);
       }

       reach_waypoint=false;

       reached_waypoint=true;
   }

   /*******************************************************************************/
   bool reachFinalPoint()
   {
       for (size_t i=0; i<7;i++)
       {
           ctrlmode->setControlMode(i,VOCAB_CM_POSITION_DIRECT);
       }

       int context;
       icart_arm_move->storeContext(&context);

       Vector dof(10,1.0);
       dof[0]=dof[1]=dof[2]=0.0;

       icart_arm_move->setDOF(dof,dof);
       icart_arm_move->setTrajTime(2.0);

       if (!correct_pose)
       {
           cout<< " Going to  final pose: "<< pose_second.subVector(0,2).toString(3,3)<<" "<<pose_second.subVector(3,6).toString(3,3)<<endl<<endl;

           icart_arm_move->goToPoseSync(pose_second.subVector(0,2), pose_second.subVector(3,6));
           icart_arm_move->waitMotionDone();

           Vector x_tmp(3,0.0);
           Vector o_tmp(4,0.0);
           icart_arm_move->getPose(x_tmp, o_tmp);

            cout<<" Reached final pose with position error: "<<norm(x_tmp - pose_second.subVector(0,2))<< " and orientation error: "<<norm(o_tmp- pose_second.subVector(3,6))<<endl<<endl;

       }
       else
       {
           cout<< " Going to  final pose: "<< pose_second_corr.subVector(0,2).toString(3,3)<<" "<<pose_second_corr.subVector(3,6).toString(3,3)<<endl<<endl;

           icart_arm_move->goToPoseSync(pose_second_corr.subVector(0,2), pose_second_corr.subVector(3,6));
           icart_arm_move->waitMotionDone();

           Vector x_tmp(3,0.0);
           Vector o_tmp(4,0.0);
           icart_arm_move->getPose(x_tmp, o_tmp);

            cout<<" Reached final pose with position error: "<<norm(x_tmp - pose_second_corr.subVector(0,2))<< " and orientation error: "<<norm(o_tmp- pose_second_corr.subVector(3,6))<<endl<<endl;
       }
	yDebug()<< " Reached final pose!";

       yDebug()<<" Stopped control: "<<icart_arm_move->stopControl();

       icart_arm_move->setTrackingMode(false);

       icart_arm_move->restoreContext(context);
       icart_arm_move->deleteContext(context);

       reach_final_pose=false;

       for (size_t i=0; i<7;i++)
       {
           ctrlmode->setControlMode(i,VOCAB_CM_POSITION);
       }

       reached_final=true;

       return true;

   }

   /*******************************************************************************/
   void chooseSecondHandPose()
   {
       Matrix Hfinal_first(4,4);
       Hfinal_first.zero();
       Hfinal_first=axis2dcm(first_arm_pose[index].subVector(3,6));
       Hfinal_first(0,3)=first_arm_pose[index][0];
       Hfinal_first(1,3)=first_arm_pose[index][1];
       Hfinal_first(2,3)=first_arm_pose[index][2];
       Hfinal_first(3,3)=1;
       Matrix orient(4,4);
       orient(3,3)=1;
       orient.setSubcol((x_axis[index]-positions[index].subVector(0,2))/norm(x_axis[index]-positions[index].subVector(0,2)), 0, 0);
       orient.setSubcol((y_axis[index]-positions[index].subVector(0,2))/norm(y_axis[index]-positions[index].subVector(0,2)), 0, 1);
       orient.setSubcol((z_axis[index]-positions[index].subVector(0,2))/norm(z_axis[index]-positions[index].subVector(0,2)), 0, 2);

       Vector tmp(4,1.0);

       tmp.setSubvector(0,positions_rotated[index].subVector(0,2));

       tmp=Hfinal_first*SE3inv(H_hand)*tmp;

       pose_second.setSubvector(0,tmp.subVector(0,2));
       pose_second.setSubvector(3, dcm2axis(Hfinal_first*H_object*orient));

//      if (correct)
//      {
//           Matrix orient2(4,4);
//           orient2=axis2dcm(pose_second.subVector(3,6));

//           pose_second.setSubvector(0,pose_second.subVector(0,2)-y_corr*(orient2.subcol(0,1,3))/norm(orient2.subcol(0,1,3)));
//           correct=false;
//       }

       if (correct_pose)
       {
           int l=chooseSimilarPose();
           correctSecondPose(l);
       }

       cout<<endl<< " Computed second hand pose "<<pose_second.toString(3,3)<<endl<<endl;
   }

   /*******************************************************************************/
   void correctSecondPose(int i)
   {
       Vector matrix=correction_matrix[i];
       Matrix corr_matrix(4,4);
       corr_matrix(0,0)=matrix[0]; corr_matrix(1,0)=matrix[4]; corr_matrix(2,0)=matrix[8];  corr_matrix(3,0)=matrix[12];
       corr_matrix(0,1)=matrix[1]; corr_matrix(1,1)=matrix[5]; corr_matrix(2,1)=matrix[9];  corr_matrix(3,1)=matrix[13];
       corr_matrix(0,2)=matrix[2]; corr_matrix(1,2)=matrix[6]; corr_matrix(2,2)=matrix[10]; corr_matrix(3,2)=matrix[14];
       corr_matrix(0,3)=matrix[3]; corr_matrix(1,3)=matrix[7]; corr_matrix(2,3)=matrix[11]; corr_matrix(3,3)=matrix[15];

       cout<<" Correct matrix "<<corr_matrix.toString()<<endl;

       Vector tmp(4,1.0);
       tmp.setSubvector(0,pose_second.subVector(0,2));
       tmp=corr_matrix*tmp;
       pose_second_corr.setSubvector(0,tmp.subVector(0,2));
       pose_second_corr.setSubvector(3,dcm2axis(corr_matrix*axis2dcm(pose_second.subVector(3,6))));

       yDebug()<<" Corrected second pose"<<pose_second_corr.toString();
   }

   /*******************************************************************************/
   int chooseSimilarPose()
   {
       int j;
       double err_p, err_n;
       deque<double> errors;
       double min_err;


       for (size_t i=0; i<example_poses.size();i++)
       {
           err_p=norm(pose_second.subVector(0,2)-example_poses[i].subVector(0,2));
           err_n=norm(pose_second.subVector(3,6)-example_poses[i].subVector(3,6));
           errors.push_back(err_p+err_n);
           //errors.push_back(err_p);
           cout<<"error "<<errors[i]<<endl;

           if(i>=1)
           {              
               if (errors[i]<=min_err)
               {
                    j=i;
                    min_err=errors[i];
               }
           }
           else
           {
               j=0;
               min_err=errors[0];
           }
       }

       cout<<"selected j "<<j<<endl;
       return j;
   }
};

 /*******************************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError(" Unable to find YARP server!");
        return 1;
    }

    poseSelection mod;
    ResourceFinder rf;
    rf.setDefaultContext("poseSelection");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
