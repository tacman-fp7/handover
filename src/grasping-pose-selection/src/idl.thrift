# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* Property
*
* IDL structure to set/show advanced parameters.
*/
struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

/**
* poseSelection_IDL
*
* IDL Interface to \ref poseSelection services.
*/

service poseSelection_IDL
{
    /**
    * Update pose to be shown
    * @return true/false on success/failure
    */
    bool ask_new_pose();

    /**
    * Set boolean variable select_new_pose equal to true
    * @return true/false on success/failure
    */
    bool choose_new_pose();

    /**
    * select the pose
    * @return true/false on success/failure
    */
    bool select_pose(1:i32 entry);

    /**
    * get the selected pose
    * @return bottle containing the pose
    */
    Bottle get_pose();

    /**
    * get pose of the moving arm
    * @return bottle containing the pose
    */
    Bottle get_pose_moving_arm();

    /**
    * get moving arm
    * @return string of the moving arm
    */
    string get_moving_arm();

    /**
    * update hand pose
    * @return true/false on success/failure
    */
    bool update_pose_hand();

    /**
    * get H hand
    * @ return a bottle coitaning H_hand matrix
    */
    Bottle get_Hhand();
}
