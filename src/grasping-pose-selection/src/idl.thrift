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

    /**
    * get index of the selected pose
    * @ return the index
    */
    i32 get_index();

    /**
    * Set offset in order to shift poses 
    * far away from the object surface on z-axis
    * @ return true
    */
    bool set_offset_z_approach(1:double entry);

    /**
    * Get offset used to shift poses 
    * far away from the object surface on z-axis
    * @ return offset value
    */
    double get_offset_z_approach();

    /**
    * Set offset in order to shift poses 
    * far away from the object surface on x-axis during approach
    * @ return true
    */
    bool set_offset_x_approach(1:double entry);

    /**
    * Get offset used to shift poses 
    * far away from the object surface on x-axis durign approach
    * @ return offset value
    */
    double get_offset_x_approach();

    /**
    * Set offset in order to shift final poses 
    * far away from the object surface on x-axis 
    * @ return true
    */
    bool set_offset_x_final(1:double entry);

    /**
    * Get offset used to shift final poses 
    * far away from the object surface on x-axis
    * @ return offset value
    */
    double get_offset_x_final();

    /**
    * Set offset_z_final between waypoints
    * @ return true
    */
    bool set_offset_z_final(1:double entry);

    /**
    * Get offset_z_final between waypoints
    * @ return offset value
    */
    double get_offset_z_final();
    
    /**
    * Set angle for wrist
    * @ return true
    */
    bool set_angle(1:double entry);

    /**
    * Get angle for wrist
    * @ return angle value
    */
    double get_angle();

    /**
    * Get number of waypoints
    * @ return number of waypoints
    */
    i32 get_n_waypoint();

    /**
    * Set number of waypoints
    * @ return number of waypoints
    */
    bool set_n_waypoint(1: i32 entry);

    /**
    * Set number of waypoint to reach and reach it
    * @ return true
    */
    bool set_waypoint(1: i32 entry);

    /**
    * Set tolerance for cartesian solver
    * @ return true
    */
    bool set_tolerance(1:double tolerance);

    /**
    * Get tolerance for cartesian solver
    * @ return tolerance value
    */

    bool reach_final();
    double get_tolerance();
}
