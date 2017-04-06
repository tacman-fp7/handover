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
    * @param entry is the index of the desired pose
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
    * @return the index
    */
    i32 get_index();

    /**
    * Set offset in order to shift poses 
    * far away from the object surface on z-axis
    * @param entry is the offset value
    * @return true
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
    * @param entry is the offset value
    * @ return true
    */
    bool set_offset_x_approach(1:double entry);

    /**
    * Get offset used to shift poses 
    * far away from the object surface on x-axis durign approach
    * @return offset value
    */
    double get_offset_x_approach();

    /**
    * Set offset in order to shift final poses 
    * far away from the object surface on x-axis 
    * @param entry is the offset value
    * @return true
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
    * @param entry is the offset value
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
    * @param entry is the angle value
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
    * @param entry is the number of waypoints
    * @return number of waypoints
    */
    bool set_n_waypoint(1: i32 entry);

    /**
    * Set number of waypoint to reach and reach it
    * @param entry is the number of the waypoint we want to reach
    * @return true
    */
    bool reach_pose(1: i32 entry);

    /**
    * Move both the arms back to the initial pose
    * @ return true
    */
    bool go_back_home();

    /**
    * Set tolerance for cartesian solver
    * @param entry is the tolerance value
    * @return true
    */
    bool set_tolerance(1:double tolerance);

    /**
    * Get tolerance for cartesian solver
    * @return tolerance value
    */
    double get_tolerance();

    /**
    * Ask the robot to reach the final pose
    * @return tolerance value
    */
    bool reach_final();

    double get_y_correction();

    bool set_y_correction(1:double entry);

    /**
    * Set offset in order to shift final poses 
    * far away from the object surface on y-axis 
    * @param entry is the offset value
    * @return true
    */
    bool set_offset_y_final(1:double entry);

    /**
    * Get offset used to shift final poses 
    * far away from the object surface on y-axis
    * @return offset value
    */
    double get_offset_y_final();

    /**
    * Set trajectory time for arms movements
    * @param entry is the trajectory time value
    * @return true
    */
    bool set_traj_time(1:double entry);

    /**
    * Get trajector time for arms movements
    * @ return trajectory time value
    */
    double get_traj_time();
}
