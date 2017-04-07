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
* inHandSegmentation_IDL
*
* IDL Interface to \ref pointCloudExtraction services.
*/

service inHandSegmentation_IDL
{
    /**
    * Clear collected points
    * @return true/false on success/failure
    */
    bool clear_points();

    /**
    * save or not
    * @param entry is yes or no
    * @return true/false on success/failure
    */
    bool set_saving(1: string entry);

    /**
    * say if saving is set true or false
    * @return yes or no
    */
    string get_saving();

    /**
    * Select the saving format
    * @param entry is ply or off
    * @return true/false on success/failure
    */
    bool set_format(1: string entry);

    /**
    * say the saving format
    * @return the saving format
    */
    string get_format();

    /**
    * set filename for saving
    * @param entry is the name of file
    * @return true/false on success/failure
    */
    bool set_filename(1: string entry);

    /**
    * get filename for saving
    * @return saving filename
    */
    string get_filename();

    /**
    * acquire or not point cloud
    * @return true/false on success/failure
    */
    bool go_acquire();

    /**
    * Get the 2D pixels of the nearest blob
    * @return a bottle with the 2D points
    */
    Bottle get_2D_blob_points();

    /**
    * Get the 3D points of the nearest blob
    * @return a bottle with the 3D points
    */
    Bottle get_3D_blob_points();

    /**
    * Get reference frame selected
    * @return "robot" or "hand"
    */
    string get_frame();

    /**
    * Set reference frame selected
    * @param entry can be "robot" or "hand"
    * @return true/false on success/failure
    */
    bool set_frame(1:string entry);

    /**
    * Get pose information from kinematics
    * @return the Property including all the information about pose
    */
    Bottle get_pose();

    /**
    * Get tactile information from fingertips
    * @return the Property including all the information about tactila data
    */
    Bottle get_tactile_data();
/**
    * Get filtered points
    * @return true/false on success/failure.
    */
    Bottle get_filtered_points();

    /**
    * Say which filters are on
    * @return a string containing names of filters: HF for hand filter, GF for gray filter
    * SF for spatial filter, VF for volume filters, all is all filters are applied.
`   */
    string get_filters();

    /**
    * Set hand filter on or off
    * @param entry is on or off
    * @return return true/false on success/failure
`   */
    bool set_coarse_filter(1: string entry);

    /**
    * Set gray filter on or off
    * @param entry is on or off
    * @return return true/false on success/failure
`   */
    bool set_hand_filter(1: string entry);

    /**
    * Set spatial filter on or off
    * @param entry is on or off
    * @return return true/false on success/failure
`   */
    bool set_density_filter(1: string entry);

    /**
    * Set volume filter on or off
    * @param entry is yes or no
    * @return return true/false on success/failure
`   */
    bool set_cylinder_filter(1: string entry);

    /**
    * Set ellips filter on or off
    * @param entry is on or off
    * @return return true/false on success/failure
`   */
    bool set_ellipse_filter(1: string entry);

    /**
    * Set all filters on or off
    * @param entry is on or off
    * @return return true/false on success/failure
`   */
    bool set_all_filters(1: string entry);

    /**
    * Let start the filtering process
    * @return return true/false on success/failure
`   */
    bool go_filter();

    /**
    * Set hand filter parameter
    * @param entry is the name of the parameter
    * @param value is the value of the parameter
    * @return return true/false on success/failure
`   */
    bool set_parameter_coarse_filter(1: string entry, 2: double value);

    /**
    * Get hand filter parameter
    * @param entry is the name of the parameter
    * @return return the value of the parameter
`   */
    double get_parameter_coarse_filter(1: string entry);

    /**
    * Set ellips filter parameter
    * @param entry is the name of the parameter
    * @param value is the value of the parameter
    * @return return true/false on success/failure
`   */
    bool set_parameter_ellipse_filter(1: string entry, 2: double value);

    /**
    * Get elli[s filter parameter
    * @param entry is the name of the parameter
    * @return return the value of the parameter
`   */
    double get_parameter_ellipse_filter(1: string entry);

    /**
    * Set a new initial pose for the first hand
    * @param entry is a bottle cointaning position and orientation
    * @return true
    */
    bool new_hand_pose(1: Bottle entry)


    /**
    * Set fixate variable
    * @return true
    */
    bool set_fixate(1:string entry);

    /**
    * Get fixate variable
    * @return fixate
    */
    bool get_fixate();

    /**
    * Set automatic_acquisition variable
    * @return true
    */
    bool set_automatic_acquisition(1:string entry);

    /**
    * Get automatic_acquisition variable
    * @return automatic_acquisition
    */
    bool get_automatic_acquisition();

    /**
    * Reset count_pose variable
    * @param entry is the number of the pose
    * @return true
    */
    bool reset_count_pose(1:i32 entry);

    /**
    * Say if enabled depth2kin calibration
    *@return yes or no
    */
    string get_calib_cam();

    /**
    * Enable or not depth2kin calibration
    *@param yes or no
    *@return true/false on success/failure
    */
    bool set_calib_cam(1:string calib_or_not);


    /**
    * Set in initial pose 0
    *@param entry number of pose
    *@return true/false on success/failure
    */
    bool try_again(1:i32 entry);

    /**
    * look people in front of the robot
    *@return true
    */
    bool look_in_front();

    /**
    * Get the current lua_status (for demo)
    * @return lua status (true/false)
    */
    bool check_status();

}
