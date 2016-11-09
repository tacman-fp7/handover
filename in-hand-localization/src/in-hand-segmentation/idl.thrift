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
    * @return true/false on success/failure.
    */
    bool clear_points();

    /**
    * save or not
    * @param entry is yes or no
    * @return true/false on success/failure.
    */
    bool set_saving(1: string entry);

    /**
    * Select the saving format
    * @param entry is ply or off
    * @return true/false on success/failure.
    */
    bool set_format(1: string entry);

    /**
    * set filename for saving
    * @param entry is the name of file
    * @return true/false on success/failure.
    */
    bool set_filename(1: string entry);

    /**
    * acquire or not point cloud
    * @return true/false on success/failure.
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
    * @param entry is yes or no
    * @return return true/false on success/failure
`   */
    bool set_hand_filter(1: string entry);

    /**
    * Set hand filter on or off
    * @param entry is yes or no
    * @return return true/false on success/failure
`   */
    bool set_gray_filter(1: string entry);

    /**
    * Set hand filter on or off
    * @param entry is yes or no
    * @return return true/false on success/failure
`   */
    bool set_spatial_filter(1: string entry);

    /**
    * Set hand filter on or off
    * @param entry is yes or no
    * @return return true/false on success/failure
`   */
    bool set_volume_filter(1: string entry);

    /**
    * Set all filters on or off
    * @param entry is yes or no
    * @return return true/false on success/failure
`   */
    bool set_all_filters(1: string entry);

    /**
    * Let start the filtering process
    * @return return true/false on success/failure
`   */
    bool go_filter();



}
