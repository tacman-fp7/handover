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
* kinematicsTactileInfo_IDL
*
* IDL Interface to \ref pointCloudExtraction services.
*/

service kinematicsTactileInfo_IDL
{
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
    * Save pose information from kinematics
    * @return true/false on success/failure
    */
    bool save_pose();

    /**
    * Save tactile information from fingertips
    * @return true/false on success/failure
    */
    bool save_tactile_data();
}


