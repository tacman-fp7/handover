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
* localizer_IDL
*
* IDL Interface to \ref pointCloudExtraction services.
*/

service localizer_IDL
{
    /**
    * Get estimated pose
    * @return the 6D vector representing the object estimated pose
    */
    Bottle get_estimated_pose();

    /**
    * Set acquire variable true and acquire points
    * @return true
    */
    bool go_acquire();

    /**
    * Set localize variable true and start localization
    * @return true
    */
    bool go_localize();

    /**
    * Set localize variable false and stop localization
    * @return true
    */
    bool stop_localize();

    /**
    * Get measurements used for localization
    * @return measurements used for localization
    */
    Bottle get_meas();

    /**
    * Get statistics about i-th solution
    * @param is the index of the solution of interest
    * @return Bottle with solution information
    */
    Bottle get_solution_statistics(1: i32 entry);

    /**
    * Set object name 
    * @param is the name of the object to be localized
    * @return true if the name is one of the known objects
    */
    bool set_object_name(1: string entry);

    /**
    * Get the current lua_status (for demo)
    * @return lua status (true/false)
    */
    bool check_status();
}
