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

}
