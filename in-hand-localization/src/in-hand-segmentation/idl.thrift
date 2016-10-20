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
struct Property
{
} (
   yarp.name = "yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
  )

/**
* pointCloudExtraction_IDL
*
* IDL Interface to \ref pointCloudExtraction services.
*/

service pointCloudExtraction_IDL
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
    * @param entry is "yes" or "no"
    * @return true/false on success/failure.
    */
    bool acquiring(1: string entry);

}
