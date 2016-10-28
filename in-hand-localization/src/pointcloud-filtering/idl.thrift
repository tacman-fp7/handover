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
* pointCloudFiltering_IDL
*
* IDL Interface to \ref pointCloudExtraction services.
*/

service pointCloudFiltering_IDL
{
    /**
    * save or not
    * @param entry is yes or no
    * @return true/false on success/failure.
    */
    bool set_saving(1: string entry);

    /**
    * set filename for saving
    * @param entry is the name of file
    * @return true/false on success/failure.
    */
    bool set_filename(1: string entry);

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

}
