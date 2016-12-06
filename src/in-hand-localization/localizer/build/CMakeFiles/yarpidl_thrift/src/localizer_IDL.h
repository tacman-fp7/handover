// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_localizer_IDL
#define YARP_THRIFT_GENERATOR_localizer_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Bottle.h>

class localizer_IDL;


/**
 * localizer_IDL
 * IDL Interface to \ref pointCloudExtraction services.
 */
class localizer_IDL : public yarp::os::Wire {
public:
  localizer_IDL();
  /**
   * Get estimated pose
   * @return the 6D vector representing the object estimated pose
   */
  virtual yarp::os::Bottle get_estimated_pose();
  /**
   * Set acquire variable true and acquire points
   * @return true
   */
  virtual bool go_acquire();
  /**
   * Set localize variable true and start localization
   * @return true
   */
  virtual bool go_localize();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
