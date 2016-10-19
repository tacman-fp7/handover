// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_kinematicsTactileInfo_IDL
#define YARP_THRIFT_GENERATOR_kinematicsTactileInfo_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Bottle.h>

class kinematicsTactileInfo_IDL;


/**
 * kinematicsTactileInfo_IDL
 * IDL Interface to \ref pointCloudExtraction services.
 */
class kinematicsTactileInfo_IDL : public yarp::os::Wire {
public:
  kinematicsTactileInfo_IDL();
  /**
   * Get reference frame selected
   * @return "robot" or "hand"
   */
  virtual std::string get_frame();
  /**
   * Set reference frame selected
   * @param entry can be "robot" or "hand"
   * @return true/false on success/failure
   */
  virtual bool set_frame(const std::string& entry);
  /**
   * Get pose information from kinematics
   * @return the Property including all the information about pose
   */
  virtual yarp::os::Bottle get_pose();
  /**
   * Get tactile information from fingertips
   * @return the Property including all the information about tactila data
   */
  virtual yarp::os::Bottle get_tactile_data();
  /**
   * Save pose information from kinematics
   * @return true/false on success/failure
   */
  virtual bool save_pose();
  /**
   * Save tactile information from fingertips
   * @return true/false on success/failure
   */
  virtual bool save_tactile_data();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
