// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_pointCloudExtraction_IDL
#define YARP_THRIFT_GENERATOR_pointCloudExtraction_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class pointCloudExtraction_IDL;


/**
 * pointCloudExtraction_IDL
 * IDL Interface to \ref pointCloudExtraction services.
 */
class pointCloudExtraction_IDL : public yarp::os::Wire {
public:
  pointCloudExtraction_IDL();
  /**
   * Clear collected points
   * @return true/false on success/failure.
   */
  virtual bool clear_points();
  /**
   * Set or unset autoseed variable equal to true
   * @param entry is yes or no
   * @return true/false on success/failure.
   */
  virtual bool set_auto_seed(const std::string& entry);
  /**
   * save or not
   * @param entry is yes or no
   * @return true/false on success/failure.
   */
  virtual bool set_saving(const std::string& entry);
  /**
   * Select the saving format
   * @param entry is ply or off
   * @return true/false on success/failure.
   */
  virtual bool set_format(const std::string& entry);
  /**
   * set filename for saving
   * @param entry is the name of file
   * @return true/false on success/failure.
   */
  virtual bool set_filename(const std::string& entry);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
