// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_inHandSegmentation_IDL
#define YARP_THRIFT_GENERATOR_inHandSegmentation_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Bottle.h>

class inHandSegmentation_IDL;


/**
 * inHandSegmentation_IDL
 * IDL Interface to \ref pointCloudExtraction services.
 */
class inHandSegmentation_IDL : public yarp::os::Wire {
public:
  inHandSegmentation_IDL();
  /**
   * Clear collected points
   * @return true/false on success/failure
   */
  virtual bool clear_points();
  /**
   * save or not
   * @param entry is yes or no
   * @return true/false on success/failure
   */
  virtual bool set_saving(const std::string& entry);
  /**
   * say if saving is set true or false
   * @return yes or no
   */
  virtual std::string get_saving();
  /**
   * Select the saving format
   * @param entry is ply or off
   * @return true/false on success/failure
   */
  virtual bool set_format(const std::string& entry);
  /**
   * say the saving format
   * @return the saving format
   */
  virtual std::string get_format();
  /**
   * set filename for saving
   * @param entry is the name of file
   * @return true/false on success/failure
   */
  virtual bool set_filename(const std::string& entry);
  /**
   * get filename for saving
   * @return saving filename
   */
  virtual std::string get_filename();
  /**
   * acquire or not point cloud
   * @return true/false on success/failure
   */
  virtual bool go_acquire();
  /**
   * Get the 2D pixels of the nearest blob
   * @return a bottle with the 2D points
   */
  virtual yarp::os::Bottle get_2D_blob_points();
  /**
   * Get the 3D points of the nearest blob
   * @return a bottle with the 3D points
   */
  virtual yarp::os::Bottle get_3D_blob_points();
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
   * Get filtered points
   * @return true/false on success/failure.
   */
  virtual yarp::os::Bottle get_filtered_points();
  /**
   *     * Say which filters are on
   *     * @return a string containing names of filters: HF for hand filter, GF for gray filter
   *     * SF for spatial filter, VF for volume filters, all is all filters are applied.
   * `
   */
  virtual std::string get_filters();
  /**
   *     * Set hand filter on or off
   *     * @param entry is on or off
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_coarse_filter(const std::string& entry);
  /**
   *     * Set gray filter on or off
   *     * @param entry is on or off
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_hand_filter(const std::string& entry);
  /**
   *     * Set spatial filter on or off
   *     * @param entry is on or off
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_density_filter(const std::string& entry);
  /**
   *     * Set volume filter on or off
   *     * @param entry is yes or no
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_cylinder_filter(const std::string& entry);
  /**
   *     * Set ellips filter on or off
   *     * @param entry is on or off
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_ellipse_filter(const std::string& entry);
  /**
   *     * Set all filters on or off
   *     * @param entry is on or off
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_all_filters(const std::string& entry);
  /**
   *     * Let start the filtering process
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool go_filter();
  /**
   *     * Set hand filter parameter
   *     * @param entry is the name of the parameter
   *     * @param value is the value of the parameter
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_parameter_coarse_filter(const std::string& entry, const double value);
  /**
   *     * Get hand filter parameter
   *     * @param entry is the name of the parameter
   *     * @return return the value of the parameter
   * `
   */
  virtual double get_parameter_coarse_filter(const std::string& entry);
  /**
   *     * Set ellips filter parameter
   *     * @param entry is the name of the parameter
   *     * @param value is the value of the parameter
   *     * @return return true/false on success/failure
   * `
   */
  virtual bool set_parameter_ellipse_filter(const std::string& entry, const double value);
  /**
   *     * Get elli[s filter parameter
   *     * @param entry is the name of the parameter
   *     * @return return the value of the parameter
   * `
   */
  virtual double get_parameter_ellipse_filter(const std::string& entry);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
