// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <inHandSegmentation_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class inHandSegmentation_IDL_clear_points : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_saving : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_saving : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_format : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_format : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_filename : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_filename : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_go_acquire : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_2D_blob_points : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_3D_blob_points : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_frame : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_frame : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_pose : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_tactile_data : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_filtered_points : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_filters : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_coarse_filter : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_hand_filter : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_density_filter : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_cylinder_filter : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_ellipse_filter : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_all_filters : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_go_filter : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_parameter_coarse_filter : public yarp::os::Portable {
public:
  std::string entry;
  double value;
  bool _return;
  void init(const std::string& entry, const double value);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_parameter_coarse_filter : public yarp::os::Portable {
public:
  std::string entry;
  double _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_set_parameter_ellipse_filter : public yarp::os::Portable {
public:
  std::string entry;
  double value;
  bool _return;
  void init(const std::string& entry, const double value);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class inHandSegmentation_IDL_get_parameter_ellipse_filter : public yarp::os::Portable {
public:
  std::string entry;
  double _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool inHandSegmentation_IDL_clear_points::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("clear_points",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_clear_points::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_clear_points::init() {
  _return = false;
}

bool inHandSegmentation_IDL_set_saving::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_saving",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_saving::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_saving::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_get_saving::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_saving",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_saving::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_saving::init() {
  _return = "";
}

bool inHandSegmentation_IDL_set_format::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_format",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_format::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_format::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_get_format::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_format",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_format::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_format::init() {
  _return = "";
}

bool inHandSegmentation_IDL_set_filename::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_filename",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_filename::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_filename::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_get_filename::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_filename",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_filename::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_filename::init() {
  _return = "";
}

bool inHandSegmentation_IDL_go_acquire::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("go_acquire",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_go_acquire::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_go_acquire::init() {
  _return = false;
}

bool inHandSegmentation_IDL_get_2D_blob_points::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("get_2D_blob_points",1,4)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_2D_blob_points::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_2D_blob_points::init() {
}

bool inHandSegmentation_IDL_get_3D_blob_points::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("get_3D_blob_points",1,4)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_3D_blob_points::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_3D_blob_points::init() {
}

bool inHandSegmentation_IDL_get_frame::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_frame",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_frame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_frame::init() {
  _return = "";
}

bool inHandSegmentation_IDL_set_frame::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_frame",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_frame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_frame::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_get_pose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_pose",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_pose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_pose::init() {
}

bool inHandSegmentation_IDL_get_tactile_data::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("get_tactile_data",1,3)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_tactile_data::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_tactile_data::init() {
}

bool inHandSegmentation_IDL_get_filtered_points::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("get_filtered_points",1,3)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_filtered_points::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_filtered_points::init() {
}

bool inHandSegmentation_IDL_get_filters::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_filters",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_filters::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_filters::init() {
  _return = "";
}

bool inHandSegmentation_IDL_set_coarse_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_coarse_filter",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_coarse_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_coarse_filter::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_set_hand_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_hand_filter",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_hand_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_hand_filter::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_set_density_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_density_filter",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_density_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_density_filter::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_set_cylinder_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_cylinder_filter",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_cylinder_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_cylinder_filter::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_set_ellipse_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_ellipse_filter",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_ellipse_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_ellipse_filter::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_set_all_filters::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_all_filters",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_all_filters::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_all_filters::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool inHandSegmentation_IDL_go_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("go_filter",1,2)) return false;
  return true;
}

bool inHandSegmentation_IDL_go_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_go_filter::init() {
  _return = false;
}

bool inHandSegmentation_IDL_set_parameter_coarse_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  if (!writer.writeTag("set_parameter_coarse_filter",1,4)) return false;
  if (!writer.writeString(entry)) return false;
  if (!writer.writeDouble(value)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_parameter_coarse_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_parameter_coarse_filter::init(const std::string& entry, const double value) {
  _return = false;
  this->entry = entry;
  this->value = value;
}

bool inHandSegmentation_IDL_get_parameter_coarse_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("get_parameter_coarse_filter",1,4)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_parameter_coarse_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_parameter_coarse_filter::init(const std::string& entry) {
  _return = (double)0;
  this->entry = entry;
}

bool inHandSegmentation_IDL_set_parameter_ellipse_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  if (!writer.writeTag("set_parameter_ellipse_filter",1,4)) return false;
  if (!writer.writeString(entry)) return false;
  if (!writer.writeDouble(value)) return false;
  return true;
}

bool inHandSegmentation_IDL_set_parameter_ellipse_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_set_parameter_ellipse_filter::init(const std::string& entry, const double value) {
  _return = false;
  this->entry = entry;
  this->value = value;
}

bool inHandSegmentation_IDL_get_parameter_ellipse_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("get_parameter_ellipse_filter",1,4)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool inHandSegmentation_IDL_get_parameter_ellipse_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void inHandSegmentation_IDL_get_parameter_ellipse_filter::init(const std::string& entry) {
  _return = (double)0;
  this->entry = entry;
}

inHandSegmentation_IDL::inHandSegmentation_IDL() {
  yarp().setOwner(*this);
}
bool inHandSegmentation_IDL::clear_points() {
  bool _return = false;
  inHandSegmentation_IDL_clear_points helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::clear_points()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_saving(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_saving helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_saving(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string inHandSegmentation_IDL::get_saving() {
  std::string _return = "";
  inHandSegmentation_IDL_get_saving helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string inHandSegmentation_IDL::get_saving()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_format(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_format helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_format(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string inHandSegmentation_IDL::get_format() {
  std::string _return = "";
  inHandSegmentation_IDL_get_format helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string inHandSegmentation_IDL::get_format()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_filename(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_filename helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_filename(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string inHandSegmentation_IDL::get_filename() {
  std::string _return = "";
  inHandSegmentation_IDL_get_filename helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string inHandSegmentation_IDL::get_filename()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::go_acquire() {
  bool _return = false;
  inHandSegmentation_IDL_go_acquire helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::go_acquire()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle inHandSegmentation_IDL::get_2D_blob_points() {
  yarp::os::Bottle _return;
  inHandSegmentation_IDL_get_2D_blob_points helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle inHandSegmentation_IDL::get_2D_blob_points()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle inHandSegmentation_IDL::get_3D_blob_points() {
  yarp::os::Bottle _return;
  inHandSegmentation_IDL_get_3D_blob_points helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle inHandSegmentation_IDL::get_3D_blob_points()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string inHandSegmentation_IDL::get_frame() {
  std::string _return = "";
  inHandSegmentation_IDL_get_frame helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string inHandSegmentation_IDL::get_frame()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_frame(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_frame helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_frame(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle inHandSegmentation_IDL::get_pose() {
  yarp::os::Bottle _return;
  inHandSegmentation_IDL_get_pose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle inHandSegmentation_IDL::get_pose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle inHandSegmentation_IDL::get_tactile_data() {
  yarp::os::Bottle _return;
  inHandSegmentation_IDL_get_tactile_data helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle inHandSegmentation_IDL::get_tactile_data()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle inHandSegmentation_IDL::get_filtered_points() {
  yarp::os::Bottle _return;
  inHandSegmentation_IDL_get_filtered_points helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle inHandSegmentation_IDL::get_filtered_points()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string inHandSegmentation_IDL::get_filters() {
  std::string _return = "";
  inHandSegmentation_IDL_get_filters helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string inHandSegmentation_IDL::get_filters()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_coarse_filter(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_coarse_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_coarse_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_hand_filter(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_hand_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_hand_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_density_filter(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_density_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_density_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_cylinder_filter(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_cylinder_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_cylinder_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_ellipse_filter(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_ellipse_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_ellipse_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_all_filters(const std::string& entry) {
  bool _return = false;
  inHandSegmentation_IDL_set_all_filters helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_all_filters(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::go_filter() {
  bool _return = false;
  inHandSegmentation_IDL_go_filter helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::go_filter()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_parameter_coarse_filter(const std::string& entry, const double value) {
  bool _return = false;
  inHandSegmentation_IDL_set_parameter_coarse_filter helper;
  helper.init(entry,value);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_parameter_coarse_filter(const std::string& entry, const double value)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double inHandSegmentation_IDL::get_parameter_coarse_filter(const std::string& entry) {
  double _return = (double)0;
  inHandSegmentation_IDL_get_parameter_coarse_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double inHandSegmentation_IDL::get_parameter_coarse_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool inHandSegmentation_IDL::set_parameter_ellipse_filter(const std::string& entry, const double value) {
  bool _return = false;
  inHandSegmentation_IDL_set_parameter_ellipse_filter helper;
  helper.init(entry,value);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool inHandSegmentation_IDL::set_parameter_ellipse_filter(const std::string& entry, const double value)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double inHandSegmentation_IDL::get_parameter_ellipse_filter(const std::string& entry) {
  double _return = (double)0;
  inHandSegmentation_IDL_get_parameter_ellipse_filter helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double inHandSegmentation_IDL::get_parameter_ellipse_filter(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool inHandSegmentation_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "clear_points") {
      bool _return;
      _return = clear_points();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_saving") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_saving(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_saving") {
      std::string _return;
      _return = get_saving();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_format") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_format(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_format") {
      std::string _return;
      _return = get_format();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_filename") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_filename(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_filename") {
      std::string _return;
      _return = get_filename();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "go_acquire") {
      bool _return;
      _return = go_acquire();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_2D_blob_points") {
      yarp::os::Bottle _return;
      _return = get_2D_blob_points();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_3D_blob_points") {
      yarp::os::Bottle _return;
      _return = get_3D_blob_points();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_frame") {
      std::string _return;
      _return = get_frame();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_frame") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_frame(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_pose") {
      yarp::os::Bottle _return;
      _return = get_pose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_tactile_data") {
      yarp::os::Bottle _return;
      _return = get_tactile_data();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_filtered_points") {
      yarp::os::Bottle _return;
      _return = get_filtered_points();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_filters") {
      std::string _return;
      _return = get_filters();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_coarse_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_coarse_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_hand_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_hand_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_density_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_density_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_cylinder_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_cylinder_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_ellipse_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_ellipse_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_all_filters") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_all_filters(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "go_filter") {
      bool _return;
      _return = go_filter();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_parameter_coarse_filter") {
      std::string entry;
      double value;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(value)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_parameter_coarse_filter(entry,value);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_parameter_coarse_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      double _return;
      _return = get_parameter_coarse_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_parameter_ellipse_filter") {
      std::string entry;
      double value;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(value)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_parameter_ellipse_filter(entry,value);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_parameter_ellipse_filter") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      double _return;
      _return = get_parameter_ellipse_filter(entry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> inHandSegmentation_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("clear_points");
    helpString.push_back("set_saving");
    helpString.push_back("get_saving");
    helpString.push_back("set_format");
    helpString.push_back("get_format");
    helpString.push_back("set_filename");
    helpString.push_back("get_filename");
    helpString.push_back("go_acquire");
    helpString.push_back("get_2D_blob_points");
    helpString.push_back("get_3D_blob_points");
    helpString.push_back("get_frame");
    helpString.push_back("set_frame");
    helpString.push_back("get_pose");
    helpString.push_back("get_tactile_data");
    helpString.push_back("get_filtered_points");
    helpString.push_back("get_filters");
    helpString.push_back("set_coarse_filter");
    helpString.push_back("set_hand_filter");
    helpString.push_back("set_density_filter");
    helpString.push_back("set_cylinder_filter");
    helpString.push_back("set_ellipse_filter");
    helpString.push_back("set_all_filters");
    helpString.push_back("go_filter");
    helpString.push_back("set_parameter_coarse_filter");
    helpString.push_back("get_parameter_coarse_filter");
    helpString.push_back("set_parameter_ellipse_filter");
    helpString.push_back("get_parameter_ellipse_filter");
    helpString.push_back("help");
  }
  else {
    if (functionName=="clear_points") {
      helpString.push_back("bool clear_points() ");
      helpString.push_back("Clear collected points ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="set_saving") {
      helpString.push_back("bool set_saving(const std::string& entry) ");
      helpString.push_back("save or not ");
      helpString.push_back("@param entry is yes or no ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_saving") {
      helpString.push_back("std::string get_saving() ");
      helpString.push_back("say if saving is set true or false ");
      helpString.push_back("@return yes or no ");
    }
    if (functionName=="set_format") {
      helpString.push_back("bool set_format(const std::string& entry) ");
      helpString.push_back("Select the saving format ");
      helpString.push_back("@param entry is ply or off ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_format") {
      helpString.push_back("std::string get_format() ");
      helpString.push_back("say the saving format ");
      helpString.push_back("@return the saving format ");
    }
    if (functionName=="set_filename") {
      helpString.push_back("bool set_filename(const std::string& entry) ");
      helpString.push_back("set filename for saving ");
      helpString.push_back("@param entry is the name of file ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_filename") {
      helpString.push_back("std::string get_filename() ");
      helpString.push_back("get filename for saving ");
      helpString.push_back("@return saving filename ");
    }
    if (functionName=="go_acquire") {
      helpString.push_back("bool go_acquire() ");
      helpString.push_back("acquire or not point cloud ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_2D_blob_points") {
      helpString.push_back("yarp::os::Bottle get_2D_blob_points() ");
      helpString.push_back("Get the 2D pixels of the nearest blob ");
      helpString.push_back("@return a bottle with the 2D points ");
    }
    if (functionName=="get_3D_blob_points") {
      helpString.push_back("yarp::os::Bottle get_3D_blob_points() ");
      helpString.push_back("Get the 3D points of the nearest blob ");
      helpString.push_back("@return a bottle with the 3D points ");
    }
    if (functionName=="get_frame") {
      helpString.push_back("std::string get_frame() ");
      helpString.push_back("Get reference frame selected ");
      helpString.push_back("@return \"robot\" or \"hand\" ");
    }
    if (functionName=="set_frame") {
      helpString.push_back("bool set_frame(const std::string& entry) ");
      helpString.push_back("Set reference frame selected ");
      helpString.push_back("@param entry can be \"robot\" or \"hand\" ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_pose") {
      helpString.push_back("yarp::os::Bottle get_pose() ");
      helpString.push_back("Get pose information from kinematics ");
      helpString.push_back("@return the Property including all the information about pose ");
    }
    if (functionName=="get_tactile_data") {
      helpString.push_back("yarp::os::Bottle get_tactile_data() ");
      helpString.push_back("Get tactile information from fingertips ");
      helpString.push_back("@return the Property including all the information about tactila data ");
    }
    if (functionName=="get_filtered_points") {
      helpString.push_back("yarp::os::Bottle get_filtered_points() ");
      helpString.push_back("Get filtered points ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="get_filters") {
      helpString.push_back("std::string get_filters() ");
      helpString.push_back("    * Say which filters are on ");
      helpString.push_back("    * @return a string containing names of filters: HF for hand filter, GF for gray filter ");
      helpString.push_back("    * SF for spatial filter, VF for volume filters, all is all filters are applied. ");
      helpString.push_back("` ");
    }
    if (functionName=="set_coarse_filter") {
      helpString.push_back("bool set_coarse_filter(const std::string& entry) ");
      helpString.push_back("    * Set hand filter on or off ");
      helpString.push_back("    * @param entry is on or off ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="set_hand_filter") {
      helpString.push_back("bool set_hand_filter(const std::string& entry) ");
      helpString.push_back("    * Set gray filter on or off ");
      helpString.push_back("    * @param entry is on or off ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="set_density_filter") {
      helpString.push_back("bool set_density_filter(const std::string& entry) ");
      helpString.push_back("    * Set spatial filter on or off ");
      helpString.push_back("    * @param entry is on or off ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="set_cylinder_filter") {
      helpString.push_back("bool set_cylinder_filter(const std::string& entry) ");
      helpString.push_back("    * Set volume filter on or off ");
      helpString.push_back("    * @param entry is yes or no ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="set_ellipse_filter") {
      helpString.push_back("bool set_ellipse_filter(const std::string& entry) ");
      helpString.push_back("    * Set ellips filter on or off ");
      helpString.push_back("    * @param entry is on or off ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="set_all_filters") {
      helpString.push_back("bool set_all_filters(const std::string& entry) ");
      helpString.push_back("    * Set all filters on or off ");
      helpString.push_back("    * @param entry is on or off ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="go_filter") {
      helpString.push_back("bool go_filter() ");
      helpString.push_back("    * Let start the filtering process ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="set_parameter_coarse_filter") {
      helpString.push_back("bool set_parameter_coarse_filter(const std::string& entry, const double value) ");
      helpString.push_back("    * Set hand filter parameter ");
      helpString.push_back("    * @param entry is the name of the parameter ");
      helpString.push_back("    * @param value is the value of the parameter ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="get_parameter_coarse_filter") {
      helpString.push_back("double get_parameter_coarse_filter(const std::string& entry) ");
      helpString.push_back("    * Get hand filter parameter ");
      helpString.push_back("    * @param entry is the name of the parameter ");
      helpString.push_back("    * @return return the value of the parameter ");
      helpString.push_back("` ");
    }
    if (functionName=="set_parameter_ellipse_filter") {
      helpString.push_back("bool set_parameter_ellipse_filter(const std::string& entry, const double value) ");
      helpString.push_back("    * Set ellips filter parameter ");
      helpString.push_back("    * @param entry is the name of the parameter ");
      helpString.push_back("    * @param value is the value of the parameter ");
      helpString.push_back("    * @return return true/false on success/failure ");
      helpString.push_back("` ");
    }
    if (functionName=="get_parameter_ellipse_filter") {
      helpString.push_back("double get_parameter_ellipse_filter(const std::string& entry) ");
      helpString.push_back("    * Get elli[s filter parameter ");
      helpString.push_back("    * @param entry is the name of the parameter ");
      helpString.push_back("    * @return return the value of the parameter ");
      helpString.push_back("` ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


