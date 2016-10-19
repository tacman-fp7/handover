// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <kinematicsTactileInfo_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class kinematicsTactileInfo_IDL_get_frame : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class kinematicsTactileInfo_IDL_set_frame : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class kinematicsTactileInfo_IDL_get_pose : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class kinematicsTactileInfo_IDL_get_tactile_data : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class kinematicsTactileInfo_IDL_save_pose : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class kinematicsTactileInfo_IDL_save_tactile_data : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool kinematicsTactileInfo_IDL_get_frame::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_frame",1,2)) return false;
  return true;
}

bool kinematicsTactileInfo_IDL_get_frame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void kinematicsTactileInfo_IDL_get_frame::init() {
  _return = "";
}

bool kinematicsTactileInfo_IDL_set_frame::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_frame",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool kinematicsTactileInfo_IDL_set_frame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void kinematicsTactileInfo_IDL_set_frame::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool kinematicsTactileInfo_IDL_get_pose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_pose",1,2)) return false;
  return true;
}

bool kinematicsTactileInfo_IDL_get_pose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void kinematicsTactileInfo_IDL_get_pose::init() {
}

bool kinematicsTactileInfo_IDL_get_tactile_data::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("get_tactile_data",1,3)) return false;
  return true;
}

bool kinematicsTactileInfo_IDL_get_tactile_data::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void kinematicsTactileInfo_IDL_get_tactile_data::init() {
}

bool kinematicsTactileInfo_IDL_save_pose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("save_pose",1,2)) return false;
  return true;
}

bool kinematicsTactileInfo_IDL_save_pose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void kinematicsTactileInfo_IDL_save_pose::init() {
  _return = false;
}

bool kinematicsTactileInfo_IDL_save_tactile_data::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("save_tactile_data",1,3)) return false;
  return true;
}

bool kinematicsTactileInfo_IDL_save_tactile_data::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void kinematicsTactileInfo_IDL_save_tactile_data::init() {
  _return = false;
}

kinematicsTactileInfo_IDL::kinematicsTactileInfo_IDL() {
  yarp().setOwner(*this);
}
std::string kinematicsTactileInfo_IDL::get_frame() {
  std::string _return = "";
  kinematicsTactileInfo_IDL_get_frame helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string kinematicsTactileInfo_IDL::get_frame()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool kinematicsTactileInfo_IDL::set_frame(const std::string& entry) {
  bool _return = false;
  kinematicsTactileInfo_IDL_set_frame helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool kinematicsTactileInfo_IDL::set_frame(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle kinematicsTactileInfo_IDL::get_pose() {
  yarp::os::Bottle _return;
  kinematicsTactileInfo_IDL_get_pose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle kinematicsTactileInfo_IDL::get_pose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
yarp::os::Bottle kinematicsTactileInfo_IDL::get_tactile_data() {
  yarp::os::Bottle _return;
  kinematicsTactileInfo_IDL_get_tactile_data helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle kinematicsTactileInfo_IDL::get_tactile_data()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool kinematicsTactileInfo_IDL::save_pose() {
  bool _return = false;
  kinematicsTactileInfo_IDL_save_pose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool kinematicsTactileInfo_IDL::save_pose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool kinematicsTactileInfo_IDL::save_tactile_data() {
  bool _return = false;
  kinematicsTactileInfo_IDL_save_tactile_data helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool kinematicsTactileInfo_IDL::save_tactile_data()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool kinematicsTactileInfo_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
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
    if (tag == "save_pose") {
      bool _return;
      _return = save_pose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "save_tactile_data") {
      bool _return;
      _return = save_tactile_data();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
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

std::vector<std::string> kinematicsTactileInfo_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("get_frame");
    helpString.push_back("set_frame");
    helpString.push_back("get_pose");
    helpString.push_back("get_tactile_data");
    helpString.push_back("save_pose");
    helpString.push_back("save_tactile_data");
    helpString.push_back("help");
  }
  else {
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
    if (functionName=="save_pose") {
      helpString.push_back("bool save_pose() ");
      helpString.push_back("Save pose information from kinematics ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="save_tactile_data") {
      helpString.push_back("bool save_tactile_data() ");
      helpString.push_back("Save tactile information from fingertips ");
      helpString.push_back("@return true/false on success/failure ");
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


