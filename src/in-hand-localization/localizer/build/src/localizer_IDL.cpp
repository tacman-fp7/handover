// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <src/localizer_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class localizer_IDL_get_estimated_pose : public yarp::os::Portable {
public:
  yarp::os::Bottle _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class localizer_IDL_go_acquire : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class localizer_IDL_go_localize : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool localizer_IDL_get_estimated_pose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("get_estimated_pose",1,3)) return false;
  return true;
}

bool localizer_IDL_get_estimated_pose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void localizer_IDL_get_estimated_pose::init() {
}

bool localizer_IDL_go_acquire::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("go_acquire",1,2)) return false;
  return true;
}

bool localizer_IDL_go_acquire::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void localizer_IDL_go_acquire::init() {
  _return = false;
}

bool localizer_IDL_go_localize::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("go_localize",1,2)) return false;
  return true;
}

bool localizer_IDL_go_localize::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void localizer_IDL_go_localize::init() {
  _return = false;
}

localizer_IDL::localizer_IDL() {
  yarp().setOwner(*this);
}
yarp::os::Bottle localizer_IDL::get_estimated_pose() {
  yarp::os::Bottle _return;
  localizer_IDL_get_estimated_pose helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","yarp::os::Bottle localizer_IDL::get_estimated_pose()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool localizer_IDL::go_acquire() {
  bool _return = false;
  localizer_IDL_go_acquire helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool localizer_IDL::go_acquire()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool localizer_IDL::go_localize() {
  bool _return = false;
  localizer_IDL_go_localize helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool localizer_IDL::go_localize()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool localizer_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "get_estimated_pose") {
      yarp::os::Bottle _return;
      _return = get_estimated_pose();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.write(_return)) return false;
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
    if (tag == "go_localize") {
      bool _return;
      _return = go_localize();
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

std::vector<std::string> localizer_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("get_estimated_pose");
    helpString.push_back("go_acquire");
    helpString.push_back("go_localize");
    helpString.push_back("help");
  }
  else {
    if (functionName=="get_estimated_pose") {
      helpString.push_back("yarp::os::Bottle get_estimated_pose() ");
      helpString.push_back("Get estimated pose ");
      helpString.push_back("@return the 6D vector representing the object estimated pose ");
    }
    if (functionName=="go_acquire") {
      helpString.push_back("bool go_acquire() ");
      helpString.push_back("Set acquire variable true and acquire points ");
      helpString.push_back("@return true ");
    }
    if (functionName=="go_localize") {
      helpString.push_back("bool go_localize() ");
      helpString.push_back("Set localize variable true and start localization ");
      helpString.push_back("@return true ");
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


