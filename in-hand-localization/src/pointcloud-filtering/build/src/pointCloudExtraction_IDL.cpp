// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <pointCloudExtraction_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class pointCloudExtraction_IDL_clear_points : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class pointCloudExtraction_IDL_set_auto_seed : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class pointCloudExtraction_IDL_set_saving : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class pointCloudExtraction_IDL_set_format : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class pointCloudExtraction_IDL_set_filename : public yarp::os::Portable {
public:
  std::string entry;
  bool _return;
  void init(const std::string& entry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool pointCloudExtraction_IDL_clear_points::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("clear_points",1,2)) return false;
  return true;
}

bool pointCloudExtraction_IDL_clear_points::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void pointCloudExtraction_IDL_clear_points::init() {
  _return = false;
}

bool pointCloudExtraction_IDL_set_auto_seed::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("set_auto_seed",1,3)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool pointCloudExtraction_IDL_set_auto_seed::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void pointCloudExtraction_IDL_set_auto_seed::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool pointCloudExtraction_IDL_set_saving::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_saving",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool pointCloudExtraction_IDL_set_saving::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void pointCloudExtraction_IDL_set_saving::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool pointCloudExtraction_IDL_set_format::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_format",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool pointCloudExtraction_IDL_set_format::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void pointCloudExtraction_IDL_set_format::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

bool pointCloudExtraction_IDL_set_filename::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_filename",1,2)) return false;
  if (!writer.writeString(entry)) return false;
  return true;
}

bool pointCloudExtraction_IDL_set_filename::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void pointCloudExtraction_IDL_set_filename::init(const std::string& entry) {
  _return = false;
  this->entry = entry;
}

pointCloudExtraction_IDL::pointCloudExtraction_IDL() {
  yarp().setOwner(*this);
}
bool pointCloudExtraction_IDL::clear_points() {
  bool _return = false;
  pointCloudExtraction_IDL_clear_points helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool pointCloudExtraction_IDL::clear_points()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool pointCloudExtraction_IDL::set_auto_seed(const std::string& entry) {
  bool _return = false;
  pointCloudExtraction_IDL_set_auto_seed helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool pointCloudExtraction_IDL::set_auto_seed(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool pointCloudExtraction_IDL::set_saving(const std::string& entry) {
  bool _return = false;
  pointCloudExtraction_IDL_set_saving helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool pointCloudExtraction_IDL::set_saving(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool pointCloudExtraction_IDL::set_format(const std::string& entry) {
  bool _return = false;
  pointCloudExtraction_IDL_set_format helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool pointCloudExtraction_IDL::set_format(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool pointCloudExtraction_IDL::set_filename(const std::string& entry) {
  bool _return = false;
  pointCloudExtraction_IDL_set_filename helper;
  helper.init(entry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool pointCloudExtraction_IDL::set_filename(const std::string& entry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool pointCloudExtraction_IDL::read(yarp::os::ConnectionReader& connection) {
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
    if (tag == "set_auto_seed") {
      std::string entry;
      if (!reader.readString(entry)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_auto_seed(entry);
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

std::vector<std::string> pointCloudExtraction_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("clear_points");
    helpString.push_back("set_auto_seed");
    helpString.push_back("set_saving");
    helpString.push_back("set_format");
    helpString.push_back("set_filename");
    helpString.push_back("help");
  }
  else {
    if (functionName=="clear_points") {
      helpString.push_back("bool clear_points() ");
      helpString.push_back("Clear collected points ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_auto_seed") {
      helpString.push_back("bool set_auto_seed(const std::string& entry) ");
      helpString.push_back("Set or unset autoseed variable equal to true ");
      helpString.push_back("@param entry is yes or no ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_saving") {
      helpString.push_back("bool set_saving(const std::string& entry) ");
      helpString.push_back("save or not ");
      helpString.push_back("@param entry is yes or no ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_format") {
      helpString.push_back("bool set_format(const std::string& entry) ");
      helpString.push_back("Select the saving format ");
      helpString.push_back("@param entry is ply or off ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="set_filename") {
      helpString.push_back("bool set_filename(const std::string& entry) ");
      helpString.push_back("set filename for saving ");
      helpString.push_back("@param entry is the name of file ");
      helpString.push_back("@return true/false on success/failure. ");
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


