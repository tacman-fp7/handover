# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build

# Include any dependencies generated for this target.
include CMakeFiles/localizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/localizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/localizer.dir/flags.make

src/localizer_IDL.cpp: ../src/idl.thrift
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating code from src/idl.thrift"
	cd /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer && /usr/local/bin/yarpidl_thrift --gen yarp:include_prefix --I /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer --out /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles/yarpidl_thrift/src src/idl.thrift
	cd /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer && /usr/local/bin/cmake -E copy /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles/yarpidl_thrift/src/localizer_IDL.cpp /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/src/localizer_IDL.cpp
	cd /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer && /usr/local/bin/cmake -E copy /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles/yarpidl_thrift/src/localizer_IDL.h /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/include/src/localizer_IDL.h

include/src/localizer_IDL.h: src/localizer_IDL.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate include/src/localizer_IDL.h

CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o: CMakeFiles/localizer.dir/flags.make
CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o: ../src/unscentedParticleFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o -c /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/unscentedParticleFilter.cpp

CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/unscentedParticleFilter.cpp > CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.i

CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/unscentedParticleFilter.cpp -o CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.s

CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.requires:

.PHONY : CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.requires

CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.provides: CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/localizer.dir/build.make CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.provides.build
.PHONY : CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.provides

CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.provides.build: CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o


CMakeFiles/localizer.dir/src/main.cpp.o: CMakeFiles/localizer.dir/flags.make
CMakeFiles/localizer.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/localizer.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/localizer.dir/src/main.cpp.o -c /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/main.cpp

CMakeFiles/localizer.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/localizer.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/main.cpp > CMakeFiles/localizer.dir/src/main.cpp.i

CMakeFiles/localizer.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/localizer.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/main.cpp -o CMakeFiles/localizer.dir/src/main.cpp.s

CMakeFiles/localizer.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/localizer.dir/src/main.cpp.o.requires

CMakeFiles/localizer.dir/src/main.cpp.o.provides: CMakeFiles/localizer.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/localizer.dir/build.make CMakeFiles/localizer.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/localizer.dir/src/main.cpp.o.provides

CMakeFiles/localizer.dir/src/main.cpp.o.provides.build: CMakeFiles/localizer.dir/src/main.cpp.o


CMakeFiles/localizer.dir/src/localizer.cpp.o: CMakeFiles/localizer.dir/flags.make
CMakeFiles/localizer.dir/src/localizer.cpp.o: ../src/localizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/localizer.dir/src/localizer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/localizer.dir/src/localizer.cpp.o -c /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/localizer.cpp

CMakeFiles/localizer.dir/src/localizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/localizer.dir/src/localizer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/localizer.cpp > CMakeFiles/localizer.dir/src/localizer.cpp.i

CMakeFiles/localizer.dir/src/localizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/localizer.dir/src/localizer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/localizer.cpp -o CMakeFiles/localizer.dir/src/localizer.cpp.s

CMakeFiles/localizer.dir/src/localizer.cpp.o.requires:

.PHONY : CMakeFiles/localizer.dir/src/localizer.cpp.o.requires

CMakeFiles/localizer.dir/src/localizer.cpp.o.provides: CMakeFiles/localizer.dir/src/localizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/localizer.dir/build.make CMakeFiles/localizer.dir/src/localizer.cpp.o.provides.build
.PHONY : CMakeFiles/localizer.dir/src/localizer.cpp.o.provides

CMakeFiles/localizer.dir/src/localizer.cpp.o.provides.build: CMakeFiles/localizer.dir/src/localizer.cpp.o


CMakeFiles/localizer.dir/src/scalingSeries.cpp.o: CMakeFiles/localizer.dir/flags.make
CMakeFiles/localizer.dir/src/scalingSeries.cpp.o: ../src/scalingSeries.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/localizer.dir/src/scalingSeries.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/localizer.dir/src/scalingSeries.cpp.o -c /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/scalingSeries.cpp

CMakeFiles/localizer.dir/src/scalingSeries.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/localizer.dir/src/scalingSeries.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/scalingSeries.cpp > CMakeFiles/localizer.dir/src/scalingSeries.cpp.i

CMakeFiles/localizer.dir/src/scalingSeries.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/localizer.dir/src/scalingSeries.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/src/scalingSeries.cpp -o CMakeFiles/localizer.dir/src/scalingSeries.cpp.s

CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.requires:

.PHONY : CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.requires

CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.provides: CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.requires
	$(MAKE) -f CMakeFiles/localizer.dir/build.make CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.provides.build
.PHONY : CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.provides

CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.provides.build: CMakeFiles/localizer.dir/src/scalingSeries.cpp.o


CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o: CMakeFiles/localizer.dir/flags.make
CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o: src/localizer_IDL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o -c /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/src/localizer_IDL.cpp

CMakeFiles/localizer.dir/src/localizer_IDL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/localizer.dir/src/localizer_IDL.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/src/localizer_IDL.cpp > CMakeFiles/localizer.dir/src/localizer_IDL.cpp.i

CMakeFiles/localizer.dir/src/localizer_IDL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/localizer.dir/src/localizer_IDL.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/src/localizer_IDL.cpp -o CMakeFiles/localizer.dir/src/localizer_IDL.cpp.s

CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.requires:

.PHONY : CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.requires

CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.provides: CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.requires
	$(MAKE) -f CMakeFiles/localizer.dir/build.make CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.provides.build
.PHONY : CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.provides

CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.provides.build: CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o


# Object files for target localizer
localizer_OBJECTS = \
"CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o" \
"CMakeFiles/localizer.dir/src/main.cpp.o" \
"CMakeFiles/localizer.dir/src/localizer.cpp.o" \
"CMakeFiles/localizer.dir/src/scalingSeries.cpp.o" \
"CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o"

# External object files for target localizer
localizer_EXTERNAL_OBJECTS =

bin/localizer: CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o
bin/localizer: CMakeFiles/localizer.dir/src/main.cpp.o
bin/localizer: CMakeFiles/localizer.dir/src/localizer.cpp.o
bin/localizer: CMakeFiles/localizer.dir/src/scalingSeries.cpp.o
bin/localizer: CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o
bin/localizer: CMakeFiles/localizer.dir/build.make
bin/localizer: /usr/lib/x86_64-linux-gnu/libmpfr.so
bin/localizer: /usr/lib/x86_64-linux-gnu/libgmp.so
bin/localizer: /usr/local/lib/libCGAL.so.11.0.1
bin/localizer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/localizer: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/localizer: /usr/local/lib/libYARP_dev.so.2.3.67.3
bin/localizer: /usr/local/lib/libYARP_name.so.2.3.67.3
bin/localizer: /usr/local/lib/libYARP_init.so.2.3.67.3
bin/localizer: /usr/local/lib/libYARP_math.so.2.3.67.3
bin/localizer: /usr/local/lib/libYARP_gsl.so.2.3.67.3
bin/localizer: /usr/local/lib/libYARP_sig.so.2.3.67.3
bin/localizer: /usr/local/lib/libYARP_OS.so.2.3.67.3
bin/localizer: CMakeFiles/localizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable bin/localizer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/localizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/localizer.dir/build: bin/localizer

.PHONY : CMakeFiles/localizer.dir/build

CMakeFiles/localizer.dir/requires: CMakeFiles/localizer.dir/src/unscentedParticleFilter.cpp.o.requires
CMakeFiles/localizer.dir/requires: CMakeFiles/localizer.dir/src/main.cpp.o.requires
CMakeFiles/localizer.dir/requires: CMakeFiles/localizer.dir/src/localizer.cpp.o.requires
CMakeFiles/localizer.dir/requires: CMakeFiles/localizer.dir/src/scalingSeries.cpp.o.requires
CMakeFiles/localizer.dir/requires: CMakeFiles/localizer.dir/src/localizer_IDL.cpp.o.requires

.PHONY : CMakeFiles/localizer.dir/requires

CMakeFiles/localizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/localizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/localizer.dir/clean

CMakeFiles/localizer.dir/depend: src/localizer_IDL.cpp
CMakeFiles/localizer.dir/depend: include/src/localizer_IDL.h
	cd /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build /home/gvezzani/Desktop/PhD/Anno_2/handover_tacman/handover/in-hand-localization/src/localizer/build/CMakeFiles/localizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/localizer.dir/depend
