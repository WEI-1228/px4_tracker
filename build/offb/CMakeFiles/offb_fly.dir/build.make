# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ljw/uav_offb/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ljw/uav_offb/build

# Include any dependencies generated for this target.
include offb/CMakeFiles/offb_fly.dir/depend.make

# Include the progress variables for this target.
include offb/CMakeFiles/offb_fly.dir/progress.make

# Include the compile flags for this target's objects.
include offb/CMakeFiles/offb_fly.dir/flags.make

offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o: offb/CMakeFiles/offb_fly.dir/flags.make
offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o: /home/ljw/uav_offb/src/offb/src/offb_fly.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ljw/uav_offb/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o"
	cd /home/ljw/uav_offb/build/offb && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o -c /home/ljw/uav_offb/src/offb/src/offb_fly.cpp

offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/offb_fly.dir/src/offb_fly.cpp.i"
	cd /home/ljw/uav_offb/build/offb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ljw/uav_offb/src/offb/src/offb_fly.cpp > CMakeFiles/offb_fly.dir/src/offb_fly.cpp.i

offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/offb_fly.dir/src/offb_fly.cpp.s"
	cd /home/ljw/uav_offb/build/offb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ljw/uav_offb/src/offb/src/offb_fly.cpp -o CMakeFiles/offb_fly.dir/src/offb_fly.cpp.s

offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.requires:

.PHONY : offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.requires

offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.provides: offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.requires
	$(MAKE) -f offb/CMakeFiles/offb_fly.dir/build.make offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.provides.build
.PHONY : offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.provides

offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.provides.build: offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o


# Object files for target offb_fly
offb_fly_OBJECTS = \
"CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o"

# External object files for target offb_fly
offb_fly_EXTERNAL_OBJECTS =

/home/ljw/uav_offb/devel/lib/offb/offb_fly: offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o
/home/ljw/uav_offb/devel/lib/offb/offb_fly: offb/CMakeFiles/offb_fly.dir/build.make
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libmavros.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libdiagnostic_updater.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libeigen_conversions.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libmavconn.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libclass_loader.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/libPocoFoundation.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libdl.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libroslib.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/librospack.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libtf2_ros.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libactionlib.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libmessage_filters.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libtf2.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libroscpp.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/librosconsole.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/librostime.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /opt/ros/melodic/lib/libcpp_common.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/ljw/uav_offb/devel/lib/offb/offb_fly: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/ljw/uav_offb/devel/lib/offb/offb_fly: offb/CMakeFiles/offb_fly.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ljw/uav_offb/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ljw/uav_offb/devel/lib/offb/offb_fly"
	cd /home/ljw/uav_offb/build/offb && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/offb_fly.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
offb/CMakeFiles/offb_fly.dir/build: /home/ljw/uav_offb/devel/lib/offb/offb_fly

.PHONY : offb/CMakeFiles/offb_fly.dir/build

offb/CMakeFiles/offb_fly.dir/requires: offb/CMakeFiles/offb_fly.dir/src/offb_fly.cpp.o.requires

.PHONY : offb/CMakeFiles/offb_fly.dir/requires

offb/CMakeFiles/offb_fly.dir/clean:
	cd /home/ljw/uav_offb/build/offb && $(CMAKE_COMMAND) -P CMakeFiles/offb_fly.dir/cmake_clean.cmake
.PHONY : offb/CMakeFiles/offb_fly.dir/clean

offb/CMakeFiles/offb_fly.dir/depend:
	cd /home/ljw/uav_offb/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ljw/uav_offb/src /home/ljw/uav_offb/src/offb /home/ljw/uav_offb/build /home/ljw/uav_offb/build/offb /home/ljw/uav_offb/build/offb/CMakeFiles/offb_fly.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : offb/CMakeFiles/offb_fly.dir/depend

