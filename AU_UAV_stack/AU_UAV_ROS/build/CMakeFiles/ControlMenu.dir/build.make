# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build

# Include any dependencies generated for this target.
include CMakeFiles/ControlMenu.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ControlMenu.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ControlMenu.dir/flags.make

CMakeFiles/ControlMenu.dir/src/ControlMenu.o: CMakeFiles/ControlMenu.dir/flags.make
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: ../src/ControlMenu.cpp
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: ../manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/ControlMenu.dir/src/ControlMenu.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ControlMenu.dir/src/ControlMenu.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/ControlMenu.dir/src/ControlMenu.o -c /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/ControlMenu.cpp

CMakeFiles/ControlMenu.dir/src/ControlMenu.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ControlMenu.dir/src/ControlMenu.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/ControlMenu.cpp > CMakeFiles/ControlMenu.dir/src/ControlMenu.i

CMakeFiles/ControlMenu.dir/src/ControlMenu.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ControlMenu.dir/src/ControlMenu.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/ControlMenu.cpp -o CMakeFiles/ControlMenu.dir/src/ControlMenu.s

CMakeFiles/ControlMenu.dir/src/ControlMenu.o.requires:
.PHONY : CMakeFiles/ControlMenu.dir/src/ControlMenu.o.requires

CMakeFiles/ControlMenu.dir/src/ControlMenu.o.provides: CMakeFiles/ControlMenu.dir/src/ControlMenu.o.requires
	$(MAKE) -f CMakeFiles/ControlMenu.dir/build.make CMakeFiles/ControlMenu.dir/src/ControlMenu.o.provides.build
.PHONY : CMakeFiles/ControlMenu.dir/src/ControlMenu.o.provides

CMakeFiles/ControlMenu.dir/src/ControlMenu.o.provides.build: CMakeFiles/ControlMenu.dir/src/ControlMenu.o

CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: CMakeFiles/ControlMenu.dir/flags.make
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: ../src/AU_UAV_ROS/standardDefs.cpp
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: ../manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o -c /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/standardDefs.cpp

CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/standardDefs.cpp > CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.i

CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/standardDefs.cpp -o CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.s

CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.requires:
.PHONY : CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.requires

CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.provides: CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.requires
	$(MAKE) -f CMakeFiles/ControlMenu.dir/build.make CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.provides.build
.PHONY : CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.provides

CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.provides.build: CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o

# Object files for target ControlMenu
ControlMenu_OBJECTS = \
"CMakeFiles/ControlMenu.dir/src/ControlMenu.o" \
"CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o"

# External object files for target ControlMenu
ControlMenu_EXTERNAL_OBJECTS =

../bin/ControlMenu: CMakeFiles/ControlMenu.dir/src/ControlMenu.o
../bin/ControlMenu: CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o
../bin/ControlMenu: CMakeFiles/ControlMenu.dir/build.make
../bin/ControlMenu: CMakeFiles/ControlMenu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/ControlMenu"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ControlMenu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ControlMenu.dir/build: ../bin/ControlMenu
.PHONY : CMakeFiles/ControlMenu.dir/build

CMakeFiles/ControlMenu.dir/requires: CMakeFiles/ControlMenu.dir/src/ControlMenu.o.requires
CMakeFiles/ControlMenu.dir/requires: CMakeFiles/ControlMenu.dir/src/AU_UAV_ROS/standardDefs.o.requires
.PHONY : CMakeFiles/ControlMenu.dir/requires

CMakeFiles/ControlMenu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ControlMenu.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ControlMenu.dir/clean

CMakeFiles/ControlMenu.dir/depend:
	cd /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/build/CMakeFiles/ControlMenu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ControlMenu.dir/depend

