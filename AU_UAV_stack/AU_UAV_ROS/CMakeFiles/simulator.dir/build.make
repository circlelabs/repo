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
CMAKE_BINARY_DIR = /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS

# Include any dependencies generated for this target.
include CMakeFiles/simulator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simulator.dir/flags.make

CMakeFiles/simulator.dir/src/simulator.o: CMakeFiles/simulator.dir/flags.make
CMakeFiles/simulator.dir/src/simulator.o: src/simulator.cpp
CMakeFiles/simulator.dir/src/simulator.o: manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/simulator.dir/src/simulator.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/simulator.dir/src/simulator.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/simulator.dir/src/simulator.o -c /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/simulator.cpp

CMakeFiles/simulator.dir/src/simulator.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulator.dir/src/simulator.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/simulator.cpp > CMakeFiles/simulator.dir/src/simulator.i

CMakeFiles/simulator.dir/src/simulator.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulator.dir/src/simulator.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/simulator.cpp -o CMakeFiles/simulator.dir/src/simulator.s

CMakeFiles/simulator.dir/src/simulator.o.requires:
.PHONY : CMakeFiles/simulator.dir/src/simulator.o.requires

CMakeFiles/simulator.dir/src/simulator.o.provides: CMakeFiles/simulator.dir/src/simulator.o.requires
	$(MAKE) -f CMakeFiles/simulator.dir/build.make CMakeFiles/simulator.dir/src/simulator.o.provides.build
.PHONY : CMakeFiles/simulator.dir/src/simulator.o.provides

CMakeFiles/simulator.dir/src/simulator.o.provides.build: CMakeFiles/simulator.dir/src/simulator.o

CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: CMakeFiles/simulator.dir/flags.make
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: src/AU_UAV_ROS/SimulatedPlane.cpp
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o -c /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/SimulatedPlane.cpp

CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/SimulatedPlane.cpp > CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.i

CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/SimulatedPlane.cpp -o CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.s

CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.requires:
.PHONY : CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.requires

CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.provides: CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.requires
	$(MAKE) -f CMakeFiles/simulator.dir/build.make CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.provides.build
.PHONY : CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.provides

CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.provides.build: CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o

CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: CMakeFiles/simulator.dir/flags.make
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: src/AU_UAV_ROS/standardDefs.cpp
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o -c /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/standardDefs.cpp

CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/standardDefs.cpp > CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.i

CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/src/AU_UAV_ROS/standardDefs.cpp -o CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.s

CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.requires:
.PHONY : CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.requires

CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.provides: CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.requires
	$(MAKE) -f CMakeFiles/simulator.dir/build.make CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.provides.build
.PHONY : CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.provides

CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.provides.build: CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o

# Object files for target simulator
simulator_OBJECTS = \
"CMakeFiles/simulator.dir/src/simulator.o" \
"CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o" \
"CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o"

# External object files for target simulator
simulator_EXTERNAL_OBJECTS =

bin/simulator: CMakeFiles/simulator.dir/src/simulator.o
bin/simulator: CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o
bin/simulator: CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o
bin/simulator: CMakeFiles/simulator.dir/build.make
bin/simulator: CMakeFiles/simulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/simulator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simulator.dir/build: bin/simulator
.PHONY : CMakeFiles/simulator.dir/build

CMakeFiles/simulator.dir/requires: CMakeFiles/simulator.dir/src/simulator.o.requires
CMakeFiles/simulator.dir/requires: CMakeFiles/simulator.dir/src/AU_UAV_ROS/SimulatedPlane.o.requires
CMakeFiles/simulator.dir/requires: CMakeFiles/simulator.dir/src/AU_UAV_ROS/standardDefs.o.requires
.PHONY : CMakeFiles/simulator.dir/requires

CMakeFiles/simulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simulator.dir/clean

CMakeFiles/simulator.dir/depend:
	cd /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS /home/ericwestman/ros_workspace/repo/AU_UAV_stack/AU_UAV_ROS/CMakeFiles/simulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simulator.dir/depend

