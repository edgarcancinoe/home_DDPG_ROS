# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/edgarcancinoe/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/edgarcancinoe/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api

# Include any dependencies generated for this target.
include CMakeFiles/xarm_ros_client.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/xarm_ros_client.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/xarm_ros_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xarm_ros_client.dir/flags.make

CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o: CMakeFiles/xarm_ros_client.dir/flags.make
CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o: /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/src/xarm_ros_client.cpp
CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o: CMakeFiles/xarm_ros_client.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o -MF CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o.d -o CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o -c /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/src/xarm_ros_client.cpp

CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/src/xarm_ros_client.cpp > CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.i

CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/src/xarm_ros_client.cpp -o CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.s

# Object files for target xarm_ros_client
xarm_ros_client_OBJECTS = \
"CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o"

# External object files for target xarm_ros_client
xarm_ros_client_EXTERNAL_OBJECTS =

/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/libxarm_ros_client.so: CMakeFiles/xarm_ros_client.dir/src/xarm_ros_client.cpp.o
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/libxarm_ros_client.so: CMakeFiles/xarm_ros_client.dir/build.make
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/libxarm_ros_client.so: CMakeFiles/xarm_ros_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/libxarm_ros_client.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xarm_ros_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xarm_ros_client.dir/build: /home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/libxarm_ros_client.so
.PHONY : CMakeFiles/xarm_ros_client.dir/build

CMakeFiles/xarm_ros_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xarm_ros_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xarm_ros_client.dir/clean

CMakeFiles/xarm_ros_client.dir/depend:
	cd /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api/CMakeFiles/xarm_ros_client.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/xarm_ros_client.dir/depend

