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
include CMakeFiles/test_xarm_velo_move.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_xarm_velo_move.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_xarm_velo_move.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_xarm_velo_move.dir/flags.make

CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o: CMakeFiles/test_xarm_velo_move.dir/flags.make
CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o: /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/test/test_xarm_velo_move.cpp
CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o: CMakeFiles/test_xarm_velo_move.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o -MF CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o.d -o CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o -c /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/test/test_xarm_velo_move.cpp

CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/test/test_xarm_velo_move.cpp > CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.i

CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api/test/test_xarm_velo_move.cpp -o CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.s

# Object files for target test_xarm_velo_move
test_xarm_velo_move_OBJECTS = \
"CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o"

# External object files for target test_xarm_velo_move
test_xarm_velo_move_EXTERNAL_OBJECTS =

/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: CMakeFiles/test_xarm_velo_move.dir/test/test_xarm_velo_move.cpp.o
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: CMakeFiles/test_xarm_velo_move.dir/build.make
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/libxarm_ros_client.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/libactionlib.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/libroscpp.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/librosconsole.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/librostime.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /opt/ros/noetic/lib/libcpp_common.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: /home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_sdk/lib/libxarm_cxx_sdk.so
/home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move: CMakeFiles/test_xarm_velo_move.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_xarm_velo_move.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_xarm_velo_move.dir/build: /home/edgarcancinoe/xarm6/catkin_ws/devel/.private/xarm_api/lib/xarm_api/test_xarm_velo_move
.PHONY : CMakeFiles/test_xarm_velo_move.dir/build

CMakeFiles/test_xarm_velo_move.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_xarm_velo_move.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_xarm_velo_move.dir/clean

CMakeFiles/test_xarm_velo_move.dir/depend:
	cd /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/src/xarm_ros/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api /home/edgarcancinoe/xarm6/catkin_ws/build/xarm_api/CMakeFiles/test_xarm_velo_move.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/test_xarm_velo_move.dir/depend

