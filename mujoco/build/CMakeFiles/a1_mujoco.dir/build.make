# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jame/Data/a1_mujoco/mujoco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jame/Data/a1_mujoco/mujoco/build

# Include any dependencies generated for this target.
include CMakeFiles/a1_mujoco.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/a1_mujoco.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/a1_mujoco.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a1_mujoco.dir/flags.make

CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o: CMakeFiles/a1_mujoco.dir/flags.make
CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o: ../src/mj_bridge.cpp
CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o: CMakeFiles/a1_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jame/Data/a1_mujoco/mujoco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o -MF CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o.d -o CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o -c /home/jame/Data/a1_mujoco/mujoco/src/mj_bridge.cpp

CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jame/Data/a1_mujoco/mujoco/src/mj_bridge.cpp > CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.i

CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jame/Data/a1_mujoco/mujoco/src/mj_bridge.cpp -o CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.s

CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o: CMakeFiles/a1_mujoco.dir/flags.make
CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o: ../src/mj_simulate.cpp
CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o: CMakeFiles/a1_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jame/Data/a1_mujoco/mujoco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o -MF CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o.d -o CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o -c /home/jame/Data/a1_mujoco/mujoco/src/mj_simulate.cpp

CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jame/Data/a1_mujoco/mujoco/src/mj_simulate.cpp > CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.i

CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jame/Data/a1_mujoco/mujoco/src/mj_simulate.cpp -o CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.s

CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o: CMakeFiles/a1_mujoco.dir/flags.make
CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o: ../mujoco210/include/uitools.c
CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o: CMakeFiles/a1_mujoco.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jame/Data/a1_mujoco/mujoco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o -MF CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o.d -o CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o -c /home/jame/Data/a1_mujoco/mujoco/mujoco210/include/uitools.c

CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jame/Data/a1_mujoco/mujoco/mujoco210/include/uitools.c > CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.i

CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jame/Data/a1_mujoco/mujoco/mujoco210/include/uitools.c -o CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.s

# Object files for target a1_mujoco
a1_mujoco_OBJECTS = \
"CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o" \
"CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o" \
"CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o"

# External object files for target a1_mujoco
a1_mujoco_EXTERNAL_OBJECTS =

a1_mujoco: CMakeFiles/a1_mujoco.dir/src/mj_bridge.cpp.o
a1_mujoco: CMakeFiles/a1_mujoco.dir/src/mj_simulate.cpp.o
a1_mujoco: CMakeFiles/a1_mujoco.dir/mujoco210/include/uitools.c.o
a1_mujoco: CMakeFiles/a1_mujoco.dir/build.make
a1_mujoco: /opt/openrobots/lib/libpinocchio.so
a1_mujoco: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
a1_mujoco: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
a1_mujoco: /usr/lib/x86_64-linux-gnu/libboost_system.so
a1_mujoco: CMakeFiles/a1_mujoco.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jame/Data/a1_mujoco/mujoco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable a1_mujoco"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a1_mujoco.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a1_mujoco.dir/build: a1_mujoco
.PHONY : CMakeFiles/a1_mujoco.dir/build

CMakeFiles/a1_mujoco.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a1_mujoco.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a1_mujoco.dir/clean

CMakeFiles/a1_mujoco.dir/depend:
	cd /home/jame/Data/a1_mujoco/mujoco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jame/Data/a1_mujoco/mujoco /home/jame/Data/a1_mujoco/mujoco /home/jame/Data/a1_mujoco/mujoco/build /home/jame/Data/a1_mujoco/mujoco/build /home/jame/Data/a1_mujoco/mujoco/build/CMakeFiles/a1_mujoco.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/a1_mujoco.dir/depend
