# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM

# The top-level build directory on which CMake was Run.
CMAKE_BINARY_DIR = /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug

# Include any dependencies generated for this target.
include ORUtils/CMakeFiles/ORUtils.dir/depend.make

# Include the progress variables for this target.
include ORUtils/CMakeFiles/ORUtils.dir/progress.make

# Include the compile flags for this target's objects.
include ORUtils/CMakeFiles/ORUtils.dir/flags.make

ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o: ORUtils/CMakeFiles/ORUtils.dir/flags.make
ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o: ../ORUtils/FileUtils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ORUtils.dir/FileUtils.cpp.o -c /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/FileUtils.cpp

ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ORUtils.dir/FileUtils.cpp.i"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/FileUtils.cpp > CMakeFiles/ORUtils.dir/FileUtils.cpp.i

ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ORUtils.dir/FileUtils.cpp.s"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/FileUtils.cpp -o CMakeFiles/ORUtils.dir/FileUtils.cpp.s

ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.requires:

.PHONY : ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.requires

ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.provides: ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.requires
	$(MAKE) -f ORUtils/CMakeFiles/ORUtils.dir/build.make ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.provides.build
.PHONY : ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.provides

ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.provides.build: ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o


ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o: ORUtils/CMakeFiles/ORUtils.dir/flags.make
ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o: ../ORUtils/KeyValueConfig.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o -c /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/KeyValueConfig.cpp

ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.i"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/KeyValueConfig.cpp > CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.i

ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.s"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/KeyValueConfig.cpp -o CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.s

ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.requires:

.PHONY : ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.requires

ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.provides: ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.requires
	$(MAKE) -f ORUtils/CMakeFiles/ORUtils.dir/build.make ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.provides.build
.PHONY : ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.provides

ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.provides.build: ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o


ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o: ORUtils/CMakeFiles/ORUtils.dir/flags.make
ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o: ../ORUtils/SE3Pose.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ORUtils.dir/SE3Pose.cpp.o -c /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.cpp

ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ORUtils.dir/SE3Pose.cpp.i"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.cpp > CMakeFiles/ORUtils.dir/SE3Pose.cpp.i

ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ORUtils.dir/SE3Pose.cpp.s"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.cpp -o CMakeFiles/ORUtils.dir/SE3Pose.cpp.s

ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.requires:

.PHONY : ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.requires

ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.provides: ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.requires
	$(MAKE) -f ORUtils/CMakeFiles/ORUtils.dir/build.make ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.provides.build
.PHONY : ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.provides

ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.provides.build: ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o


# Object files for target ORUtils
ORUtils_OBJECTS = \
"CMakeFiles/ORUtils.dir/FileUtils.cpp.o" \
"CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o" \
"CMakeFiles/ORUtils.dir/SE3Pose.cpp.o"

# External object files for target ORUtils
ORUtils_EXTERNAL_OBJECTS =

ORUtils/libORUtils.a: ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o
ORUtils/libORUtils.a: ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o
ORUtils/libORUtils.a: ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o
ORUtils/libORUtils.a: ORUtils/CMakeFiles/ORUtils.dir/build.make
ORUtils/libORUtils.a: ORUtils/CMakeFiles/ORUtils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libORUtils.a"
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && $(CMAKE_COMMAND) -P CMakeFiles/ORUtils.dir/cmake_clean_target.cmake
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ORUtils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ORUtils/CMakeFiles/ORUtils.dir/build: ORUtils/libORUtils.a

.PHONY : ORUtils/CMakeFiles/ORUtils.dir/build

ORUtils/CMakeFiles/ORUtils.dir/requires: ORUtils/CMakeFiles/ORUtils.dir/FileUtils.cpp.o.requires
ORUtils/CMakeFiles/ORUtils.dir/requires: ORUtils/CMakeFiles/ORUtils.dir/KeyValueConfig.cpp.o.requires
ORUtils/CMakeFiles/ORUtils.dir/requires: ORUtils/CMakeFiles/ORUtils.dir/SE3Pose.cpp.o.requires

.PHONY : ORUtils/CMakeFiles/ORUtils.dir/requires

ORUtils/CMakeFiles/ORUtils.dir/clean:
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils && $(CMAKE_COMMAND) -P CMakeFiles/ORUtils.dir/cmake_clean.cmake
.PHONY : ORUtils/CMakeFiles/ORUtils.dir/clean

ORUtils/CMakeFiles/ORUtils.dir/depend:
	cd /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/ORUtils /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils /mnt/d/LocalDocuments/MT_Local_Repos/InfiniTAM/InfiniTAM/cmake-build-debug/ORUtils/CMakeFiles/ORUtils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ORUtils/CMakeFiles/ORUtils.dir/depend

