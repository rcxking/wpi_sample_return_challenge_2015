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
CMAKE_SOURCE_DIR = /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/build

# Utility rule file for object_recognition_gencpp.

# Include the progress variables for this target.
include object_recognition/CMakeFiles/object_recognition_gencpp.dir/progress.make

object_recognition/CMakeFiles/object_recognition_gencpp:

object_recognition_gencpp: object_recognition/CMakeFiles/object_recognition_gencpp
object_recognition_gencpp: object_recognition/CMakeFiles/object_recognition_gencpp.dir/build.make
.PHONY : object_recognition_gencpp

# Rule to build all files generated by this target.
object_recognition/CMakeFiles/object_recognition_gencpp.dir/build: object_recognition_gencpp
.PHONY : object_recognition/CMakeFiles/object_recognition_gencpp.dir/build

object_recognition/CMakeFiles/object_recognition_gencpp.dir/clean:
	cd /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/build/object_recognition && $(CMAKE_COMMAND) -P CMakeFiles/object_recognition_gencpp.dir/cmake_clean.cmake
.PHONY : object_recognition/CMakeFiles/object_recognition_gencpp.dir/clean

object_recognition/CMakeFiles/object_recognition_gencpp.dir/depend:
	cd /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/src /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/src/object_recognition /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/build /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/build/object_recognition /home/rockie/wpi_sample_return_challenge_2015/test_indigo_workspace/build/object_recognition/CMakeFiles/object_recognition_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : object_recognition/CMakeFiles/object_recognition_gencpp.dir/depend

