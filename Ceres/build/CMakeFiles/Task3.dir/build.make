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
CMAKE_SOURCE_DIR = /home/sentinel/Algorithms-Plus/Ceres

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sentinel/Algorithms-Plus/Ceres/build

# Include any dependencies generated for this target.
include CMakeFiles/Task3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Task3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Task3.dir/flags.make

CMakeFiles/Task3.dir/ceres_test.cpp.o: CMakeFiles/Task3.dir/flags.make
CMakeFiles/Task3.dir/ceres_test.cpp.o: ../ceres_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sentinel/Algorithms-Plus/Ceres/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Task3.dir/ceres_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Task3.dir/ceres_test.cpp.o -c /home/sentinel/Algorithms-Plus/Ceres/ceres_test.cpp

CMakeFiles/Task3.dir/ceres_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Task3.dir/ceres_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sentinel/Algorithms-Plus/Ceres/ceres_test.cpp > CMakeFiles/Task3.dir/ceres_test.cpp.i

CMakeFiles/Task3.dir/ceres_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Task3.dir/ceres_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sentinel/Algorithms-Plus/Ceres/ceres_test.cpp -o CMakeFiles/Task3.dir/ceres_test.cpp.s

CMakeFiles/Task3.dir/ceres_test.cpp.o.requires:

.PHONY : CMakeFiles/Task3.dir/ceres_test.cpp.o.requires

CMakeFiles/Task3.dir/ceres_test.cpp.o.provides: CMakeFiles/Task3.dir/ceres_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/Task3.dir/build.make CMakeFiles/Task3.dir/ceres_test.cpp.o.provides.build
.PHONY : CMakeFiles/Task3.dir/ceres_test.cpp.o.provides

CMakeFiles/Task3.dir/ceres_test.cpp.o.provides.build: CMakeFiles/Task3.dir/ceres_test.cpp.o


# Object files for target Task3
Task3_OBJECTS = \
"CMakeFiles/Task3.dir/ceres_test.cpp.o"

# External object files for target Task3
Task3_EXTERNAL_OBJECTS =

Task3: CMakeFiles/Task3.dir/ceres_test.cpp.o
Task3: CMakeFiles/Task3.dir/build.make
Task3: /usr/local/lib/libceres.a
Task3: /usr/lib/x86_64-linux-gnu/libglog.so
Task3: /usr/lib/x86_64-linux-gnu/libgflags.so
Task3: /usr/lib/x86_64-linux-gnu/libspqr.so
Task3: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
Task3: /usr/lib/x86_64-linux-gnu/libtbb.so
Task3: /usr/lib/x86_64-linux-gnu/libcholmod.so
Task3: /usr/lib/x86_64-linux-gnu/libccolamd.so
Task3: /usr/lib/x86_64-linux-gnu/libcamd.so
Task3: /usr/lib/x86_64-linux-gnu/libcolamd.so
Task3: /usr/lib/x86_64-linux-gnu/libamd.so
Task3: /usr/lib/liblapack.so
Task3: /usr/lib/libblas.so
Task3: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
Task3: /usr/lib/x86_64-linux-gnu/librt.so
Task3: /usr/lib/x86_64-linux-gnu/libcxsparse.so
Task3: /usr/lib/x86_64-linux-gnu/libspqr.so
Task3: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
Task3: /usr/lib/x86_64-linux-gnu/libtbb.so
Task3: /usr/lib/x86_64-linux-gnu/libcholmod.so
Task3: /usr/lib/x86_64-linux-gnu/libccolamd.so
Task3: /usr/lib/x86_64-linux-gnu/libcamd.so
Task3: /usr/lib/x86_64-linux-gnu/libcolamd.so
Task3: /usr/lib/x86_64-linux-gnu/libamd.so
Task3: /usr/lib/liblapack.so
Task3: /usr/lib/libblas.so
Task3: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
Task3: /usr/lib/x86_64-linux-gnu/librt.so
Task3: /usr/lib/x86_64-linux-gnu/libcxsparse.so
Task3: CMakeFiles/Task3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sentinel/Algorithms-Plus/Ceres/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Task3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Task3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Task3.dir/build: Task3

.PHONY : CMakeFiles/Task3.dir/build

CMakeFiles/Task3.dir/requires: CMakeFiles/Task3.dir/ceres_test.cpp.o.requires

.PHONY : CMakeFiles/Task3.dir/requires

CMakeFiles/Task3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Task3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Task3.dir/clean

CMakeFiles/Task3.dir/depend:
	cd /home/sentinel/Algorithms-Plus/Ceres/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sentinel/Algorithms-Plus/Ceres /home/sentinel/Algorithms-Plus/Ceres /home/sentinel/Algorithms-Plus/Ceres/build /home/sentinel/Algorithms-Plus/Ceres/build /home/sentinel/Algorithms-Plus/Ceres/build/CMakeFiles/Task3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Task3.dir/depend

