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
include CMakeFiles/Task2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Task2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Task2.dir/flags.make

CMakeFiles/Task2.dir/ceres_learn.cc.o: CMakeFiles/Task2.dir/flags.make
CMakeFiles/Task2.dir/ceres_learn.cc.o: ../ceres_learn.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sentinel/Algorithms-Plus/Ceres/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Task2.dir/ceres_learn.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Task2.dir/ceres_learn.cc.o -c /home/sentinel/Algorithms-Plus/Ceres/ceres_learn.cc

CMakeFiles/Task2.dir/ceres_learn.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Task2.dir/ceres_learn.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sentinel/Algorithms-Plus/Ceres/ceres_learn.cc > CMakeFiles/Task2.dir/ceres_learn.cc.i

CMakeFiles/Task2.dir/ceres_learn.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Task2.dir/ceres_learn.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sentinel/Algorithms-Plus/Ceres/ceres_learn.cc -o CMakeFiles/Task2.dir/ceres_learn.cc.s

CMakeFiles/Task2.dir/ceres_learn.cc.o.requires:

.PHONY : CMakeFiles/Task2.dir/ceres_learn.cc.o.requires

CMakeFiles/Task2.dir/ceres_learn.cc.o.provides: CMakeFiles/Task2.dir/ceres_learn.cc.o.requires
	$(MAKE) -f CMakeFiles/Task2.dir/build.make CMakeFiles/Task2.dir/ceres_learn.cc.o.provides.build
.PHONY : CMakeFiles/Task2.dir/ceres_learn.cc.o.provides

CMakeFiles/Task2.dir/ceres_learn.cc.o.provides.build: CMakeFiles/Task2.dir/ceres_learn.cc.o


# Object files for target Task2
Task2_OBJECTS = \
"CMakeFiles/Task2.dir/ceres_learn.cc.o"

# External object files for target Task2
Task2_EXTERNAL_OBJECTS =

Task2: CMakeFiles/Task2.dir/ceres_learn.cc.o
Task2: CMakeFiles/Task2.dir/build.make
Task2: /usr/local/lib/libceres.a
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
Task2: /usr/lib/x86_64-linux-gnu/libglog.so
Task2: /usr/lib/x86_64-linux-gnu/libgflags.so
Task2: /usr/lib/x86_64-linux-gnu/libspqr.so
Task2: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
Task2: /usr/lib/x86_64-linux-gnu/libtbb.so
Task2: /usr/lib/x86_64-linux-gnu/libcholmod.so
Task2: /usr/lib/x86_64-linux-gnu/libccolamd.so
Task2: /usr/lib/x86_64-linux-gnu/libcamd.so
Task2: /usr/lib/x86_64-linux-gnu/libcolamd.so
Task2: /usr/lib/x86_64-linux-gnu/libamd.so
Task2: /usr/lib/liblapack.so
Task2: /usr/lib/libblas.so
Task2: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
Task2: /usr/lib/x86_64-linux-gnu/librt.so
Task2: /usr/lib/x86_64-linux-gnu/libcxsparse.so
Task2: /usr/lib/x86_64-linux-gnu/libspqr.so
Task2: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
Task2: /usr/lib/x86_64-linux-gnu/libtbb.so
Task2: /usr/lib/x86_64-linux-gnu/libcholmod.so
Task2: /usr/lib/x86_64-linux-gnu/libccolamd.so
Task2: /usr/lib/x86_64-linux-gnu/libcamd.so
Task2: /usr/lib/x86_64-linux-gnu/libcolamd.so
Task2: /usr/lib/x86_64-linux-gnu/libamd.so
Task2: /usr/lib/liblapack.so
Task2: /usr/lib/libblas.so
Task2: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
Task2: /usr/lib/x86_64-linux-gnu/librt.so
Task2: /usr/lib/x86_64-linux-gnu/libcxsparse.so
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
Task2: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
Task2: CMakeFiles/Task2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sentinel/Algorithms-Plus/Ceres/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Task2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Task2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Task2.dir/build: Task2

.PHONY : CMakeFiles/Task2.dir/build

CMakeFiles/Task2.dir/requires: CMakeFiles/Task2.dir/ceres_learn.cc.o.requires

.PHONY : CMakeFiles/Task2.dir/requires

CMakeFiles/Task2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Task2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Task2.dir/clean

CMakeFiles/Task2.dir/depend:
	cd /home/sentinel/Algorithms-Plus/Ceres/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sentinel/Algorithms-Plus/Ceres /home/sentinel/Algorithms-Plus/Ceres /home/sentinel/Algorithms-Plus/Ceres/build /home/sentinel/Algorithms-Plus/Ceres/build /home/sentinel/Algorithms-Plus/Ceres/build/CMakeFiles/Task2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Task2.dir/depend

