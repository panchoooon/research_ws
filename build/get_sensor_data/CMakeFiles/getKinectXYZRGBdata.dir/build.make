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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pancho/research_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pancho/research_ws/build

# Include any dependencies generated for this target.
include get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/depend.make

# Include the progress variables for this target.
include get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/progress.make

# Include the compile flags for this target's objects.
include get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/flags.make

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/flags.make
get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o: /home/pancho/research_ws/src/get_sensor_data/src/getKinectXYZRGBdata.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pancho/research_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o"
	cd /home/pancho/research_ws/build/get_sensor_data && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o -c /home/pancho/research_ws/src/get_sensor_data/src/getKinectXYZRGBdata.cpp

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.i"
	cd /home/pancho/research_ws/build/get_sensor_data && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pancho/research_ws/src/get_sensor_data/src/getKinectXYZRGBdata.cpp > CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.i

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.s"
	cd /home/pancho/research_ws/build/get_sensor_data && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pancho/research_ws/src/get_sensor_data/src/getKinectXYZRGBdata.cpp -o CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.s

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.requires:
.PHONY : get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.requires

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.provides: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.requires
	$(MAKE) -f get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/build.make get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.provides.build
.PHONY : get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.provides

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.provides.build: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o

# Object files for target getKinectXYZRGBdata
getKinectXYZRGBdata_OBJECTS = \
"CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o"

# External object files for target getKinectXYZRGBdata
getKinectXYZRGBdata_EXTERNAL_OBJECTS =

get_sensor_data/getKinectXYZRGBdata: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o
get_sensor_data/getKinectXYZRGBdata: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/build.make
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_viz.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_videostab.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_video.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_superres.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_stitching.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_photo.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_ocl.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_objdetect.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_nonfree.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_ml.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_legacy.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_imgproc.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_highgui.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_gpu.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_flann.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_features2d.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_core.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_contrib.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_calib3d.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_system.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_thread.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libpthread.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_common.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_kdtree.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_octree.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_search.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libqhull.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_surface.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_sample_consensus.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libOpenNI.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libOpenNI2.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkCommon.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkFiltering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkImaging.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGraphics.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGenericFiltering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkIO.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkRendering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkVolumeRendering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkHybrid.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkWidgets.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkParallel.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkInfovis.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGeovis.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkViews.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkCharts.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_io.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_filters.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_features.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_keypoints.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_registration.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_segmentation.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_recognition.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_visualization.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_people.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_outofcore.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_tracking.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_apps.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_system.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_thread.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libpthread.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libqhull.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libOpenNI.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libOpenNI2.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkCommon.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkFiltering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkImaging.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGraphics.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGenericFiltering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkIO.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkRendering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkVolumeRendering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkHybrid.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkWidgets.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkParallel.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkInfovis.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGeovis.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkViews.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkCharts.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkViews.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkInfovis.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkWidgets.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkVolumeRendering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkHybrid.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkParallel.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkRendering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkImaging.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkGraphics.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkIO.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkFiltering.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtkCommon.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libvtksys.so.5.8.0
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_nonfree.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_ocl.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_gpu.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_photo.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_objdetect.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_legacy.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_video.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_ml.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_calib3d.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_features2d.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_highgui.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_imgproc.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_flann.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/local/lib/libopencv_core.so.2.4.11
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_common.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_kdtree.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_octree.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_search.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_surface.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_sample_consensus.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_io.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_filters.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_features.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_keypoints.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_registration.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_segmentation.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_recognition.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_visualization.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_people.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_outofcore.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_tracking.so
get_sensor_data/getKinectXYZRGBdata: /usr/lib/libpcl_apps.so
get_sensor_data/getKinectXYZRGBdata: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable getKinectXYZRGBdata"
	cd /home/pancho/research_ws/build/get_sensor_data && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getKinectXYZRGBdata.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/build: get_sensor_data/getKinectXYZRGBdata
.PHONY : get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/build

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/requires: get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/src/getKinectXYZRGBdata.cpp.o.requires
.PHONY : get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/requires

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/clean:
	cd /home/pancho/research_ws/build/get_sensor_data && $(CMAKE_COMMAND) -P CMakeFiles/getKinectXYZRGBdata.dir/cmake_clean.cmake
.PHONY : get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/clean

get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/depend:
	cd /home/pancho/research_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pancho/research_ws/src /home/pancho/research_ws/src/get_sensor_data /home/pancho/research_ws/build /home/pancho/research_ws/build/get_sensor_data /home/pancho/research_ws/build/get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : get_sensor_data/CMakeFiles/getKinectXYZRGBdata.dir/depend

