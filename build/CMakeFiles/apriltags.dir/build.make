# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/nadim/Desktop/aprilGrej/apriltags

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nadim/Desktop/aprilGrej/apriltags/build

# Include any dependencies generated for this target.
include CMakeFiles/apriltags.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/apriltags.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/apriltags.dir/flags.make

CMakeFiles/apriltags.dir/apriltags.cc.o: CMakeFiles/apriltags.dir/flags.make
CMakeFiles/apriltags.dir/apriltags.cc.o: ../apriltags.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nadim/Desktop/aprilGrej/apriltags/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/apriltags.dir/apriltags.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/apriltags.dir/apriltags.cc.o -c /home/nadim/Desktop/aprilGrej/apriltags/apriltags.cc

CMakeFiles/apriltags.dir/apriltags.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/apriltags.dir/apriltags.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nadim/Desktop/aprilGrej/apriltags/apriltags.cc > CMakeFiles/apriltags.dir/apriltags.cc.i

CMakeFiles/apriltags.dir/apriltags.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/apriltags.dir/apriltags.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nadim/Desktop/aprilGrej/apriltags/apriltags.cc -o CMakeFiles/apriltags.dir/apriltags.cc.s

CMakeFiles/apriltags.dir/apriltags.cc.o.requires:

.PHONY : CMakeFiles/apriltags.dir/apriltags.cc.o.requires

CMakeFiles/apriltags.dir/apriltags.cc.o.provides: CMakeFiles/apriltags.dir/apriltags.cc.o.requires
	$(MAKE) -f CMakeFiles/apriltags.dir/build.make CMakeFiles/apriltags.dir/apriltags.cc.o.provides.build
.PHONY : CMakeFiles/apriltags.dir/apriltags.cc.o.provides

CMakeFiles/apriltags.dir/apriltags.cc.o.provides.build: CMakeFiles/apriltags.dir/apriltags.cc.o


# Object files for target apriltags
apriltags_OBJECTS = \
"CMakeFiles/apriltags.dir/apriltags.cc.o"

# External object files for target apriltags
apriltags_EXTERNAL_OBJECTS =

apriltags: CMakeFiles/apriltags.dir/apriltags.cc.o
apriltags: CMakeFiles/apriltags.dir/build.make
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
apriltags: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
apriltags: CMakeFiles/apriltags.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nadim/Desktop/aprilGrej/apriltags/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable apriltags"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/apriltags.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/apriltags.dir/build: apriltags

.PHONY : CMakeFiles/apriltags.dir/build

CMakeFiles/apriltags.dir/requires: CMakeFiles/apriltags.dir/apriltags.cc.o.requires

.PHONY : CMakeFiles/apriltags.dir/requires

CMakeFiles/apriltags.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/apriltags.dir/cmake_clean.cmake
.PHONY : CMakeFiles/apriltags.dir/clean

CMakeFiles/apriltags.dir/depend:
	cd /home/nadim/Desktop/aprilGrej/apriltags/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nadim/Desktop/aprilGrej/apriltags /home/nadim/Desktop/aprilGrej/apriltags /home/nadim/Desktop/aprilGrej/apriltags/build /home/nadim/Desktop/aprilGrej/apriltags/build /home/nadim/Desktop/aprilGrej/apriltags/build/CMakeFiles/apriltags.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/apriltags.dir/depend

