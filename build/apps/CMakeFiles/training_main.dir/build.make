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
CMAKE_SOURCE_DIR = /home/miguelmg/repositorios/vpl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/miguelmg/repositorios/vpl/build

# Include any dependencies generated for this target.
include apps/CMakeFiles/training_main.dir/depend.make

# Include the progress variables for this target.
include apps/CMakeFiles/training_main.dir/progress.make

# Include the compile flags for this target's objects.
include apps/CMakeFiles/training_main.dir/flags.make

apps/CMakeFiles/training_main.dir/training_main.cpp.o: apps/CMakeFiles/training_main.dir/flags.make
apps/CMakeFiles/training_main.dir/training_main.cpp.o: ../apps/training_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/miguelmg/repositorios/vpl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/CMakeFiles/training_main.dir/training_main.cpp.o"
	cd /home/miguelmg/repositorios/vpl/build/apps && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/training_main.dir/training_main.cpp.o -c /home/miguelmg/repositorios/vpl/apps/training_main.cpp

apps/CMakeFiles/training_main.dir/training_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/training_main.dir/training_main.cpp.i"
	cd /home/miguelmg/repositorios/vpl/build/apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/miguelmg/repositorios/vpl/apps/training_main.cpp > CMakeFiles/training_main.dir/training_main.cpp.i

apps/CMakeFiles/training_main.dir/training_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/training_main.dir/training_main.cpp.s"
	cd /home/miguelmg/repositorios/vpl/build/apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/miguelmg/repositorios/vpl/apps/training_main.cpp -o CMakeFiles/training_main.dir/training_main.cpp.s

apps/CMakeFiles/training_main.dir/training_main.cpp.o.requires:

.PHONY : apps/CMakeFiles/training_main.dir/training_main.cpp.o.requires

apps/CMakeFiles/training_main.dir/training_main.cpp.o.provides: apps/CMakeFiles/training_main.dir/training_main.cpp.o.requires
	$(MAKE) -f apps/CMakeFiles/training_main.dir/build.make apps/CMakeFiles/training_main.dir/training_main.cpp.o.provides.build
.PHONY : apps/CMakeFiles/training_main.dir/training_main.cpp.o.provides

apps/CMakeFiles/training_main.dir/training_main.cpp.o.provides.build: apps/CMakeFiles/training_main.dir/training_main.cpp.o


# Object files for target training_main
training_main_OBJECTS = \
"CMakeFiles/training_main.dir/training_main.cpp.o"

# External object files for target training_main
training_main_EXTERNAL_OBJECTS =

apps/training_main: apps/CMakeFiles/training_main.dir/training_main.cpp.o
apps/training_main: apps/CMakeFiles/training_main.dir/build.make
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_system.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libpthread.so
apps/training_main: /usr/local/lib/libpcl_common.so
apps/training_main: /usr/local/lib/libpcl_octree.so
apps/training_main: /usr/lib/libOpenNI.so
apps/training_main: /usr/lib/libOpenNI2.so
apps/training_main: /usr/local/lib/libpcl_io.so
apps/training_main: /usr/local/lib/libpcl_stereo.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
apps/training_main: /usr/local/lib/libpcl_kdtree.so
apps/training_main: /usr/local/lib/libpcl_search.so
apps/training_main: /usr/local/lib/libpcl_sample_consensus.so
apps/training_main: /usr/local/lib/libpcl_filters.so
apps/training_main: /usr/local/lib/libpcl_features.so
apps/training_main: /usr/local/lib/libpcl_registration.so
apps/training_main: /usr/local/lib/libpcl_visualization.so
apps/training_main: /usr/local/lib/libpcl_outofcore.so
apps/training_main: /usr/local/lib/libpcl_keypoints.so
apps/training_main: /usr/local/lib/libpcl_ml.so
apps/training_main: /usr/local/lib/libpcl_segmentation.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libqhull.so
apps/training_main: /usr/local/lib/libpcl_surface.so
apps/training_main: /usr/local/lib/libpcl_tracking.so
apps/training_main: /usr/local/lib/libpcl_recognition.so
apps/training_main: /usr/local/lib/libpcl_people.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_system.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libpthread.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libqhull.so
apps/training_main: /usr/lib/libOpenNI.so
apps/training_main: /usr/lib/libOpenNI2.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
apps/training_main: /usr/local/lib/libvtkRenderingContextOpenGL2-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersProgrammable-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOEnSight-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOVideo-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOPLY-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersSelection-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersVerdict-7.1.so.1
apps/training_main: /usr/local/lib/libvtkverdict-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingImage-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOMovie-7.1.so.1
apps/training_main: /usr/local/lib/libvtkoggtheora-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOParallel-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOGeometry-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIONetCDF-7.1.so.1
apps/training_main: /usr/local/lib/libvtkjsoncpp-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOTecplotTable-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersParallelImaging-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOLSDyna-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersSMP-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOSQL-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingLOD-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOMINC-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOExport-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingStatistics-7.1.so.1
apps/training_main: /usr/local/lib/libvtkGeovisCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkproj4-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOParallelXML-7.1.so.1
apps/training_main: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersHyperTree-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersFlowPaths-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersTexture-7.1.so.1
apps/training_main: /usr/local/lib/libvtkViewsContext2D-7.1.so.1
apps/training_main: /usr/local/lib/libvtkViewsInfovis-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersPoints-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingMorphological-7.1.so.1
apps/training_main: /usr/local/lib/libvtkInteractionImage-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOExodus-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOAMR-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOInfovis-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingStencil-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOImport-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersGeneric-7.1.so.1
apps/training_main: ../lib/libPartialModel.so
apps/training_main: /usr/local/lib/libpcl_common.so
apps/training_main: /usr/local/lib/libpcl_octree.so
apps/training_main: /usr/local/lib/libpcl_io.so
apps/training_main: /usr/local/lib/libpcl_stereo.so
apps/training_main: /usr/local/lib/libpcl_kdtree.so
apps/training_main: /usr/local/lib/libpcl_search.so
apps/training_main: /usr/local/lib/libpcl_sample_consensus.so
apps/training_main: /usr/local/lib/libpcl_filters.so
apps/training_main: /usr/local/lib/libpcl_features.so
apps/training_main: /usr/local/lib/libpcl_registration.so
apps/training_main: /usr/local/lib/libpcl_visualization.so
apps/training_main: /usr/local/lib/libpcl_outofcore.so
apps/training_main: /usr/local/lib/libpcl_keypoints.so
apps/training_main: /usr/local/lib/libpcl_ml.so
apps/training_main: /usr/local/lib/libpcl_segmentation.so
apps/training_main: /usr/local/lib/libpcl_surface.so
apps/training_main: /usr/local/lib/libpcl_tracking.so
apps/training_main: /usr/local/lib/libpcl_recognition.so
apps/training_main: /usr/local/lib/libpcl_people.so
apps/training_main: /usr/local/lib/libvtkFiltersParallel-7.1.so.1
apps/training_main: /usr/local/lib/libvtksqlite-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-7.1.so.1
apps/training_main: /usr/local/lib/libvtkgl2ps-7.1.so.1
apps/training_main: /usr/local/lib/libvtkDomainsChemistry-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersImaging-7.1.so.1
apps/training_main: /usr/local/lib/libvtkInfovisLayout-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingLabel-7.1.so.1
apps/training_main: /usr/local/lib/libvtkChartsCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingContext2D-7.1.so.1
apps/training_main: /usr/local/lib/libvtkViewsCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkInteractionWidgets-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingGeneral-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersModeling-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingHybrid-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingAnnotation-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingFreeType-7.1.so.1
apps/training_main: /usr/local/lib/libvtkfreetype-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingColor-7.1.so.1
apps/training_main: /usr/local/lib/libvtkInteractionStyle-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersHybrid-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingSources-7.1.so.1
apps/training_main: /usr/local/lib/libvtkexoIIc-7.1.so.1
apps/training_main: /usr/local/lib/libvtkNetCDF_cxx-7.1.so.1
apps/training_main: /usr/local/lib/libvtkNetCDF-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingOpenGL2-7.1.so.1
apps/training_main: /usr/lib/x86_64-linux-gnu/libSM.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libICE.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libX11.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libXext.so
apps/training_main: /usr/lib/x86_64-linux-gnu/libXt.so
apps/training_main: /usr/local/lib/libvtkglew-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingMath-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingVolume-7.1.so.1
apps/training_main: /usr/local/lib/libvtkhdf5_hl-7.1.so.1
apps/training_main: /usr/local/lib/libvtkhdf5-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersAMR-7.1.so.1
apps/training_main: /usr/local/lib/libvtkParallelCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkInfovisCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersExtraction-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersStatistics-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingFourier-7.1.so.1
apps/training_main: /usr/local/lib/libvtkalglib-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOLegacy-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOXML-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOXMLParser-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkexpat-7.1.so.1
apps/training_main: /usr/local/lib/libvtklibxml2-7.1.so.1
apps/training_main: /usr/local/lib/libvtkImagingCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkRenderingCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonColor-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersGeometry-7.1.so.1
apps/training_main: /usr/local/lib/libvtkIOImage-7.1.so.1
apps/training_main: /usr/local/lib/libvtkDICOMParser-7.1.so.1
apps/training_main: /usr/local/lib/libvtkmetaio-7.1.so.1
apps/training_main: /usr/local/lib/libvtkpng-7.1.so.1
apps/training_main: /usr/local/lib/libvtktiff-7.1.so.1
apps/training_main: /usr/local/lib/libvtkzlib-7.1.so.1
apps/training_main: /usr/local/lib/libvtkjpeg-7.1.so.1
apps/training_main: /usr/lib/x86_64-linux-gnu/libm.so
apps/training_main: /usr/local/lib/libvtkFiltersSources-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersGeneral-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonComputationalGeometry-7.1.so.1
apps/training_main: /usr/local/lib/libvtkFiltersCore-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonExecutionModel-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonDataModel-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonMisc-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonSystem-7.1.so.1
apps/training_main: /usr/local/lib/libvtksys-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonTransforms-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonMath-7.1.so.1
apps/training_main: /usr/local/lib/libvtkCommonCore-7.1.so.1
apps/training_main: apps/CMakeFiles/training_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/miguelmg/repositorios/vpl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable training_main"
	cd /home/miguelmg/repositorios/vpl/build/apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/training_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/CMakeFiles/training_main.dir/build: apps/training_main

.PHONY : apps/CMakeFiles/training_main.dir/build

apps/CMakeFiles/training_main.dir/requires: apps/CMakeFiles/training_main.dir/training_main.cpp.o.requires

.PHONY : apps/CMakeFiles/training_main.dir/requires

apps/CMakeFiles/training_main.dir/clean:
	cd /home/miguelmg/repositorios/vpl/build/apps && $(CMAKE_COMMAND) -P CMakeFiles/training_main.dir/cmake_clean.cmake
.PHONY : apps/CMakeFiles/training_main.dir/clean

apps/CMakeFiles/training_main.dir/depend:
	cd /home/miguelmg/repositorios/vpl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/miguelmg/repositorios/vpl /home/miguelmg/repositorios/vpl/apps /home/miguelmg/repositorios/vpl/build /home/miguelmg/repositorios/vpl/build/apps /home/miguelmg/repositorios/vpl/build/apps/CMakeFiles/training_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/CMakeFiles/training_main.dir/depend

