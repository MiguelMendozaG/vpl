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
include apps/CMakeFiles/test_freeflyer.dir/depend.make

# Include the progress variables for this target.
include apps/CMakeFiles/test_freeflyer.dir/progress.make

# Include the compile flags for this target's objects.
include apps/CMakeFiles/test_freeflyer.dir/flags.make

apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o: apps/CMakeFiles/test_freeflyer.dir/flags.make
apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o: ../apps/test_freeflyer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/miguelmg/repositorios/vpl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o"
	cd /home/miguelmg/repositorios/vpl/build/apps && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o -c /home/miguelmg/repositorios/vpl/apps/test_freeflyer.cpp

apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.i"
	cd /home/miguelmg/repositorios/vpl/build/apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/miguelmg/repositorios/vpl/apps/test_freeflyer.cpp > CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.i

apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.s"
	cd /home/miguelmg/repositorios/vpl/build/apps && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/miguelmg/repositorios/vpl/apps/test_freeflyer.cpp -o CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.s

apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.requires:

.PHONY : apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.requires

apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.provides: apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.requires
	$(MAKE) -f apps/CMakeFiles/test_freeflyer.dir/build.make apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.provides.build
.PHONY : apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.provides

apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.provides.build: apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o


# Object files for target test_freeflyer
test_freeflyer_OBJECTS = \
"CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o"

# External object files for target test_freeflyer
test_freeflyer_EXTERNAL_OBJECTS =

apps/test_freeflyer: apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o
apps/test_freeflyer: apps/CMakeFiles/test_freeflyer.dir/build.make
apps/test_freeflyer: ../lib/libViewPlanning.so
apps/test_freeflyer: ../lib/libRangeSimulator.so
apps/test_freeflyer: ../lib/libPartialModel.so
apps/test_freeflyer: apps/CMakeFiles/test_freeflyer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/miguelmg/repositorios/vpl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_freeflyer"
	cd /home/miguelmg/repositorios/vpl/build/apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_freeflyer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/CMakeFiles/test_freeflyer.dir/build: apps/test_freeflyer

.PHONY : apps/CMakeFiles/test_freeflyer.dir/build

apps/CMakeFiles/test_freeflyer.dir/requires: apps/CMakeFiles/test_freeflyer.dir/test_freeflyer.cpp.o.requires

.PHONY : apps/CMakeFiles/test_freeflyer.dir/requires

apps/CMakeFiles/test_freeflyer.dir/clean:
	cd /home/miguelmg/repositorios/vpl/build/apps && $(CMAKE_COMMAND) -P CMakeFiles/test_freeflyer.dir/cmake_clean.cmake
.PHONY : apps/CMakeFiles/test_freeflyer.dir/clean

apps/CMakeFiles/test_freeflyer.dir/depend:
	cd /home/miguelmg/repositorios/vpl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/miguelmg/repositorios/vpl /home/miguelmg/repositorios/vpl/apps /home/miguelmg/repositorios/vpl/build /home/miguelmg/repositorios/vpl/build/apps /home/miguelmg/repositorios/vpl/build/apps/CMakeFiles/test_freeflyer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/CMakeFiles/test_freeflyer.dir/depend

