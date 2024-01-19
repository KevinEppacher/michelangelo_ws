# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/kevin/michelangelo_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/michelangelo_ws/build

# Include any dependencies generated for this target.
include _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/depend.make

# Include the progress variables for this target.
include _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/flags.make

_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/zblat1.f.o: _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/flags.make
_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/zblat1.f.o: _deps/eigen-src/blas/testing/zblat1.f
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/michelangelo_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building Fortran object _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/zblat1.f.o"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing && /usr/bin/f95 $(Fortran_DEFINES) $(Fortran_INCLUDES) $(Fortran_FLAGS) -c /home/kevin/michelangelo_ws/build/_deps/eigen-src/blas/testing/zblat1.f -o CMakeFiles/zblat1.dir/zblat1.f.o

_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/zblat1.f.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing Fortran source to CMakeFiles/zblat1.dir/zblat1.f.i"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing && /usr/bin/f95 $(Fortran_DEFINES) $(Fortran_INCLUDES) $(Fortran_FLAGS) -E /home/kevin/michelangelo_ws/build/_deps/eigen-src/blas/testing/zblat1.f > CMakeFiles/zblat1.dir/zblat1.f.i

_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/zblat1.f.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling Fortran source to assembly CMakeFiles/zblat1.dir/zblat1.f.s"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing && /usr/bin/f95 $(Fortran_DEFINES) $(Fortran_INCLUDES) $(Fortran_FLAGS) -S /home/kevin/michelangelo_ws/build/_deps/eigen-src/blas/testing/zblat1.f -o CMakeFiles/zblat1.dir/zblat1.f.s

# Object files for target zblat1
zblat1_OBJECTS = \
"CMakeFiles/zblat1.dir/zblat1.f.o"

# External object files for target zblat1
zblat1_EXTERNAL_OBJECTS =

_deps/eigen-build/blas/testing/zblat1: _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/zblat1.f.o
_deps/eigen-build/blas/testing/zblat1: _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/build.make
_deps/eigen-build/blas/testing/zblat1: _deps/eigen-build/blas/libeigen_blas.so
_deps/eigen-build/blas/testing/zblat1: _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/michelangelo_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking Fortran executable zblat1"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/zblat1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/build: _deps/eigen-build/blas/testing/zblat1

.PHONY : _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/build

_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/clean:
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing && $(CMAKE_COMMAND) -P CMakeFiles/zblat1.dir/cmake_clean.cmake
.PHONY : _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/clean

_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/depend:
	cd /home/kevin/michelangelo_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/michelangelo_ws /home/kevin/michelangelo_ws/build/_deps/eigen-src/blas/testing /home/kevin/michelangelo_ws/build /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing /home/kevin/michelangelo_ws/build/_deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/eigen-build/blas/testing/CMakeFiles/zblat1.dir/depend

