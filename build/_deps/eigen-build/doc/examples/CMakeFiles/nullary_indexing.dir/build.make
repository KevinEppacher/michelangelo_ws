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
include _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/depend.make

# Include the progress variables for this target.
include _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/flags.make

_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.o: _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/flags.make
_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.o: _deps/eigen-src/doc/examples/nullary_indexing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/michelangelo_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.o"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.o -c /home/kevin/michelangelo_ws/build/_deps/eigen-src/doc/examples/nullary_indexing.cpp

_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.i"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/michelangelo_ws/build/_deps/eigen-src/doc/examples/nullary_indexing.cpp > CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.i

_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.s"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/michelangelo_ws/build/_deps/eigen-src/doc/examples/nullary_indexing.cpp -o CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.s

# Object files for target nullary_indexing
nullary_indexing_OBJECTS = \
"CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.o"

# External object files for target nullary_indexing
nullary_indexing_EXTERNAL_OBJECTS =

_deps/eigen-build/doc/examples/nullary_indexing: _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/nullary_indexing.cpp.o
_deps/eigen-build/doc/examples/nullary_indexing: _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/build.make
_deps/eigen-build/doc/examples/nullary_indexing: _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/michelangelo_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable nullary_indexing"
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nullary_indexing.dir/link.txt --verbose=$(VERBOSE)
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples && ./nullary_indexing >/home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples/nullary_indexing.out

# Rule to build all files generated by this target.
_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/build: _deps/eigen-build/doc/examples/nullary_indexing

.PHONY : _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/build

_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/clean:
	cd /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples && $(CMAKE_COMMAND) -P CMakeFiles/nullary_indexing.dir/cmake_clean.cmake
.PHONY : _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/clean

_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/depend:
	cd /home/kevin/michelangelo_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/michelangelo_ws /home/kevin/michelangelo_ws/build/_deps/eigen-src/doc/examples /home/kevin/michelangelo_ws/build /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples /home/kevin/michelangelo_ws/build/_deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/eigen-build/doc/examples/CMakeFiles/nullary_indexing.dir/depend

