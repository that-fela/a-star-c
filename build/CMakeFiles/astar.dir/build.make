# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leo/dev/a-star-for-robots

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/dev/a-star-for-robots/build

# Include any dependencies generated for this target.
include CMakeFiles/astar.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/astar.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/astar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/astar.dir/flags.make

CMakeFiles/astar.dir/main.c.o: CMakeFiles/astar.dir/flags.make
CMakeFiles/astar.dir/main.c.o: ../main.c
CMakeFiles/astar.dir/main.c.o: CMakeFiles/astar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/dev/a-star-for-robots/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/astar.dir/main.c.o"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/astar.dir/main.c.o -MF CMakeFiles/astar.dir/main.c.o.d -o CMakeFiles/astar.dir/main.c.o -c /home/leo/dev/a-star-for-robots/main.c

CMakeFiles/astar.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/astar.dir/main.c.i"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leo/dev/a-star-for-robots/main.c > CMakeFiles/astar.dir/main.c.i

CMakeFiles/astar.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/astar.dir/main.c.s"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leo/dev/a-star-for-robots/main.c -o CMakeFiles/astar.dir/main.c.s

CMakeFiles/astar.dir/src/astar-h1.c.o: CMakeFiles/astar.dir/flags.make
CMakeFiles/astar.dir/src/astar-h1.c.o: ../src/astar-h1.c
CMakeFiles/astar.dir/src/astar-h1.c.o: CMakeFiles/astar.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/dev/a-star-for-robots/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/astar.dir/src/astar-h1.c.o"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/astar.dir/src/astar-h1.c.o -MF CMakeFiles/astar.dir/src/astar-h1.c.o.d -o CMakeFiles/astar.dir/src/astar-h1.c.o -c /home/leo/dev/a-star-for-robots/src/astar-h1.c

CMakeFiles/astar.dir/src/astar-h1.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/astar.dir/src/astar-h1.c.i"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leo/dev/a-star-for-robots/src/astar-h1.c > CMakeFiles/astar.dir/src/astar-h1.c.i

CMakeFiles/astar.dir/src/astar-h1.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/astar.dir/src/astar-h1.c.s"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leo/dev/a-star-for-robots/src/astar-h1.c -o CMakeFiles/astar.dir/src/astar-h1.c.s

# Object files for target astar
astar_OBJECTS = \
"CMakeFiles/astar.dir/main.c.o" \
"CMakeFiles/astar.dir/src/astar-h1.c.o"

# External object files for target astar
astar_EXTERNAL_OBJECTS =

astar: CMakeFiles/astar.dir/main.c.o
astar: CMakeFiles/astar.dir/src/astar-h1.c.o
astar: CMakeFiles/astar.dir/build.make
astar: CMakeFiles/astar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/dev/a-star-for-robots/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable astar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/astar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/astar.dir/build: astar
.PHONY : CMakeFiles/astar.dir/build

CMakeFiles/astar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/astar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/astar.dir/clean

CMakeFiles/astar.dir/depend:
	cd /home/leo/dev/a-star-for-robots/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/dev/a-star-for-robots /home/leo/dev/a-star-for-robots /home/leo/dev/a-star-for-robots/build /home/leo/dev/a-star-for-robots/build /home/leo/dev/a-star-for-robots/build/CMakeFiles/astar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/astar.dir/depend

