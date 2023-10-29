# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.27

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\Alex\Documents\BehaviorTree.CPP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\Alex\Documents\BehaviorTree.CPP\build

# Include any dependencies generated for this target.
include sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/compiler_depend.make

# Include the progress variables for this target.
include sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/progress.make

# Include the compile flags for this target's objects.
include sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/flags.make

sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/flags.make
sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/includes_CXX.rsp
sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj: C:/Users/Alex/Documents/BehaviorTree.CPP/sample_nodes/crossdoor_nodes.cpp
sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\Users\Alex\Documents\BehaviorTree.CPP\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj"
	cd /d C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes && C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj -MF CMakeFiles\crossdoor_nodes_dyn.dir\crossdoor_nodes.cpp.obj.d -o CMakeFiles\crossdoor_nodes_dyn.dir\crossdoor_nodes.cpp.obj -c C:\Users\Alex\Documents\BehaviorTree.CPP\sample_nodes\crossdoor_nodes.cpp

sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.i"
	cd /d C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes && C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\Alex\Documents\BehaviorTree.CPP\sample_nodes\crossdoor_nodes.cpp > CMakeFiles\crossdoor_nodes_dyn.dir\crossdoor_nodes.cpp.i

sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.s"
	cd /d C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes && C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\Alex\Documents\BehaviorTree.CPP\sample_nodes\crossdoor_nodes.cpp -o CMakeFiles\crossdoor_nodes_dyn.dir\crossdoor_nodes.cpp.s

# Object files for target crossdoor_nodes_dyn
crossdoor_nodes_dyn_OBJECTS = \
"CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj"

# External object files for target crossdoor_nodes_dyn
crossdoor_nodes_dyn_EXTERNAL_OBJECTS =

sample_nodes/libcrossdoor_nodes_dyn.dll: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/crossdoor_nodes.cpp.obj
sample_nodes/libcrossdoor_nodes_dyn.dll: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/build.make
sample_nodes/libcrossdoor_nodes_dyn.dll: libbehaviortree_cpp.dll.a
sample_nodes/libcrossdoor_nodes_dyn.dll: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/linkLibs.rsp
sample_nodes/libcrossdoor_nodes_dyn.dll: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/objects1.rsp
sample_nodes/libcrossdoor_nodes_dyn.dll: sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=C:\Users\Alex\Documents\BehaviorTree.CPP\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libcrossdoor_nodes_dyn.dll"
	cd /d C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\crossdoor_nodes_dyn.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/build: sample_nodes/libcrossdoor_nodes_dyn.dll
.PHONY : sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/build

sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/clean:
	cd /d C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes && $(CMAKE_COMMAND) -P CMakeFiles\crossdoor_nodes_dyn.dir\cmake_clean.cmake
.PHONY : sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/clean

sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\Alex\Documents\BehaviorTree.CPP C:\Users\Alex\Documents\BehaviorTree.CPP\sample_nodes C:\Users\Alex\Documents\BehaviorTree.CPP\build C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes C:\Users\Alex\Documents\BehaviorTree.CPP\build\sample_nodes\CMakeFiles\crossdoor_nodes_dyn.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : sample_nodes/CMakeFiles/crossdoor_nodes_dyn.dir/depend

