named in honor of our cad prgram , fusion 360

# Instructions

1. Clone repo inside your workspaces src directory (Ex. phnx_ws/src)
2. `rosdep install --from-paths . --ignore-src -r -y` to install deps
3. `colcon build` to make sure the repo builds before you mess with it
4. Replace the following in both file names and code exactly and consistently. 
   1. mult_path_fusere with the package name. Use snake case. Ex. `mult_path_fuser`
   2. TODO_PACKAGE_NAME: Replace with the node name. Use Pascal case. Ex. `multPathFuser360`
5. `colcon build` again. If it builds, you are done
6. Rename outer folder
7. Review the optional dependencies, and remove what you do not need

# Dependencies
Some common extra dependencies are included. Review them and remove what you don't need.
These are marked with TODO_EXTRA.

# Features

- Unit tests
- ROS-Industrial github CI (will test units and lints)
- C++ formatting via clangformat
- A selection of sane lints
- A single node setup in a multithreaded executor

# File structure

```
.
├── include
│   └── mult_path_fuser
│       └── mult_path_fuser_node.hpp
├── package.xml
├── README.md
├── src
│   ├── mult_path_fuser.cpp
│   └── mult_path_fuser_node.cpp
└── tests
    └── unit.cpp
```

mult_path_fuser_node: Source files for the ROS2 node object itself, and only itself

mult_path_fuser.cpp: Source for the main function of the node, and only the main function

tests/unit.cpp: Example file for unit tests. This is linked to the node and ros, so both can be used
