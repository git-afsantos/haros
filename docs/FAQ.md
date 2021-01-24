# Frequently Asked Questions

## Model Extraction

Getting model extraction to work requires some setup.
If you are running HAROS on a new machine, you may have to go through some errors before it starts running.

### KeyError: 'ROS_WORKSPACE'

This error tells that you do not have a catkin workspace, and that the `ROS_WORKSPACE` environment variable is not set.
This variable should be automatically set by the `setup.sh` script after you install ROS and create a workspace.

http://wiki.ros.org/ROS/EnvironmentVariables#ROS_WORKSPACE
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

If you type `echo $ROS_WORKSPACE` in the same terminal you ran HAROS, you should get back a path.

### HAROS cannot find the CMakeLists.txt file

HAROS is a tool for analysis of source code, so you always have to have package source code available.

Also, the code must compile with Clang on your system (you have to have all dependencies of your code installed, but those can be with `apt-get`).

If you analyse code that is not your own, sometimes you may have to change compiler flags, or some specific small things in the CMake files.
This is because most projects are developed with GCC instead of Clang, but it should not be difficult to compile with both.

### The Models Only Show Nodes

Make sure you are using the `-n` option on the `analyse` or `full` commands.

Generate a `compile_commands.json` within your *build* directory.

You can do it either with `catkin` or with `cmake`. Run within your workspace directory:

```
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-3.8 src
```

or

```
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-3.8
```

### Other Tips

Make sure ROS knows where the packages you want to analyse are.

> Running `rospack find YOUR_PACKAGE` should point to the directory where the *source* of the package is.

Make sure the clang and the python bindings versions match (tested with 3.8).

> Running `pip show clang` should print the same version as your libclang installation.
> If the above command does not work, run `python` and inside the interpreter run:

```
import pkg_resources
pkg_resources.get_distribution("clang").version
```

Make sure HAROS knows where to look for the files. This is optional in most cases.

In the `~/.haros/configs.yaml` file make sure that:

- `workspace` is pointing to your workspace directory, e.g. `/home/me/catkin_ws`
- `cpp:parser_lib` is pointing to the directory where you have `libclang.so` installed, e.g. `/usr/lib/llvm-3.8/lib`
- `cpp:std_includes` is pointing to the includes provided by Clang. Should be a couple directories below the above, e.g. `/usr/lib/llvm-3.8/lib/clang/3.8.0/include`
- `cpp:compile_db` is pointing  to the directory where your `compile_commands.json` is, e.g. `/home/me/catkin_ws/build`
