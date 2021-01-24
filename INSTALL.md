# Installing HAROS

Here are some instructions to help you get HAROS running in your machine.
This assumes that you already have a **working installation of ROS**.
HAROS has been tested with *ROS Indigo*, *ROS Kinetic* and *ROS Melodic*, on *Linux Mint* and *Linux Ubuntu*.
These setups should provide you with most of the basic dependencies of HAROS, namely **Python 2.7** and a **Web browser** (if you want to use the visualiser).

**NOTE** This tool depends on other analysis tools.
If you would rather install these dependencies first, then `Ctrl+F` *$dependencies$*.
Otherwise, just keep reading.

**NOTE** This tool assumes that the current terminal shell has a `source /opt/ros/<distro>/setup.bash` and a `source /path/to/workspace/devel/setup.bash`.
In other words, you need a working ROS installation and a catkin workspace.

### Method 1: Running Without Installation

Open a terminal, and move to a directory where you want to clone this repository.

```bash
mkdir ros
cd ros
git clone https://github.com/git-afsantos/haros.git
```

There is an executable script in the root of this repository to help you get started.
It allows you to run haros without installing it.
Make sure that your terminal is at the root of the repository.

```bash
cd haros
python haros-runner.py <args>
```

You can also run it with the executable package syntax.

```bash
python -m haros <args>
```

### Method 2: Installing HAROS on Your Machine

HAROS is available on [PyPi](https://pypi.python.org/pypi/haros).
You can install it from source or from a wheel.

```bash
pip install haros
```

The command above will install HAROS for you.
Alternatively, download and extract its source, move to the project's root directory, and then execute the following.

```bash
python setup.py install
```

After installation, you should be able to run the command `haros` in your terminal.

### Requirements

Before running any kind of analysis, you need to install some analysis tools and plugins that HAROS uses behind the curtains.
Install these *$dependencies$* with the following commands.

Python [requirements](requirements.txt):
*(Not necessary if you install HAROS from `pip`)*

```bash
pip install -r requirements.txt
```

Additional analysis tools:

```bash
[sudo] apt-get install cppcheck
[sudo] apt-get install cccc
```

Optionally, install `pyflwor` to enable queries to run on extracted models later on.

```bash
pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
```

or, if you have a `virtualenv`

```bash
pip install pyflwor-ext
```

If you want to use the model extraction features of HAROS, you must install
additional *$dependencies$*.
These features are only available for C++ code as of now.

```bash
[sudo] pip install -Iv clang==3.8
[sudo] apt-get install libclang-3.8-dev
```

Optional step: set up the `LD_LIBRARY_PATH` environment variable to point to
the `libclang.so` shared library.
Example:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib
```

NB: you might need to specify in `~/.haros/configs.yaml` the library file and the includes in case you are using a non-zero minor release (e.g., libclang 3.8.1)

If you do not perform this step and your library is installed in a different path, you will need to specify it in the configuration file located in `~/.haros/configs.yaml`.
This file becomes available after running HAROS for the first time.

The same applies if you want to use a version of `libclang.so` other than 3.8.
Preliminary tests suggest that 3.9, 4.0, 5.0 and 6.0 also work (as long as the versions of `libclang-X.Y-dev` and Python's `clang` package match).

**Example for version 4.0:**

```bash
[sudo] apt-get install libclang-4.0-dev
[sudo] pip install -Iv clang==4.0
```

`~/.haros/configs.yaml`:

```yaml
%YAML 1.1
---
workspace: '/home/me/ros/ws'
cpp:
    parser_lib: '/usr/lib/llvm-4.0/lib'
    std_includes: '/usr/lib/llvm-4.0/lib/clang/4.0.1/include'
    compile_db: '/home/me/ros/ws/build'
```

Finally, you need to perform some initialisation operations.
Do so with:

```bash
haros init
```

**Note:** if you opted for running HAROS without installing it, replace `haros` with your preferred method.

HAROS is now installed and ready to use.
