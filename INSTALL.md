# Installing HAROS

Here are some instructions to help you get HAROS running manually in your machine.
A simplified version of these instructions, with convenience scripts, can be found at [make-haros-easy](https://github.com/git-afsantos/make-haros-easy).

These instructions assume that you already have a **working installation of ROS**.
HAROS has been tested with *ROS Indigo*, *ROS Kinetic*, *ROS Melodic* and *ROS Noetic*, on *Linux Mint* and *Linux Ubuntu*.
These setups should provide you with most of the basic dependencies of HAROS, namely **Python 2.7+** (or **Python 3.6+**) and a **web browser** (if you want to use the visualiser).

For a full list of dependencies, see the [Requirements section](#requirements).

Quick links:

1. [Running From Source](#method-1-running-from-source)
2. [Installing HAROS](#method-2-installing-haros)
3. [Running With Docker](#method-3-running-with-docker)

> **NOTE** This tool assumes that the current terminal shell has a `source /opt/ros/<distro>/setup.bash` and a `source /path/to/workspace/devel/setup.bash`.
> In other words, you need a working ROS installation and a catkin workspace.

## Method 1: Running From Source

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
python haros-runner.py ARGS
```

You can also run it with the executable package syntax.

```bash
python -m haros ARGS
```

## Method 2: Installing HAROS

HAROS is available on [PyPi](https://pypi.python.org/pypi/haros).
You can install it from source or from a wheel with `pip`.

```bash
pip install haros
```

The command above will install HAROS for you.
Alternatively, download and extract its source, move to the project's root directory, and then execute the following.

```bash
python setup.py install
```

After installation, you should be able to run the command `haros` in your terminal.

> **Note:** To install via `pip` you might be asked to provide `sudo` privileges.
> If not, it is possible that the main [executable is not found](https://stackoverflow.com/a/59436732) after installing locally, due to an issue with `pip` in Debian/Ubuntu.
> This issue causes executables to be placed at `~/.local/bin`, which is not part of the default `$PATH`.
> A *recommended alternative* is to install `haros` within a `virtualenv` **virtual environment**.

## Method 3: Running With Docker

See the [*Make HAROS Easy* repository](https://github.com/git-afsantos/make-haros-easy) or the [*HAROS Tutorials* repository](https://github.com/git-afsantos/haros_tutorials).

## Requirements

Before using HAROS, you need to install some additional dependencies from the Python [requirements file](requirements.txt).  
*(Not necessary if you install HAROS via `pip`)*

```bash
pip install -r requirements.txt
```

In addition, you need to perform some initialisation operations.
Do so with:

```bash
haros init
```

> **Note:** if you opted for running HAROS from source, replace `haros` with the appropriate command.

HAROS is now installed and ready to use.
The rest of the instructions depend on the type of analysis you want to perform.

### Internal Code Quality

Analysis based on *linters*, code quality metrics, coding style, etc..

```bash
[sudo] apt-get install cppcheck
[sudo] apt-get install cccc
```

### Model Extraction

These features are only working for C++ code as of now.
Python code has limited support in the current version.

```bash
[sudo] apt-get install libclang-3.8-dev
pip install -Iv clang==3.8
```

Other versions of `libclang` are tested and supported, up to `libclang-10-dev`, at least.
Just **make sure** that the versions of `libclang` and Python's `clang` match, as in the example above.

**(Optional)** If you want to ensure that your code compiles with Clang, you might want to install `clang++` in addition to `libclang-dev` (for the same version).

**(Optional)** Set up the `LD_LIBRARY_PATH` environment variable to point to the `libclang.so` shared library.
Example:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib
```

**After running HAROS `init`:** Open the `configs.yaml` file in the HAROS home directory (default: `~/.haros/configs.yaml`) and provide the paths to the library and the include directories of your `libclang` version (uncomment any commented lines).
Example:

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

### Computation Graph Model Queries

Install `pyflwor` to enable queries to run on extracted models.

```bash
# without virtualenv
pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
# with virtualenv
pip install pyflwor-ext
```

Install also the respective [HAROS plugin](https://github.com/git-afsantos/haros-plugin-pyflwor).

### Runtime Monitor Generation

> TODO

### Property-based Test Generator

> TODO
