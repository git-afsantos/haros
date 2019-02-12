.. role:: math(raw)
   :format: html latex
..

HAROS
=====

HAROS is a framework for static analysis of ROS-based code. It has been
published in the IROS 2016 conference. If you want to cite HAROS in your
publications, please cite the `original
paper <https://doi.org/10.1109/IROS.2016.7759661>`__.

*Static analysis* consists on **extracting information** from the source
code **without executing** it (and, sometimes, even without compiling
it). The kind of properties that can be verified include simple
conformity checks, such as checking whether a variable is initialised,
to more complex properties, such as functional behavior of the program.
This allows **early detection of problems** in the software development
life cycle, which would otherwise go unnoticed into later stages or even
into production.

Needless to say, sometimes in robotics it is very hard (or very
expensive) to properly test software, not to talk about possible risks.
Hence the appeal of static analysis.

Current Status
--------------

HAROS is still being developed, as of March 2018.

Installation
------------

Here are some instructions to help you get HAROS running in your
machine. This assumes that you already have a **working installation of
ROS**. HAROS has been tested with *ROS Indigo* and *ROS Kinetic*, on
*Linux Mint* and *Linux Ubuntu*. These setups should provide you with
most of the basic dependencies of HAROS, namely **Python 2.7** and a
**Web browser** (if you want to use the visualiser).

**NOTE** This tool depends on other analysis tools. If you would rather
install these dependencies first, then ``Ctrl+F``
*:math:`dependencies`*. Otherwise, just keep reading.

Method 1: Running Without Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is an executable script in the root of this repository to help you
get started. It allows you to run haros without installing it. Make sure
that your terminal is at the root of the repository.

.. code:: bash

    cd haros
    python haros-runner.py &lt;args&gt;

You can also run it with the executable package syntax.

.. code:: bash

    python -m haros &lt;args&gt;

Method 2: Installing HAROS on Your Machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install HAROS from source or from a wheel.
Either of the following commands will install HAROS for you.

.. code:: bash

    [sudo] pip install haros
    python setup.py install

After installation, you should be able to run the command ``haros`` in
your terminal from anywhere.

Requirements
~~~~~~~~~~~~

Before you can actually run analyses with HAROS, you need to perform
some initialisation operations. These operations include downloading a
basic set of analysis plugins. Do so with:

.. code:: bash

    haros init

**Note:** if you opted for running HAROS without installing it, replace
``haros`` with your preferred method.

After initialisation, you still need to install some analysis tools that
HAROS uses behind the curtains. Install these *:math:`dependencies`*
with the following commands.

.. code:: bash

    [sudo] apt-get install cppcheck
    [sudo] apt-get install cccc
    pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor

If you want to use the model extraction features of HAROS, you must install
additional *:math:`dependencies`*.
These features are only available for C++ code as of now.

.. code:: bash

    [sudo] pip install -Iv clang==3.8
    [sudo] apt-get install libclang-3.8-dev

Optional step: set up the ``LD_LIBRARY_PATH`` environment variable to point to
the ``libclang.so`` shared library. Example:

.. code:: bash

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib

If you do not perform this step and your library is installed in a different path,
you will need to specify it in the configuration file located in
``~/.haros/index.yaml``. This file becomes available after running the
``init`` command of HAROS (details below).

HAROS is now installed and ready to use.

Usage
-----

Here is a basic example to help you get started with HAROS. Additional
examples should be added in a future update.

HAROS works with the concept of **project files**. These files are more
or less an equivalent to a project description, and they tell HAROS
which packages you want to analyse. For this basic example, you should
have the packages installed, and with available source code. If you run
``rospack find my_package`` and it returns the location of your
package's source code, you're good to go.

HAROS will only use one project file at a time, but you can create as many
as you want (*e.g.* one for each of your robots). The default project file
(empty) lies in ``~/.haros/index.yaml``, but feel free to create your
own, like so.

.. code:: bash

    touch my_index.yaml
    nano my_index.yaml

And ``my_index.yaml``'s contents:

.. code:: yaml

    %YAML 1.1
    ---
    packages:
        - package1
        - package2
        - package3

Now, you are ready to run analysis and visualisation on the given list
of packages.

.. code:: bash

    haros full -p my_index.yaml

The ``full`` command tells HAROS to run analysis and then visualisation.
If you just want to run analysis, use the ``analyse`` command instead.

The ``-p`` option lets you specify an project file of your own, instead of
using the default one.

When the analysis finishes, HAROS should start a visualisation server
and your web browser on the appropriate page. To exit, just close your
browser and press ``Enter`` on the terminal.

If you want to analyse several projects, or groups of packages, it is
recommended to create an project file for each project, and define a project
name as well. This way, HAROS will store analysis results separately.
Example:

.. code:: yaml

    %YAML 1.1
    ---
    project: my_robot
    packages:
        - package1
        - package2

Below you can find the basic commands that HAROS provides.

haros init
~~~~~~~~~~

This command runs initialisation and setup operations. This command
needs to be run before the first analysis takes place. You can also run
this command later on when you update HAROS.

haros analyse
~~~~~~~~~~~~~

This command runs analysis and model extraction on a given list of packages.

haros analyse (no options)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Runs analysis with the list of packages found within the default project
file (``~/.haros/index.yaml``). You are free to edit this file.

haros analyse -p PROJECT_FILE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Uses the given project file to run the analysis, instead of the default
one.

haros analyse -r
^^^^^^^^^^^^^^^^

Uses repository information when available. If HAROS cannot find one of
the packages you specified, it will look for it in the official ROS
distribution and download it.

If your package is not in the official distribution, you can modify your
project file to tell HAROS in which repository to look for the source
(e.g. you can specify private repositories this way). Here is an
example:

.. code:: yaml

    %YAML 1.1
    ---
    packages:
        - my_package
    repositories:
        repository_name:
            type:       git
            url:        https://github.com/git-user/repository_name.git
            version:    master
            packages:
                - my_package
                - another_package

The only supported repository type, for now, is ``git``. There is
partial support for ``hg`` and ``svn``, but these have not been fully
tested.

haros analyse -w PLUGIN [-w PLUGIN, ...]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Whitelist the given plugins. The analysis will **only** run these
plugins. This option does not work with ``-b``.

haros analyse -b PLUGIN [-b PLUGIN, ...]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Blacklist the given plugins. The analysis will **not** run these
plugins. This option does not work with ``-w``.

haros analyse -d DATA_DIR
^^^^^^^^^^^^^^^^^^^^^^^^^

Export analysis results to the given directory, instead of the default one.
This option will also install the visualisation files.
If ``DATA_DIR`` contains a previous analysis database for the current project
within its tree, it will be loaded and new results will be added to that
database.

**Note:** it is advised to use an empty/dedicated directory for this purpose.
Previous versions deleted any existing files within ``DATA_DIR``.

haros analyse -n
^^^^^^^^^^^^^^^^

Parse the source code of ROS nodes when possible, so as to extract a model from it.
This options produces a result similar to ``rqt_graph``, but without executing code.

**Note:** this option requires that you have the appropriate parsing libraries
installed (e.g. ``libclang`` for C++).

haros analyse --no-cache
^^^^^^^^^^^^^^^^^^^^^^^^

Do not use cached data. This is useful, for instance, if you want to force nodes
to be parsed again, despite any cached data.

Caches are currently invalidated by source files modified more recently than the
last analysed versions. Use this option, for instance, if you replace a file with
another with a previous modification date.

haros analyse --env
^^^^^^^^^^^^^^^^^^^

Use a full copy of your environment variables for the analysis.

haros export
~~~~~~~~~~~~

This command exports the analysis results (e.g. JSON files) to a
location of your choosing. It assumes that some analyses were run
previously.

haros export DATA_DIR
^^^^^^^^^^^^^^^^^^^^^

Exports analysis data to the given directory. This command will create
files and directories within the given directory.

haros export -v
^^^^^^^^^^^^^^^

Export visualisation files along with analysis data.

**Note:** it is advised to use an empty/dedicated directory for this purpose.
Previous versions deleted any existing files within ``DATA_DIR``.

haros export -p PROJECT_NAME
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Export a specific project's data, instead of the default one.
A special project name, ``all``, can be used to export all available projects.

haros viz
~~~~~~~~~

This command runs the visualisation only. It assumes that some analyses
were run previously.

haros viz (no options)
^^^^^^^^^^^^^^^^^^^^^^

Launches the web visualiser and the visualisation server at
``localhost:8080``.

haros viz -s HOST:PORT
^^^^^^^^^^^^^^^^^^^^^^

Launches the web visusaliser and the visualisation server at the given
host.

haros viz -d DATA_DIR
^^^^^^^^^^^^^^^^^^^^^

Serve the given directory, instead of the default one.

haros viz --headless
^^^^^^^^^^^^^^^^^^^^

Start the viz server without launching a web browser.

haros full
~~~~~~~~~~

Runs analysis and visualisation. This command accepts the same options
as ``haros analyse`` and ``haros viz``.


Settings File
-------------

HAROS uses a configuration file (located at ``~/.haros/configs.yaml``) with
some default settings. These can be changed to meet your needs, and,
in some cases, must be modified for the tool to function properly.
Future versions may expose more settings in this file.
When applicable, command-line arguments will override the settings in this file.

Here follows the current file structure.

.. code:: yaml

    %YAML 1.1
    ---
    workspace: "/path/to/catkin_ws"
    environment: null
    plugin_blacklist: []
    cpp:
        parser_lib: "/usr/lib/llvm-3.8/lib"
        std_includes: "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"
        compile_db: "/path/to/catkin_ws/build"

workspace
~~~~~~~~~

Specifies a path to your ROS catkin workspace. This setting can be omitted or
set to ``null``, in which case HAROS will attempt to find your default workspace,
using the same behaviour as the ``roscd`` tool.

environment
~~~~~~~~~~~

Specifies a mapping of variables (string keys and string values) to act as the
environment variables used during analysis. This can be used to specify variables
and values your system needs, making analyses yield the same results
independently of the machine you run HAROS on.

This value can be omitted or set to ``null``, in which case a *mostly* empty
environment will be used for analysis.

Alternatively, instead of a variable mapping, you can use the special value
``copy``, which is a shortcut to use a copy of your local environment.

plugin_blacklist
~~~~~~~~~~~~~~~~

Specifies a list of plugins to be blacklisted by default.

cpp
~~~

Under this mapping there are settings related to parsing C++ files.

parser_lib
^^^^^^^^^^

Specifies the path to the directory containing your installation of ``libclang``.
By default, this is under ``/usr/lib/llvm-3.8/lib``.

**Note:** this is a required setting by the clang compiler.

std_includes
^^^^^^^^^^^^

Specifies the path to the directory containing the C++ standard includes
provided by ``libclang``.
By default, this is under ``/usr/lib/llvm-3.8/lib/clang/3.8.0/include``.

compile_db
^^^^^^^^^^

Specifies the path to the directory containing a compilation database
(a ``compile_commands.json`` file). By default, this is under the ``build`` directory
within your catkin workspace.

This setting can be set to ``null``, in which case HAROS will try to use the
default location.

Alternatively, this setting can be set to ``false``, in which case HAROS will not
use a compilation database to parse C++ files.
