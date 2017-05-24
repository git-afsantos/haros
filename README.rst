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

HAROS is being developed, as of May 2017. It is undergoing some
substantial changes, compared to its original version, so I ask for your
comprehension regarding bugs and a lack of documentation. Do not
hesitate, however, to provide your feedback.

Do note, also, that HAROS is not my job, and, as such, it is evolving at
a relatively slow pace.

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

You can install HAROS from source or from an egg.
Either of the following commands will install HAROS for you.

.. code:: bash

    [sudo] pip install haros
    python setup.py install

After installation, you should be able to run the command ``haros`` in
your terminal from anywhere.

Prerequisites
~~~~~~~~~~~~~

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

    sudo apt-get install python-pip
    pip install --upgrade pip
    sudo pip install radon
    sudo pip install lizard
    sudo apt-get install cppcheck

HAROS is now installed and ready to use.

Usage
-----

Here is a basic example to help you get started with HAROS. Additional
examples should be added in a future update.

HAROS works with the concept of **index files**. These files tell HAROS
which packages you want to analyse. For this basic example, you should
have the packages installed, and with available source code. If you run
``rospack find my_package`` and it returns the location of your
package's source code, you're good to go.

HAROS will only use one index file at a time, but you can create as many
as you want (*e.g.* one for each of your robots). The default index file
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

The ``-p`` option lets you specify an index file of your own, instead of
using the default one.

When the analysis finishes, HAROS should start a visualisation server
and your web browser on the appropriate page. To exit, just close your
browser and press ``Enter`` on the terminal.

Below you can find the basic commands that HAROS provides.

haros init
~~~~~~~~~~

This command runs initialisation and setup operations. This command
needs to be run before the first analysis takes place. You can also run
this command later on when you update HAROS.

haros analyse
~~~~~~~~~~~~~

This command runs analysis on a given list of packages.

haros analyse (no options)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Runs analysis with the list of packages found within the default index
file (``~/.haros/index.yaml``). You are free to edit this file.

haros analyse -p <index file>
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Uses the given index file to run the analysis, instead of the default
one.

haros analyse -r
^^^^^^^^^^^^^^^^

Uses repository information when available. If HAROS cannot find one of
the packages you specified, it will look for it in the official ROS
distribution and download it.

If your package is not in the official distribution, you can modify your
index file to tell HAROS in which repository to look for the source
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

haros analyse -w <plugin name> [-w <another plugin>, ...]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Whitelist the given plugins. The analysis will **only** run these
plugins. This option does not work with ``-b``.

haros analyse -b <plugin name> [-b <another plugin>, ...]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Blacklist the given plugins. The analysis will **not** run these
plugins. This option does not work with ``-w``.

haros export
~~~~~~~~~~~~

This command exports the analysis results (e.g. JSON files) to a
location of your choosing. It assumes that some analyses were run
previously.

haros export <directory>
^^^^^^^^^^^^^^^^^^^^^^^^

Exports analysis data to the given directory. This command will create
files and directories within the given directory.

haros viz
~~~~~~~~~

This command runs the visualisation only. It assumes that some analyses
were run previously.

haros viz (no options)
^^^^^^^^^^^^^^^^^^^^^^

Launches the web visualiser and the visualisation server at
``localhost:8080``.

haros viz -s <host:port>
^^^^^^^^^^^^^^^^^^^^^^^^

Launches the web visusaliser and the visualisation server at the given
host.

haros full
~~~~~~~~~~

Runs analysis and visualisation. This command accepts the same options
as ``haros analyse`` and ``haros viz``.