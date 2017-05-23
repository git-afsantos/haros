HAROS
=====

HAROS is a framework for static analysis of ROS-based code.
It has been published in the IROS 2016 conference.
If you want to cite HAROS in your publications, please cite the [original paper](https://doi.org/10.1109/IROS.2016.7759661).

*Static analysis* consists on **extracting information** from the source code
**without executing** it (and, sometimes, even without compiling it).
The kind of properties that can be verified include simple conformity checks,
such as checking whether a variable is initialised,
to more complex properties, such as functional behavior of
the program. This allows **early detection of problems** in
the software development life cycle, which would otherwise
go unnoticed into later stages or even into production.

Needless to say, sometimes in robotics it is very hard (or very expensive)
to properly test software, not to talk about possible risks.
Hence the appeal of static analysis.

Current Status
--------------

HAROS is being developed, as of early 2017. It is undergoing some
substantial changes, compared to its original version, so I ask for your
comprehension regarding bugs and a lack of documentation.
Do not hesitate, however, to provide your feedback.

Do note, also, that HAROS is not my job, and, as such, it is
evolving at a relatively slow pace.

Installation
------------

Here are some instructions to help you get HAROS running in your machine.
This assumes that you already have a **working installation of ROS**.
HAROS has been tested with *ROS Indigo* and *ROS Kinetic*, on
*Linux Mint* and *Linux Ubuntu*. These setups should provide you with
most of the basic dependencies of HAROS, namely **Python 2.7**
and a **Web browser**.

**NOTE** This tool depends on other analysis tools. If you would rather
install these dependencies first, then `Ctrl+F` *$dependencies$*.
Otherwise, just keep reading.

Open a terminal, and move to a directory where you want to clone this
repository.

```bash
mkdir ros
cd ros
git clone https://github.com/git-afsantos/haros.git
```

There is an executable script in the root of this repository to help you get started.
Copy the script to the directory where you want to be running HAROS from, and
edit it to make sure it points to the right source path.

```bash
cp haros/haros run_haros
nano run_haros
```

In this example, the path should look like this:

    ~/ros/haros/analyser/main.py

Before you can actually run HAROS, you need to perform some initialisation operations.
These operations include downloading a basic set of analysis plugins. Do so with:

```bash
./run_haros init
```

After initialisation, you still need to install some analysis tools that HAROS
uses behind the curtains. Install these *$dependencies$* with the following commands.

```bash
sudo apt-get install python-pip
pip install --upgrade pip
sudo pip install radon
sudo pip install lizard
sudo apt-get install cppcheck
```

HAROS is now installed and ready to use.

Usage
-----

Here is a basic example to help you get started with HAROS.
Additional examples should be added in a future update.

HAROS works with the concept of **index files**.
These files tell HAROS which packages you want to analyse.
For this basic example, you should have the packages installed,
and with available source code.
If you run `rospack find my_package` and it returns the location
of your package's source code, you're good to go.

HAROS will only use one index file at a time, but you can create as
many as you want (*e.g.* one for each of your robots). The default
index file (empty) lies in `~/.haros/index.yaml`, but feel free to
create your own, like so.

```bash
touch my_index.yaml
nano my_index.yaml
```

And `my_index.yaml`'s contents:

```yaml
%YAML 1.1
---
packages:
    - package1
    - package2
    - package3
```

Now, you are ready to run analysis and visualisation on the given list of packages.

```bash
./run_haros full -p my_index.yaml
```

The `full` command tells HAROS to run analysis and then visualisation. If you just
want to run analysis, use the `analyse` command instead.

The `-p` option lets you specify an index file of your own, instead of using the default one.

When the analysis finishes, HAROS should start a visualisation server and your web browser
on the appropriate page. To exit, just close your browser and press `Enter` on the terminal.

Below you can find the basic commands that HAROS provides.

###haros init

This command runs initialisation and setup operations. This command needs to be run before the first analysis takes place. You can also run this command later on when you update HAROS.

###haros analyse

This command runs analysis on a given list of packages.

####haros analyse (no options)

Runs analysis with the list of packages found within the default index file
(`~/.haros/index.yaml`). You are free to edit this file.

####haros analyse -p &lt;index file&gt;

Uses the given index file to run the analysis, instead of the default one.

####haros analyse -r

Uses repository information when available. If HAROS cannot find one of the
packages you specified, it will look for it in the official ROS distribution and
download it.

If your package is not in the official distribution, you can modify your index
file to tell HAROS in which repository to look for the source (e.g. you can
specify private repositories this way). Here is an example:

```yaml
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
```

The only supported repository type, for now, is `git`. There is partial support
for `hg` and `svn`, but these have not been fully tested.

####haros analyse -w &lt;plugin name&gt; [-w &lt;another plugin&gt;, ...]

Whitelist the given plugins. The analysis will **only** run these plugins.
This option does not work with `-b`.

####haros analyse -b &lt;plugin name&gt; [-b &lt;another plugin&gt;, ...]

Blacklist the given plugins. The analysis will **not** run these plugins.
This option does not work with `-w`.


###haros export

This command exports the analysis results (e.g. JSON files) to a location of your
choosing. It assumes that some analyses were run previously.

####haros export &lt;directory&gt;

Exports analysis data to the given directory. This command will create files and
directories within the given directory.


###haros viz

This command runs the visualisation only. It assumes that some analyses were run
previously.

####haros viz (no options)

Launches the web visualiser and the visualisation server at `localhost:8080`.

####haros viz -s &lt;host:port&gt;

Launches the web visusaliser and the visualisation server at the given host.


###haros full

Runs analysis and visualisation. This command accepts the same options as
`haros analyse` and `haros viz`.
