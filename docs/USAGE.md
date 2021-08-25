# How to Use HAROS

Here is a basic example to help you get started with HAROS.
Additional examples should be added in a future update.

HAROS works with the concept of **project files**.
These files are more or less an equivalent to a project description, and they tell HAROS which packages and applications you want to analyse.
For this basic example, you should have the packages installed, and with available source code.
If you run `rospack find my_package` and it returns the location of your package's source code, you're good to go.

HAROS will only use one project file at a time, but you can create as many as you want (e.g., one for each of your robots).

```bash
touch my_robot.yaml
nano my_robot.yaml
```

And `my_robot.yaml`'s contents:

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
haros full -p my_robot.yaml
```

The `full` command tells HAROS to run analysis and then visualisation.
If you just want to run analysis, use the `analyse` command instead.

The `-p` option lets you specify a project file of your own.

When the analysis finishes, HAROS should start a visualisation server and your web browser on the appropriate page.
To exit, just close your browser and press `Enter` on the terminal.

The visualiser presents you with different views of the analysis results.
By default, the first thing you see will be the dashboard.
The dashboard gives you an overview of the analysed items and the number of issues found.

![Dashboard screenshot](/docs/screenshots/dashboard_1.png?raw=true "Dashboard Statistics")

It also shows you the evolution of some metrics over time.

![Dashboard screenshot](/docs/screenshots/dashboard_2.png?raw=true "Dashboard Tracking")

Using the other tabs at the top bar, you can navigate to a package graph view, showing the analysed packages and their dependencies.
Each package is coloured according to a scale at the bottom, to give you an idea of how many issues the analysis found for that package.

![Packages screenshot](/docs/screenshots/packages.png?raw=true "Package Graph")

You can view the list of issues, filtering by package or by a number of tags that each rule provides.

![Issues screenshot](/docs/screenshots/issues.png?raw=true "Issues List")

If you want to analyse several projects, or groups of packages, it is recommended to create a project file for each project, and define a project name as well.
This way, HAROS will store analysis results separately.
Example:

```yaml
%YAML 1.1
---
project: my_robot
packages:
    - package1
    - package2
```

*An empty list of packages results in the analysis of all packages found under the current working directory.*

Below you can find the basic commands and options that HAROS provides.

### haros --home HOME_DIR

This top-level option sets HOME_DIR as the default HAROS home directory for the current run.
By default, HAROS uses `~/.haros` as the HOME_DIR.

### haros init

This command runs setup operations.
This command will **create directories** and **overwrite some files** (if there are some already with the same name).

### haros analyse

This command runs analysis and model extraction on a given list of packages.

#### haros analyse (no options)

Runs analysis with the list of packages found within the default project file
(`~/.haros/index.yaml`).
You are free to edit this file.

#### haros analyse -p PROJECT_FILE

Uses the given project file to run the analysis, instead of the default one.

#### haros analyse -r

Uses repository information when available.
If HAROS cannot find one of the packages you specified, it will look for it in the official ROS distribution and download it.

If your package is not in the official distribution, you can modify your project file to tell HAROS in which repository to look for the source (e.g., you can
specify private repositories this way).
Here is an example:

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

The only supported repository type, for now, is `git`.
There is partial support for `hg` and `svn`, but these have not been fully tested.

#### haros analyse -w PLUGIN [PLUGIN2 PLUGIN3 ...]

Whitelist the given plugins.
The analysis will **only** run these plugins.
This option does not work with `-b`.

#### haros analyse -b PLUGIN [PLUGIN2 PLUGIN3 ...]

Blacklist the given plugins.
The analysis will **not** run these plugins.
This option does not work with `-w`.

#### haros analyse -d DATA_DIR

Export analysis results to the given directory, instead of the default one.
This option will also install the visualisation files.
If `DATA_DIR` contains a previous analysis database for the current project within its tree, it will be loaded and new results will be added to that database.

> **Note:** it is advised to use an empty/dedicated directory for this purpose.
> Previous versions deleted any existing files within `DATA_DIR`.

#### haros analyse -n

Parse the source code of ROS nodes when possible, so as to extract a model from it.
This options produces a result similar to `rqt_graph`, but without executing code.

> **Note:** this option requires that you have the appropriate parsing libraries installed (e.g., `libclang` for C++).

For issues with model extraction be sure to check [FAQ](./FAQ.md).

#### haros analyse --no-cache

Do not use cached data.
This is useful, for instance, if you want to force nodes to be parsed again, despite any cached data.

Caches are currently invalidated by source files modified more recently than the last analysed versions.
Use this option, for instance, if you replace a file with another with a previous modification date.

#### haros analyse --env

Use a full copy of your environment variables for the analysis.

#### haros analyse --minimal-output

Only export those files necessary for viewing the HTML report.

### haros export

This command exports the analysis results (e.g. JSON files) to a location of your choosing.
It assumes that some analyses were run previously.

#### haros export DATA_DIR

Exports analysis data to the given directory.
This command will create files and directories within the given directory.

#### haros export -v

Export visualisation files along with analysis data.

> **Note:** it is advised to use an empty/dedicated directory for this purpose.
> Previous versions deleted any existing files within `DATA_DIR`.

#### haros export -p PROJECT_NAME

Export a specific project's data, instead of the default one.
A special project name, `all`, can be used to export all available projects.

#### haros export --minimal-output

Only export those files necessary for viewing the HTML report.

### haros viz

This command runs the visualisation only.
It assumes that some analyses were run previously.

#### haros viz (no options)

Launches the web visualiser and the visualisation server at `localhost:8080`.

#### haros viz -s HOST:PORT

Launches the web visusaliser and the visualisation server at the given host.

#### haros viz -d DATA_DIR

Serve the given directory, instead of the default one.

#### haros viz --headless

Start the viz server without launching a web browser.


### haros full

Runs analysis and visualisation.
This command accepts the same options as `haros analyse` and `haros viz`.


## Settings File

HAROS uses a configuration file (located at `~/.haros/configs.yaml`) with some default settings.
These can be changed to meet your needs, and, in some cases, must be modified for the tool to function properly.
Future versions may expose more settings in this file.
When applicable, command-line arguments will override the settings in this file.

Here follows the current file structure.

```yaml
%YAML 1.1
---
workspace: "/path/to/catkin_ws"
environment: null
plugin_blacklist: []
cpp:
    parser_lib: "/usr/lib/llvm-3.8/lib"
    std_includes: "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"
    compile_db: "/path/to/catkin_ws/build"
analysis:
    ignore:
        tags: []
        rules: []
        metrics: []
```

### workspace

Specifies a path to your ROS catkin workspace.
This setting can be omitted or set to `null`, in which case HAROS will attempt to find your default workspace, using the same behaviour as the `roscd` tool.

### environment

Specifies a mapping of variables (string keys and string values) to act as the environment variables used during analysis.
This can be used to specify variables and values your system needs, making analyses yield the same results independently of the machine you run HAROS on.

This value can be omitted or set to `null`, in which case a *mostly* empty environment will be used for analysis.

Alternatively, instead of a variable mapping, you can use the special value `copy`, which is a shortcut to use a copy of your local environment.

### plugin_blacklist

Specifies a list of plugins to be blacklisted by default.

### cpp

Under this mapping there are settings related to parsing C++ files.

#### parser_lib

Specifies the path to the directory containing your installation of `libclang`.
By default, this is under `/usr/lib/llvm-3.8/lib`.

> **Note:** this is a required setting by the clang compiler.

#### std_includes

Specifies the path to the directory containing the C++ standard includes provided by `libclang`.
By default, this is under `/usr/lib/llvm-3.8/lib/clang/3.8.0/include`.

#### compile_db

Specifies the path to the directory containing a compilation database (a `compile_commands.json` file).
By default, this is under the `build` directory within your catkin workspace.

This setting can be set to `null`, in which case HAROS will try to use the default location.

Alternatively, this setting can be set to `false`, in which case HAROS will not use a compilation database to parse C++ files.


## Defining Custom Applications

HAROS allows you to define your own ROS applications for analysis (called *Configurations*).
These are defined in project files.
Example:

```yaml
%YAML 1.1
---
packages:
    - my_package
configurations:
    my_app:
        launch:
            - my_package/launch/base.launch
            - my_package/launch/controller.launch
```

A Configuration is a mapping from a name (of your choosing) to a list of launch files that make the application (in the order they should be launched).
HAROS will analyse the given files to try to extract participant nodes and parameters.

If node parsing is enabled (`-n`), the source code of participant nodes will be scanned for topics and services used, thus completing the *ROS Computation Graph*.

It is possible that the extracted model is missing some entities, or is unable to resolve all ROS names.
In this case, you can provide extraction hints in the project file as well.
Hints describe which topics each node uses, as well as the message type.
Hints should be set with the names the node would use prior to any remappings.
Here follows an example.

```yaml
%YAML 1.1
---
packages:
    - my_package
configurations:
    my_app:
        launch:
            - my_package/launch/base.launch
            - my_package/launch/controller.launch
        hints:
            nodes:
                /base_node:
                    publishers:
                        - topic: "/controller_cmd"
                          msg_type: "std_msgs/Float64"
```

This feature may require some additional setup.
Make sure you look into the provided [tips](./FAQ.md).
