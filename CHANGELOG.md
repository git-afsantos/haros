# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- A new issue reporting function for plugins, focusing on runtime (`Configuration`-related) issues.

## [3.5.5] - 2019-06-25
### Changed
- Fixed a bug that caused a crash when `analyse` was called on a new HAROS home directory, without calling `init` before, and using the default YAML project file.

## [3.5.4] - 2019-06-24
### Changed
- Fixed a bug that caused the wrong default project file to be loaded when using the `--home` option.

## [3.5.3] - 2019-06-24
### Added
- A new section in project files, `nodes`, where users can provide extraction hints for nodes outside of the scope of a particular configuration. This avoids duplication of hints across similar configurations.

### Changed
- Fixed a minor bug during error handling that would cause the wrong error to be reported.

## [3.5.2] - 2019-06-17
### Changed
- Fixed a bug where user-defined queries were being ignored.

## [3.5.1] - 2019-06-17
### Changed
- Fixed an issue where packages had to be included for analysis (with source available) for nodes belonging to such packages, appearing in a launch file, to appear in extracted models.

## [3.5.0] - 2019-06-15
### Added
- Added the `--home` option to the base `haros` command. This option receives a directory as argument, to use in place of the default `$HOME/.haros` data directory. It is useful, for instance, to have multiple settings profiles.

### Changed
- All commands (`haros analyse`, `haros export`, etc.) perform the necessary setup operations formerly performed with `haros init`.
- `haros init` is no longer required before the first run of HAROS, and it is now used just to reset the `~/.haros` directory (or any other given with `--home`) to a fresh state.

## [3.4.1] - 2019-06-15
### Changed
- Improved extraction accuracy for a specific case in C++ where primitives are called within a constructor and the `NodeHandle` is a default-initialized class member.

## [3.4.0] - 2019-06-14
### Changed
- The model extraction process should now be able to resolve `typedef`s.

### Added
- Support to extract `image_transport::SubscriberFilter` calls.
- Support to extract `message_filters::Subscriber` calls.
- Support to extract `image_transport::Subscriber` calls.
- Support to extract `image_transport::Publisher` calls.

## [3.3.3] - 2019-06-12
### Changed
- Dependency on `rosgraph` is now optional.
- Fixed a bug that prevented unresolved namespaces to match with valid extraction hints (e.g. `/?/topic` would not match with a `topic` hint).

## [3.3.0] - 2019-06-03
### Changed
- Plugins are no longer cloned and installed by HAROS. Instead they are distributed and installed as a `pip` requirement.
- The plugin discovery mechanism had to change, in order to comply with the point above. Plugins are now discovered by naming convention (package names starting with `haros_plugin_`).

## [3.2.2] - 2019-05-14
### Changed
- Fixed bug in CMake parser where `include_directories` expected at least one argument.

## [3.2.1] - 2019-04-12
### Changed
- Fixed the auto-generated `~/.haros/configs.yaml` to include the new sections to ignore rules and tags.

## [3.2.0] - 2019-04-04
### Added
- `~/.haros/configs.yaml` file can be used to ignore specific rules or tags.
- C++ files can be annotated with `// haros:ignore-line` and `// haros:ignore-next-line` to exclude specific lines from analysis.
- Python files can be annotated with `# haros:ignore-line` and `# haros:ignore-next-line` to exclude specific lines from analysis.

## [3.1.2] - 2019-03-20
### Added
- `requirements.txt` file.

### Changed
- Replaced `yaml.load` with `yaml.safe_load`.
- Added a monkey patch for `pyflwor`, so that it now works from a `pip install`.

## [3.1.1] - 2019-03-04
### Changed
- Made the dependency on `pyflwor` optional (#38).
- Updated README to reflect this (#40).

## [3.1.0] - 2019-01-30
### Added
- HAROS can now detect workspaces built with `catkin_make_isolated` and `catkin build`.

## [3.0.7] - 2018-12-17
### Changed
- Fixed some bugs when parsing CMake files.

## [3.0.4] - 2018-03-08
### Changed
- Fixed a bug crashing first-time runs.

## [3.0.2] - 2018-03-08
### Changed
- Fixed a bug preventing the `full` command from working.

## [3.0.0] - 2018-03-08
### Added
- Parsers for CMake and Launch files.
- Extractor for nodes within packages (which files compile a node).
- Configuration builder: define your ROS applications (Configurations) to be automatically extracted (C++ only with `clang` parser).
- Query engine to operate on extracted models.
- Graph visualiser for extracted models (in HAROSviz).
- Plugin entry point to analyse extracted Configurations.
- User settings file in default `.haros` directory.

### Changed
- HAROS uses a new metamodel to represent its data.
- The source code indexing and analysis process was changed, although most (if not all) of these changes should not be noticeable by end users.
- Command line options for `analyse` and `full` have changed.
- Changed project files (package index files), so that they now allow the definition of custom Configurations and custom queries.
- Exported JSON formats have changed as a result of the new metamodel.

## [2.1.2] - 2017-11-27
### Changed
- Fixed an issue where `full` and `viz` commands were crashing if the `-d` option was not present.

## [2.1.1] - 2017-11-05
### Changed
- `--headless` option for `viz` does not require any user input.

## [2.1.0] - 2017-11-04
### Added
- `--headless` option for `viz` and `full` to allow the server to run without opening a web browser.

## [2.0.0] - 2017-11-04
### Added
- A reserved project name `all`. When running `export -p all` all available projects will be exported.
- Dependency on PyLint.

### Changed
- `-C` base option has been renamed to `-c` or `--cwd`.
- `--repositories` long option for `analyse` and `full` has been renamed to `--use-repos`.
- `-t`/`--target-dir` option for `analyse` and `full` has been renamed to `-d`/`--data-dir`.
- When `-d` is given, HAROS will try to load an existing analysis database from that directory. If there is one, new analysis reports will be added and stored on that database.
- `analyse -d DIR` and `export -v DIR` will no longer delete the contents of `DIR`.
- Changed the directory structure for exported results. For `analyse -d DIR` and `export -v DIR`, viz files go into `DIR`, with an additional `projects` directory for databases. For `export DIR` all results go directly into `DIR/PROJECT`.
- The main `haros` module has been rewritten to be more modular and maintainable. It is completely **backwards incompatible**.

### Removed
- `-a` option that allowed a specific analysis database to be imported. See `-d` for more.

## [1.0.0] - 2017-10-31
### Added
- Concept of project, more or less equivalent to an index file.
- `-a` option for `analyse` and `full` commands, to import an existing analysis database.

### Changed
- `-d` option from viz is now `-t` for consistency with `analyse` and `full` commands.
- Directory structure for viz and Haros data.
- Viz dashboard now allows selection of displayed project.

## [0.3.0] - 2017-10-25
### Added
- Change log.
- `-t` option for `analyse` and `full` commands, to specify a target export dir.
- `-v` option for `export` command, to also install viz files.
- `-d` option for `viz` command, to allow the server to serve a custom dir.

### Changed
- Plugins now use a random tmp dir for their operation. This allows multiple instances of haros at the same time.