# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]

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