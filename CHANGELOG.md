# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2017-10-25
### Added
- Change log.
- `-t` option for `analyse` and `full` commands, to specify a target export dir.
- `-v` option for `export` command, to also install viz files.
- `-d` option for `viz` command, to allow the server to serve a custom dir.

### Changed
- Plugins now use a random tmp dir for their operation. This allows multiple instances of haros at the same time.