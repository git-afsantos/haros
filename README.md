[![DOI](https://zenodo.org/badge/27440980.svg)](https://zenodo.org/badge/latestdoi/27440980)

![HAROS](logo.png?raw=true "HAROS Logo")

HAROS is a framework for quality assurance of ROS-based code, mostly based on **static analysis** - which consists on *extracting information* from the source code *without executing it* (and, sometimes, even without compiling it).
Its main goal is the **early detection of problems** in the software development life cycle, which would otherwise go unnoticed into later stages or even into production.

**Try it yourself:** there is a demo page of the HAROS visualizer available on [GitHub](https://git-afsantos.github.io/haros).

**Tutorials:** [on GitHub](https://github.com/git-afsantos/haros_tutorials/) and [on YouTube](https://youtube.com/playlist?list=PLrXxXaugT0cwVhjhlnxY6DU0_WYPLEmgq).

**More info:** short video presentation [on YouTube](https://www.youtube.com/watch?v=s_Zmch8xjzQ).

## Current Status

HAROS is still being developed, as of August 2021.
Help improve HAROS by participating in a short [user survey](https://docs.google.com/forms/d/e/1FAIpQLSdD0nC2tb_IBjvasoWDOR_LzplTYF4cGkI5ZQqJtgDAe8ZQsQ/viewform?usp=sf_link).

## Installing

See [INSTALL](./INSTALL.md) for installation methods and instructions.

## How to Use

See [USAGE](./docs/USAGE.md) for basic commands and usage instructions.

## Bugs, Questions and Support

Check whether your question has an answer in the [FAQ](./docs/FAQ.md).

Please use the [issue tracker](https://github.com/git-afsantos/haros/issues) for issues or feature requests directly related to HAROS.

For issues related to plugins, please use the respective plugin repository.

If you run into errors, or feel that something is not working, run HAROS in debug mode, so the log files do not miss any information, e.g.,

```
haros --debug analyse ...
```

Then, you can share the log file, found by default within `~/.haros/log.txt`.

## Citing

See [CITING](./CITING.md).

## Contributing

See [CONTRIBUTING](./CONTRIBUTING.md).

## Acknowledgment

Until March 2021, this work was financed by the ERDF – European Regional Development Fund through the Operational Programme for Competitiveness and Internationalisation - COMPETE 2020 Programme and by National Funds through the Portuguese funding agency, FCT - Fundação para a Ciência e a Tecnologia within project PTDC/CCI-INF/29583/2017 (POCI-01-0145-FEDER-029583). 
