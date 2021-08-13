
#Copyright (c) 2016 Andre Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.


# HAROS directory (data folder) structure:

# + ~/.haros
# |-- index.yaml
# |-- configs.yaml
# |-- parse_cache.json
# |-- log.txt
# |-+ repositories
#   |-+ ...
# |-+ viz
#   |-+ ...
#   |-+ data
#     |-+ <project>
#       |-- packages.json
#       |-- rules.json
#       |-- summary.json
#       |-+ compliance
#         |-- ...
#       |-+ metrics
#         |-- ...
#       |-+ models
#         |-- ...
#     |-- ...
#   |-- index.html
# |-+ export
#   |-- metrics.csv
# |-+ projects
#   |-+ <project>
#     |-- analysis.db
#     |-- haros.db

# init creates the default data dir
# viz is copied to init dir

# analysis grabs previous db and index from data dir or provided dir

# export receives a dir where it will generate the export files
# export uses db from analysis step or loads it from data dir

# viz uses data from the data dir or provided dir

# Options:
#   --debug sets the logging level to debug
#   -c changes the CWD before running
#   --home sets the HAROS directory (default "~/.haros")
#   --config sets the location of the YAML file containing the configuration
#            (default "~/.haros/configs.yaml")
#   --junit-xml-output causes HAROS to output JUnit XML format report files.
#   --minimal-output causes HAROS to output a size-optimized report,
#             omitting files not required for the dashboard display.
#   haros init
#       initialises the data directory
#   haros analyse [args]
#       runs update, analysis and export
#       -r  also register and analyse repositories
#       -p  package/project filter
#       -w  whitelist plugins
#       -b  blacklist plugins
#       -d  use given directory to load and export
#   haros export [args]
#       runs export only
#       -v export viz files too
#       -p project name to export
#   haros viz [args]
#       runs visualiser only
#       -s host
#       -d serve given directory
#   haros full [args]
#       full run (analyse + viz)


###############################################################################
#   Imports
###############################################################################

from __future__ import print_function
from __future__ import unicode_literals

from builtins import str
from past.builtins import basestring
from builtins import object
from argparse import ArgumentParser
import json
import logging
import os
import tempfile
from timeit import default_timer as timer

from shutil import copyfile, rmtree
from pkg_resources import Requirement, resource_filename

from .data import HarosDatabase, HarosSettings
from .extractor import ProjectExtractor, HardcodedNodeParser
from .config_builder import ConfigurationBuilder
from .plugin_manager import Plugin
from .analysis_manager import AnalysisManager
from .export_manager import JsonExporter, JUnitExporter
from . import visualiser as viz


###############################################################################
#   HAROS Launcher (Main Application)
###############################################################################

class HarosLauncher(object):
    """This class contains the necessary methods to launch HAROS.
        It is responsible for initialising directories and data
        structures, as well as parsing program arguments.
    """

    HAROS_DIR = os.path.join(os.path.expanduser("~"), ".haros")
    DEFAULT_INDEX = os.path.join(HAROS_DIR, "index.yaml")
    LOG_PATH = os.path.join(HAROS_DIR, "log.txt")
    VIZ_DIR = os.path.join(HAROS_DIR, "viz")

    DIR_STRUCTURE = {
        "index.yaml": "%YAML 1.1\n---\npackages: []\n",
        "configs.yaml": (
            "%YAML 1.1\n---\n"
            "# workspace: '/path/to/ws'\n"
            "# environment: null\n"
            "# plugin_blacklist: []\n"
            "# cpp:\n"
            # "#    parser: clang,\n"
            "#    parser_lib: '/usr/lib/llvm-3.8/lib'\n"
            "#    std_includes: '/usr/lib/llvm-3.8/lib/clang/3.8.0/include'\n"
            "#    compile_db: '/path/to/ws/build'\n"
            "# analysis:\n"
            "#    ignore:\n"
            "#        tags: []\n"
            "#        rules: []\n"
            "#        metrics: []\n"
        ),
        "parse_cache.json": "{}",
        "repositories": {},
        "export": {},
        "projects": {
            "default": {}
        },
        "pyflwor": {}
        # viz is generated on viz.install
    }

    def __init__(self, run_from_source=False):
        self.log = logging.getLogger()
        self.run_from_source = run_from_source
        self.initialised = False
        self.haros_dir = self.HAROS_DIR
        self.index_path = self.DEFAULT_INDEX
        self.log_path = self.LOG_PATH
        self.viz_dir = self.VIZ_DIR

    def launch(self, argv=None):
        args = self.parse_arguments(argv)
        command = getattr(args, "command", None)
        if command is None:
            return True
        self.minimal_output = getattr(args, "minimal_output", False)
        self._set_directories(args)
        if args.debug:
            logging.basicConfig(filename=self.log_path, filemode="w",
                                level=logging.DEBUG)
        else:
            logging.basicConfig(level=logging.WARNING)
        self.log.debug("Running from home directory: %s", self.haros_dir)
        original_path = os.getcwd()
        try:
            if args.cwd:
                os.chdir(args.cwd)
            self.log.info("Executing selected command.")
            return command(args)
        except KeyError as err:
            if str(err) == "ROS_WORKSPACE":
                print("[HAROS] You must have a workspace set up.")
                print("  Make sure to source its `devel/setup.bash`.")
            self.log.error(str(err))
            return False
        except RuntimeError as err:
            self.log.error(str(err))
            return False
        finally:
            os.chdir(original_path)
            if command != self.command_news:
                print("[HAROS] Check news updates with:")
                print("  $ haros news")

    def command_init(self, args):
        if not self.initialised:
            self.initialised = self._init_haros_dir(overwrite=True)
            return self.initialised
        return True

    def command_full(self, args):
        return self.command_analyse(args) and self.command_viz(args)

    def command_analyse(self, args):
        if not self.initialised:
            self.initialised = self._init_haros_dir(overwrite=False)
        if args.data_dir and not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        project_file = args.project_file or self.index_path
        if not os.path.isfile(project_file):
            raise ValueError("Not a file: " + project_file)
        if args.ws:
            if not os.path.isdir(args.ws):
                raise ValueError("Not a directory: " + args.ws)
        analyse = HarosAnalyseRunner(self.haros_dir, self.config_path,
            project_file, args.data_dir, args.whitelist, args.blacklist,
            log=self.log, run_from_source=self.run_from_source, ws=args.ws,
            use_repos=args.use_repos, parse_nodes=args.parse_nodes,
            copy_env=args.env, use_cache=(not args.no_cache),
            overwrite_cache=(not args.no_write_cache),
            junit_xml_output=args.junit_xml_output,
            minimal_output=args.minimal_output, no_hardcoded=args.no_hardcoded)
        return analyse.run()

    def command_export(self, args):
        if not self.initialised:
            self.initialised = self._init_haros_dir(overwrite=False)
        if not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        export = HarosExportRunner(self.haros_dir, self.config_path,
            args.data_dir, args.export_viz, args.project, log=self.log,
            run_from_source=self.run_from_source,
            junit_xml_output=args.junit_xml_output,
            minimal_output=args.minimal_output)
        return export.run()

    def command_viz(self, args):
        if not self.initialised:
            self.initialised = self._init_haros_dir(overwrite=False)
        data_dir = args.data_dir or self.viz_dir
        if not os.path.isdir(data_dir):
            raise ValueError("Not a directory: " + data_dir)
        server = HarosVizRunner(self.haros_dir, self.config_path, data_dir,
            args.server_host, args.headless, log=self.log,
            run_from_source=self.run_from_source)
        return server.run()

    def command_parse(self, args):
        if not self.initialised:
            self.initialised = self._init_haros_dir(overwrite=False)
        if args.data_dir and not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        project_file = args.project_file or self.index_path
        if not os.path.isfile(project_file):
            raise ValueError("Not a file: " + project_file)
        if args.ws:
            if not os.path.isdir(args.ws):
                raise ValueError("Not a directory: " + args.ws)
        parse = HarosParseRunner(self.haros_dir, self.config_path,
            project_file, args.data_dir, log=self.log,
            run_from_source=self.run_from_source, use_repos=args.use_repos,
            ws=args.ws, copy_env=args.env, use_cache=(not args.no_cache),
            overwrite_cache=(not args.no_write_cache),
            junit_xml_output=args.junit_xml_output,
            minimal_output=args.minimal_output)
        return parse.run()

    def command_news(self, args):
        print("")
        print("[2021-08-03] A new major version is under development!")
        print("It will redesign some features and will include "
              "support for ROS2.")
        print("Contribute by participating "
              "in a short survey on Google Forms:")
        print("    https://forms.gle/Ni415XYAQwyDbHE89")
        print("")
        print("[2021-08-03] HAROS Tutorial at IROS 2021")
        print("There will be a tutorial session on how to use HAROS,")
        print(" as part of the IROS 2021 conference.")
        print("For more details:")
        print("    http://haslab.github.io/SAFER/iros21-tutorial.html")

    def parse_arguments(self, argv = None):
        parser = ArgumentParser(prog = "haros",
                                description = "ROS quality assurance.")
        parser.add_argument("--home",
                            help=("HAROS data and config directory (default: "
                                  + self.haros_dir))
        parser.add_argument("--config",
                            help=("HAROS config location (default: "
                                  + self.haros_dir + "configs.yaml"))
        parser.add_argument("--debug", action = "store_true",
                            help = "set debug logging")
        parser.add_argument("-c", "--cwd",
                            help = "change current directory before running")
        subparsers = parser.add_subparsers()
        self._init_parser(subparsers.add_parser("init"))
        self._full_parser(subparsers.add_parser("full"))
        self._analyse_parser(subparsers.add_parser("analyse"))
        self._export_parser(subparsers.add_parser("export"))
        self._viz_parser(subparsers.add_parser("viz"))
        self._parse_parser(subparsers.add_parser("parse"))
        self._news_parser(subparsers.add_parser("news"))
        return parser.parse_args(argv)

    def _init_parser(self, parser):
        parser.add_argument("--minimal-output", action='store_true',
                            help = "output only those file(s) required to view the report")
        parser.set_defaults(command = self.command_init)

    def _full_parser(self, parser):
        parser.add_argument("-r", "--use-repos", action = "store_true",
                            help = "use repository information")
        parser.add_argument("-s", "--server-host", default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.add_argument("-p", "--project-file",
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("-n", "--parse-nodes", action = "store_true",
                            help = "parse C++/Python nodes (slow)")
        parser.add_argument("--env", action = "store_true",
                            help = "use a copy of current environment")
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
        parser.add_argument("--ws", help = "set the catkin workspace directory")
        parser.add_argument("--no-cache", action = "store_true",
                            help = "do not use available caches")
        parser.add_argument("--no-write-cache", action = "store_true",
                            help = "do not update parsing cache")
        parser.add_argument("--junit-xml-output", action='store_true',
                            help = "output JUnit XML report file(s)")
        parser.add_argument("--minimal-output", action='store_true',
                            help = "output only those file(s) required to view the report")
        group = parser.add_mutually_exclusive_group()
        group.add_argument("-w", "--whitelist", nargs = "*",
                           help = "execute only these plugins")
        group.add_argument("-b", "--blacklist", nargs = "*",
                           help = "skip these plugins")
        parser.add_argument("--no-hardcoded", action="store_true",
                            help="do not rely on hard-coded nodes")
        parser.add_argument("--headless", action = "store_true",
                            help = "start server without web browser")
        parser.set_defaults(command = self.command_full)

    def _analyse_parser(self, parser):
        parser.add_argument("-r", "--use-repos", action = "store_true",
                            help = "use repository information")
        parser.add_argument("-p", "--project-file",
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("-n", "--parse-nodes", action = "store_true",
                            help = "parse C++/Python nodes (slow)")
        parser.add_argument("--env", action = "store_true",
                            help = "use a copy of current environment")
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
        parser.add_argument("--ws", help = "set the catkin workspace directory")
        parser.add_argument("--no-cache", action = "store_true",
                            help = "do not use available caches")
        parser.add_argument("--no-write-cache", action = "store_true",
                            help = "do not update parsing cache")
        parser.add_argument("--junit-xml-output", action='store_true',
                            help = "output JUnit XML report file(s)")
        parser.add_argument("--minimal-output", action='store_true',
                            help = "output only those file(s) required to view the report")
        group = parser.add_mutually_exclusive_group()
        group.add_argument("-w", "--whitelist", nargs = "*",
                           help = "execute only these plugins")
        group.add_argument("-b", "--blacklist", nargs = "*",
                           help = "skip these plugins")
        parser.add_argument("--no-hardcoded", action="store_true",
                            help="do not rely on hard-coded nodes")
        parser.set_defaults(command = self.command_analyse)

    def _export_parser(self, parser):
        parser.add_argument("-v", "--export-viz", action = "store_true",
                            help = "export HTML viz files")
        parser.add_argument("-p", "--project", default = "default",
                            help = "name of project to export")
        parser.add_argument("data_dir", metavar = "dir",
                            help = "where to export data")
        parser.add_argument("--junit-xml-output", action='store_true',
                            help = "output JUnit XML report file(s)")
        parser.add_argument("--minimal-output", action='store_true',
                            help = "output only those file(s) required to view the report")
        parser.set_defaults(command = self.command_export)

    def _viz_parser(self, parser):
        parser.add_argument("-d", "--data-dir",
                            help = "served data directory")
        parser.add_argument("-s", "--server-host", default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.add_argument("--headless", action = "store_true",
                            help = "start server without web browser")
        parser.add_argument("--minimal-output", action='store_true',
                            help = "output only those file(s) required to view the report")
        parser.set_defaults(command = self.command_viz)

    def _parse_parser(self, parser):
        parser.add_argument("-r", "--use-repos", action = "store_true",
                            help = "use repository information")
        parser.add_argument("-p", "--project-file",
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("--env", action = "store_true",
                            help = "use a copy of current environment")
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
        parser.add_argument("--ws", help = "set the catkin workspace directory")
        parser.add_argument("--no-cache", action = "store_true",
                            help = "do not use available caches")
        parser.add_argument("--no-write-cache", action = "store_true",
                            help = "do not update parsing cache")
        parser.add_argument("--junit-xml-output", action='store_true',
                            help = "output JUnit XML report file(s)")
        parser.add_argument("--minimal-output", action='store_true',
                            help = "output only those file(s) required to view the report")
        parser.set_defaults(command = self.command_parse)

    def _news_parser(self, parser):
        parser.set_defaults(command=self.command_news)

    def _set_directories(self, args):
        if args.home:
            self.haros_dir = args.home
            self.index_path = os.path.join(self.haros_dir, "index.yaml")
            self.log_path = os.path.join(self.haros_dir, "log.txt")
            self.viz_dir = os.path.join(self.haros_dir, "viz")
        if args.config:
            self.config_path = args.config
        else:
            self.config_path = os.path.join(self.haros_dir, "configs.yaml")

    def _init_haros_dir(self, overwrite=False):
        print("[HAROS] Running setup operations...")
        exists = os.path.exists(self.haros_dir)
        if exists and not os.path.isdir(self.haros_dir):
            raise RuntimeError(("Could not initialise; " + self.haros_dir
                                + " already exists and is not a directory."))
        if not exists:
            self.log.info("Creating %s", self.haros_dir)
            os.makedirs(self.haros_dir)
        self._generate_dir(self.haros_dir, self.DIR_STRUCTURE,
                           overwrite=overwrite)
        if overwrite or not os.path.exists(self.viz_dir):
            viz.install(self.viz_dir, self.run_from_source, force=True, minimal_output=self.minimal_output)
        return True

    def _generate_dir(self, path, dir_dict, overwrite=True):
        """Recursively create a given directory structure."""
        self.log.debug("HarosRunner._generate_dir %s %s", path, str(dir_dict))
        for name, contents in dir_dict.items():
            new_path = os.path.join(path, name)
            exists = os.path.exists(new_path)
            if isinstance(contents, basestring):
                if exists and not os.path.isfile(new_path):
                    raise RuntimeError("Could not create file: " + new_path)
                if overwrite or not exists:
                    self.log.info("Creating %s", new_path)
                    with open(new_path, "w") as handle:
                        handle.write(contents)
            elif isinstance(contents, dict):
                if exists and not os.path.isdir(new_path):
                    raise RuntimeError("Could not create dir: " + new_path)
                if not exists:
                    self.log.info("Creating %s", new_path)
                    os.mkdir(new_path)
                self._generate_dir(new_path, contents, overwrite=overwrite)


###############################################################################
#   Base Command Runner
###############################################################################

class HarosRunner(object):
    """This is a base class for the specific commands that HAROS provides."""

    def __init__(self, haros_dir, config_path, log, run_from_source,
                 junit_xml_output = False, minimal_output = False):
        self.root               = haros_dir
        self.config_path        = config_path
        self.repo_dir           = os.path.join(haros_dir, "repositories")
        self.export_dir         = os.path.join(haros_dir, "export")
        self.project_dir        = os.path.join(haros_dir, "projects")
        self.viz_dir            = os.path.join(haros_dir, "viz")
        self.pyflwor_dir        = os.path.join(haros_dir, "pyflwor")
        self.log                = log or logging.getLogger()
        self.run_from_source    = run_from_source
        self.settings           = None
        self.junit_xml_output   = junit_xml_output
        self.minimal_output     = minimal_output

    def run(self):
        return True

    def _empty_dir(self, dir_path):
        """Deletes all files within a directory."""
        self.log.debug("HarosRunner._empty_dir %s", dir_path)
        for filename in os.listdir(dir_path):
            path = os.path.join(dir_path, filename)
            if os.path.isfile(path):
                self.log.debug("Removing file %s", path)
                os.unlink(path)

    def _ensure_dir(self, dir_path, empty = False):
        """Create a directory if it does not exist."""
        self.log.debug("HarosRunner._ensure_dir %s", dir_path)
        if not os.path.isdir(dir_path):
            if os.path.isfile(dir_path):
                raise RuntimeError("Could not create dir: " + dir_path)
            os.makedirs(dir_path)
        elif empty:
            self._empty_dir(dir_path)

    def _load_settings(self):
        try:
            self.settings = HarosSettings.parse_from(self.config_path)
        except IOError:
            self.settings = HarosSettings()

    def _setup_lazy_node_parser(self):
        if self.run_from_source:
            HardcodedNodeParser.model_dir = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "models")
            )
        else:
            HardcodedNodeParser.model_dir = resource_filename(
                Requirement.parse("haros"), "haros/models"
            )
        HardcodedNodeParser.distro = os.environ.get("ROS_DISTRO", "kinetic")


###############################################################################
#   HAROS Command Runner (analyse)
###############################################################################

class HarosCommonExporter(HarosRunner):
    """This is just an interface with common methods."""

    def _prepare_project(self):
        self.current_dir = os.path.join(self.io_projects_dir, self.project)
        self._ensure_dir(self.current_dir)
        self.json_dir = os.path.join(self.data_dir, self.project)
        self._ensure_dir(self.json_dir)

    def _export_project_data(self, exporter):
        report = self.database.report
    # ----- general data
        exporter.export_packages(self.json_dir, report.by_package)
        exporter.export_rules(self.json_dir, self.database.rules)
        if not self.minimal_output:
            # Currently, exported metrics are not used by the dashboard,
            # so exporting them is optional.
            exporter.export_metrics(self.json_dir, self.database.metrics)
        exporter.export_summary(self.json_dir, report, self.database.history)
    # ----- extracted configurations
        exporter.export_configurations(self.json_dir, report.by_config)
    # ----- compliance reports
        out_dir = os.path.join(self.json_dir, "compliance")
        self._ensure_dir(out_dir, empty = True)
        exporter.export_other_violations(out_dir, report.violations)
        out_dir = os.path.join(self.json_dir, "compliance", "source")
        self._ensure_dir(out_dir, empty = True)
        exporter.export_source_violations(out_dir, report.by_package)
        out_dir = os.path.join(self.json_dir, "compliance", "runtime")
        self._ensure_dir(out_dir, empty = True)
        exporter.export_runtime_violations(out_dir, report.by_config)
    # ----- individual metrics reports
        if not self.minimal_output:
            # Currently, exported metrics are not used by the dashboard,
            # so exporting them is optional.
            out_dir = os.path.join(self.json_dir, "metrics")
            self._ensure_dir(out_dir, empty = True)
            exporter.export_measurements(out_dir, report.by_package)
        #


class HarosAnalyseRunner(HarosCommonExporter):
    distro_url = ("https://raw.githubusercontent.com/ros/rosdistro/master/"
                  + os.environ.get("ROS_DISTRO", "kinetic")
                  + "/distribution.yaml")

    def __init__(self, haros_dir, config_path, project_file, data_dir,
                 whitelist, blacklist, log=None, run_from_source=False,
                 use_repos=False, ws=None, parse_nodes=False,
                 copy_env=False, use_cache=True, overwrite_cache=True,
                 settings=None, junit_xml_output=False,
                 minimal_output=False, no_hardcoded=False):
        HarosRunner.__init__(self, haros_dir, config_path, log,
            run_from_source, junit_xml_output, minimal_output)
        self.project_file = project_file
        self.workspace = ws
        self.use_repos = use_repos
        self.parse_nodes = parse_nodes
        self.copy_env = copy_env
        self.use_cache = use_cache
        self.overwrite_cache = overwrite_cache
        self.whitelist = whitelist
        self.blacklist = blacklist
        self.settings = settings
        self.no_hardcoded = no_hardcoded
        self.project = None
        self.database = None
        self.current_dir = None
        self.json_dir = None
        if data_dir:
            self.export_viz = True
            self.viz_dir = data_dir
            self.io_projects_dir = os.path.join(data_dir, "projects")
        else:
            self.export_viz = False
            self.io_projects_dir = self.project_dir
        self.data_dir = os.path.join(self.viz_dir, "data")

    @property
    def definitions_file(self):
        if self.run_from_source:
            return os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "definitions.yaml"))
        return resource_filename(Requirement.parse("haros"),
                                 "haros/definitions.yaml")

    def _load_settings(self):
        try:
            self.settings = HarosSettings.parse_from(
                self.config_path, ws=self.workspace)
        except IOError:
            self.settings = HarosSettings(workspace=self.workspace)

    def run(self):
        if self.settings is None:
            self._load_settings()
        self.database = HarosDatabase()
        self._setup_lazy_node_parser()
        plugins, rules, metrics = self._load_definitions_and_plugins()
        node_cache = {}
        if self.parse_nodes and self.use_cache:
            parse_cache = os.path.join(self.root, "parse_cache.json")
            try:
                with open(parse_cache, "r") as f:
                    node_cache = json.load(f)
            except IOError as e:
                self.log.warning("Could not read parsing cache: %s", e)
        configs, nodes, env = self._extract_metamodel(node_cache, rules)
        self.current_dir = os.path.join(self.io_projects_dir, self.project)
        self._load_history()
        self._extract_configurations(self.database.project, configs, nodes, env)
        self._make_node_configurations(self.database.project, nodes, env)
        self._parse_hpl_properties()
        self._analyse(plugins, rules, metrics)
        self._save_results(node_cache)
        self.database = None
        return True

    def _extract_metamodel(self, node_cache, rules):
        print("[HAROS] Reading project and indexing source code...")
        self.log.debug("Project file %s", self.project_file)
        env = dict(os.environ) if self.copy_env else self.settings.environment
        distro = self.distro_url if self.use_repos else None
        extractor = ProjectExtractor(self.project_file, env = env,
                                     repo_path = self.repo_dir,
                                     distro_url = distro,
                                     require_repos = True,
                                     node_cache = node_cache,
                                     parse_nodes = self.parse_nodes)
        if self.parse_nodes:
            print("  > Parsing nodes might take some time.")
            start_time = timer()
            # NOTE: this updates settings with ignore-line comments
            extractor.index_source(settings = self.settings)
            end_time = timer()
            self.log.debug("Parsing time: %f s", end_time - start_time)
        else:
            # NOTE: this updates settings with ignore-line comments
            extractor.index_source(settings = self.settings)
        self.project = extractor.project.name
        if not extractor.project.packages:
            raise RuntimeError("There are no packages to analyse.")
        self.database.register_project(extractor.project)
        self.database.update_analysis_preferences(self.settings)
        rs = self.database.register_rules(extractor.rules, prefix="user:",
            ignored_rules=self.settings.ignored_rules,
            ignored_tags=self.settings.ignored_tags)
        rules.update(rs) # FIXME this is a hammer
        return extractor.configurations, extractor.node_specs, env

    def _extract_configurations(self, project, configs, nodes, environment):
        # FIXME nodes is unused
        empty_dict = {}
        empty_list = ()
        for name, data in configs.items():
            if isinstance(data, list):
                builder = ConfigurationBuilder(name, environment, self.database)
                launch_files = data
                hpl = empty_dict
            else:
                builder = ConfigurationBuilder(name, environment, self.database,
                    hints=data.get("hints"), no_hardcoded=self.no_hardcoded)
                launch_files = data["launch"]
                hpl = data.get("hpl", empty_dict)
            for launch_file in launch_files:
                parts = launch_file.split(os.sep, 1)
                if not len(parts) == 2:
                    raise ValueError("invalid launch file: " + launch_file)
                pkg = self.database.packages.get("package:" + parts[0])
                if not pkg:
                    raise ValueError("unknown package: " + parts[0])
                path = os.path.join(pkg.path, parts[1])
                launch = self.database.get_file(path)
                if not launch:
                    raise ValueError("unknown launch file: " + launch_file)
                builder.add_launch(launch)
            cfg = builder.build()
            for msg in builder.errors:
                self.log.warning("Configuration %s: %s", cfg.name, msg)
            for p in hpl.get("properties", empty_list):
                cfg.hpl_properties.append(p)
            for a in hpl.get("assumptions", empty_list):
                cfg.hpl_assumptions.append(a)
            if isinstance(data, list):
                cfg.user_attributes = {}
            else:
                cfg.user_attributes = data.get("user_data", {})
            project.configurations.append(cfg)
            self.database.configurations.append(cfg)

    def _make_node_configurations(self, project, nodes, environment):
        self.log.debug("Creating Configurations for node specs.")
        for node_name, node_hints in nodes.items():
            try:
                node = project.get_node(node_name)
            except ValueError as e:
                self.log.error(str(e))
                continue
            if not node.package._analyse:
                self.log.debug("Skipping %s; package not marked for analysis",
                               node_name)
                continue
            builder = ConfigurationBuilder(node_name.replace("/", "_"),
                                           environment, self.database)
            builder.add_rosrun(node)
            cfg = builder.build()
            for msg in builder.errors:
                self.log.warning("Configuration %s: %s", cfg.name, msg)
            cfg.hpl_properties = list(node.hpl_properties)
            cfg.hpl_assumptions = list(node.hpl_assumptions)
            cfg.user_attributes = node_hints.get("user_data", {})
            project.configurations.append(cfg)
            self.database.configurations.append(cfg)

    def _load_history(self):
        """
        Load analyis history from database file
        and add it to self.database.history, append the previously
        'current' analysis report to history.
        """
        haros_db_path = os.path.join(self.current_dir, "haros.db")
        try:
            haros_db = HarosDatabase.load_state(haros_db_path)
        except IOError:
            self.log.info("No previous analysis data for " + self.project)
        except ValueError:
            self.log.info("Previous analysis data for " + self.project
                          + " uses an incompatible format.")
        else:
            self.database.history = haros_db.history
            # Commit the previous report (the report generated during the
            # last analysis) to history.
            self.database.history.append(haros_db.report)

    def _load_definitions_and_plugins(self):
        rules = set()
        metrics = set()
        print("[HAROS] Loading common definitions...")
        rs, ms = self.database.load_definitions(self.definitions_file,
                ignored_rules=self.settings.ignored_rules,
                ignored_tags=self.settings.ignored_tags,
                ignored_metrics=self.settings.ignored_metrics)
        rules.update(rs)
        metrics.update(ms)
        print("[HAROS] Loading plugins...")
        blacklist = self.blacklist or self.settings.plugin_blacklist
        plugins = Plugin.load_plugins(whitelist=self.whitelist,
                                      blacklist=blacklist,
                                      common_rules=self.database.rules,
                                      common_metrics=self.database.metrics)
        if not plugins:
            if blacklist or self.whitelist:
                msg = ("Could not find any analysis plugins "
                       "for the provided names.")
            else:
                msg = "Could not find any analysis plugins."
            raise RuntimeError(msg)
        for plugin in plugins:
            print("  > Loaded " + plugin.name)
            prefix = plugin.name + ":"
            rs = self.database.register_rules(plugin.rules, prefix=prefix,
                    ignored_rules=self.settings.ignored_rules,
                    ignored_tags=self.settings.ignored_tags)
            ms = self.database.register_metrics(plugin.metrics, prefix=prefix,
                    ignored_metrics=self.settings.ignored_metrics)
            rules.update(rs)
            metrics.update(ms)
        return plugins, rules, metrics

    def _parse_hpl_properties(self):
        configs = []
        nodes = []
        for config in self.database.project.configurations:
            if config.hpl_properties or config.hpl_assumptions:
                configs.append(config)
        for pkg in self.database.project.packages:
            for node in pkg.nodes:
                if node.hpl_properties or node.hpl_assumptions:
                    if pkg._analyse:
                        nodes.append(node)
                    else:
                        self.log.warning((
                            "Found HPL specifications for node %s, "
                            "but package %s is not marked for analysis."),
                            node.node_name, node.package.name)
        if not configs and not nodes:
            return
        parser = None
        try:
            # lazy import; this is an optional dependency
            from .hpl_parser import UserSpecParser
            parser = UserSpecParser()
        except ImportError as e:
            self.log.warning(("Found HPL specifications, "
                "but the HPL parser could not be found."))
            self.log.error(repr(e))
            return
        for config in configs:
            self.log.debug("Parsing HPL properties for %s", config.id)
            parser.parse_config_specs(config)
        for node in nodes:
            self.log.debug("Parsing HPL properties for %s", node.id)
            parser.parse_node_specs(node)

    def _analyse(self, plugins, rules, metrics):
        print("[HAROS] Running analysis...")
        self._empty_dir(self.export_dir)
        temp_path = tempfile.mkdtemp()
        analysis = AnalysisManager(self.database, temp_path, self.export_dir,
                                   pyflwor_dir=self.pyflwor_dir)
        try:
            analysis.run(plugins, allowed_rules=rules, allowed_metrics=metrics,
                         ignored_lines=self.settings.ignored_lines)
            self.database.report = analysis.report
        finally:
            rmtree(temp_path)

    def _save_results(self, node_cache):
        print("[HAROS] Saving analysis results...")
        if self.export_viz:
            viz.install(self.viz_dir, self.run_from_source, minimal_output=self.minimal_output)
        self._ensure_dir(self.data_dir)
        self._ensure_dir(self.current_dir)
        # NOTE: The database has a tendency to grow in size.
        # Old reports will have violations and metrics with references
        # to old packages, files, etc. There will be multiple versions
        # of the same projects over time, as long as the history exists.
        # This is why I added "_compact()" to the database's save_state()
        # function.
        if not self.minimal_output:
            self.database.save_state(os.path.join(self.current_dir, "haros.db"))
        self.log.debug("Exporting on-memory data manager.")
        self._prepare_project()
        exporter = JsonExporter()
        self._export_project_data(exporter)
        exporter.export_projects(self.data_dir, (self.database.project,),
                                 overwrite = False)
        if self.junit_xml_output:
            junit_exporter = JUnitExporter()
            junit_exporter.export_report(self.data_dir, self.database)
        if self.parse_nodes and self.overwrite_cache:
            for node in self.database.nodes.values():
                node_cache[node.node_name] = node.to_JSON_object()
            parse_cache = os.path.join(self.root, "parse_cache.json")
            try:
                with open(parse_cache, "w") as f:
                    json.dump(node_cache, f, indent=2, separators=(",", ":"))
            except IOError as e:
                self.log.warning("Could not save parsing cache: %s", e)


###############################################################################
#   HAROS Command Runner (parse)
###############################################################################

class HarosParseRunner(HarosAnalyseRunner):
    def __init__(self, haros_dir, config_path, project_file, data_dir,
                 log=None, run_from_source=False, use_repos=False, ws=None,
                 copy_env=False, use_cache=True, settings=None,
                 overwrite_cache=True,
                 junit_xml_output = False, minimal_output = False):
        HarosAnalyseRunner.__init__(
            self, haros_dir, config_path, project_file, data_dir,
            [], [], log=log, run_from_source=run_from_source,
            use_repos=use_repos, ws=ws, parse_nodes=True, copy_env=copy_env,
            use_cache=use_cache, overwrite_cache=overwrite_cache,
            settings=settings, junit_xml_output=junit_xml_output,
            minimal_output=minimal_output
        )

    def _load_definitions_and_plugins(self):
        rules = set()
        metrics = set()
        print("[HAROS] Loading common definitions...")
        rs, ms = self.database.load_definitions(self.definitions_file,
                ignored_rules=self.settings.ignored_rules,
                ignored_tags=self.settings.ignored_tags,
                ignored_metrics=self.settings.ignored_metrics)
        rules.update(rs)
        metrics.update(ms)
        return (), rules, metrics


###############################################################################
#   HAROS Command Runner (export)
###############################################################################

# There are four possible export scenarios.
# 1. export from memory without viz (default dir)
    # .haros/projects/<name>
    # .haros/viz/data/<name>
# 2. export from memory with viz (custom dir)
    # dir/projects/<name>
    # dir/data/<name>
# 3. export without viz
    # dir/<name>
    # dir/<name>
# 4. export with viz
    # dir/projects/<name>
    # dir/data/<name>

class HarosExportRunner(HarosCommonExporter):
    def __init__(self, haros_dir, config_path, data_dir, export_viz, project,
                 log = None, run_from_source = False, junit_xml_output = False,
                 minimal_output = False):
        HarosRunner.__init__(self, haros_dir, config_path, log,
            run_from_source, junit_xml_output, minimal_output)
        self.project = project
        self.project_data_list = []
        self.export_viz = export_viz
        self.database = None
        self.current_dir = None
        self.haros_db = None
        if export_viz:
            self.viz_dir = data_dir
            self.data_dir = os.path.join(data_dir, "data")
            self.io_projects_dir = os.path.join(data_dir, "projects")
        else:
            self.data_dir = data_dir
            self.io_projects_dir = data_dir

    def run(self):
        print("[HAROS] Exporting analysis results...")
        self._prepare_directory()
        exporter = JsonExporter()
        for project in self._project_list():
            self.project = project
            self._prepare_project()
            if self._load_database():
                if not self.minimal_output:
                    self._save_database()
                self._export_project_data(exporter)
        exporter.export_projects(self.data_dir, self.project_data_list)
        if self.junit_xml_output:
            junit_exporter = JUnitExporter()
            junit_exporter.export_report(self.data_dir, self.database)
        self.database = None
        self.project_data_list = []
        return True

    def _prepare_directory(self):
        if self.export_viz:
            viz.install(self.viz_dir, self.run_from_source)
        self._ensure_dir(self.data_dir)
        self._ensure_dir(self.io_projects_dir)

    def _project_list(self):
        if self.project == "all":
            return [name for name in os.listdir(self.project_dir)
                    if os.path.isdir(os.path.join(self.project_dir, name))]
        return [self.project]

    def _load_database(self):
        self.log.debug("Exporting data manager from file.")
        self.haros_db = os.path.join(self.project_dir, self.project,
                                     "haros.db")
        if not os.path.isfile(self.haros_db):
            self.log.error("There is no analysis data for " + self.project)
            return False
        try:
            self.database = HarosDatabase.load_state(self.haros_db)
        except (IOError, ValueError):
            return False
        self.project_data_list.append(self.database.project)
        return True

    def _save_database(self):
        db_path = os.path.join(self.current_dir, "haros.db")
        self.log.debug("Copying %s to %s", self.haros_db, db_path)
        copyfile(self.haros_db, db_path)


###############################################################################
#   HAROS Command Runner (viz)
###############################################################################

class HarosVizRunner(HarosRunner):
    def __init__(self, haros_dir, config_path, server_dir, host_str, headless,
                 log = None, run_from_source = False):
        HarosRunner.__init__(self, haros_dir, config_path, log, run_from_source)
        self.server_dir = server_dir
        self.host = host_str
        self.headless = headless

    def run(self):
        return viz.serve(self.server_dir, self.host, headless = self.headless)


###############################################################################
#   HAROS Main Function
###############################################################################

def main(argv=None, source_runner=False):
    launcher = HarosLauncher(run_from_source=source_runner)
    if launcher.launch(argv=argv):
        return 0
    return 1
