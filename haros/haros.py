
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
# |-+ plugins
#   |-+ ...
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

from argparse import ArgumentParser
import json
import logging
import os
import subprocess
import tempfile

from shutil import copyfile, rmtree
from pkg_resources import Requirement, resource_filename

from .data import HarosDatabase, HarosSettings
from .extractor import ProjectExtractor
from .config_builder import ConfigurationBuilder
from .plugin_manager import Plugin
from .analysis_manager import AnalysisManager
from .export_manager import JsonExporter
from . import visualiser as viz


###############################################################################
#   HAROS Launcher (Main Application)
###############################################################################

class HarosLauncher(object):
    """This class contains the necessary methods to launch HAROS.
        It is responsible for initialising directories and data
        structures, as well as parsing program arguments.
    """

    HAROS_DIR       = os.path.join(os.path.expanduser("~"), ".haros")
    DEFAULT_INDEX   = os.path.join(HAROS_DIR, "index.yaml")
    USER_SETINGS    = os.path.join(HAROS_DIR, "configs.yaml")
    LOG_PATH        = os.path.join(HAROS_DIR, "log.txt")
    VIZ_DIR         = os.path.join(HAROS_DIR, "viz")

    def __init__(self, run_from_source = False):
        self.log = logging.getLogger()
        self.run_from_source = run_from_source
        self.initialised = False

    def launch(self, argv = None):
        args = self.parse_arguments(argv)
        if args.debug:
            logging.basicConfig(filename = self.LOG_PATH, filemode = "w",
                                level = logging.DEBUG)
        else:
            logging.basicConfig(level = logging.WARNING)
        original_path = os.getcwd()
        try:
            if not os.path.isdir(self.HAROS_DIR):
                print "[HAROS] It seems this is a first run."
                self.command_init(args, HarosSettings())
            try:
                settings = HarosSettings.parse_from(self.USER_SETINGS)
            except IOError:
                settings = HarosSettings()
            if args.cwd:
                os.chdir(args.cwd)
            self.log.info("Executing selected command.")
            return args.command(args, settings)
        except RuntimeError as err:
            self.log.error(str(err))
            return False
        finally:
            os.chdir(original_path)

    def command_init(self, args, settings):
        if not self.initialised:
            init = HarosInitRunner(self.HAROS_DIR, self.log,
                                   self.run_from_source)
            if not init.run():
                return False
            self.initialised = True
        return True

    def command_full(self, args, settings):
        return (self.command_analyse(args, settings)
                and self.command_viz(args, settings))

    def command_analyse(self, args, settings):
        if args.data_dir and not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        if not os.path.isfile(args.project_file):
            raise ValueError("Not a file: " + args.package_index)
        analyse = HarosAnalyseRunner(self.HAROS_DIR, args.project_file,
                                     args.data_dir, args.whitelist,
                                     args.blacklist, log = self.log,
                                     run_from_source = self.run_from_source,
                                     use_repos = args.use_repos,
                                     parse_nodes = args.parse_nodes,
                                     copy_env = args.env,
                                     use_cache = not args.no_cache,
                                     settings = settings)
        return analyse.run()

    def command_export(self, args, settings):
        if not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        export = HarosExportRunner(self.HAROS_DIR, args.data_dir,
                                   args.export_viz, args.project,
                                   log = self.log,
                                   run_from_source = self.run_from_source)
        return export.run()

    def command_viz(self, args, settings):
        data_dir = args.data_dir or self.VIZ_DIR
        if not os.path.isdir(data_dir):
            raise ValueError("Not a directory: " + data_dir)
        server = HarosVizRunner(self.HAROS_DIR, data_dir,
                                args.server_host, args.headless,
                                log = self.log,
                                run_from_source = self.run_from_source)
        return server.run()

    def parse_arguments(self, argv = None):
        parser = ArgumentParser(prog = "haros",
                                description = "ROS quality assurance.")
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
        return parser.parse_args(argv)

    def _init_parser(self, parser):
        parser.set_defaults(command = self.command_init)

    def _full_parser(self, parser):
        parser.add_argument("-r", "--use-repos", action = "store_true",
                            help = "use repository information")
        parser.add_argument("-s", "--server-host", default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.add_argument("-p", "--project-file",
                            default = self.DEFAULT_INDEX,
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("-n", "--parse-nodes", action = "store_true",
                            help = "parse C++/Python nodes (slow)")
        parser.add_argument("--env", action = "store_true",
                            help = "use a copy of current environment")
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
        parser.add_argument("--no-cache", action = "store_true",
                            help = "do not use available caches")
        group = parser.add_mutually_exclusive_group()
        group.add_argument("-w", "--whitelist", nargs = "*",
                           help = "execute only these plugins")
        group.add_argument("-b", "--blacklist", nargs = "*",
                           help = "skip these plugins")
        parser.add_argument("--headless", action = "store_true",
                            help = "start server without web browser")
        parser.set_defaults(command = self.command_full)

    def _analyse_parser(self, parser):
        parser.add_argument("-r", "--use-repos", action = "store_true",
                            help = "use repository information")
        parser.add_argument("-p", "--project-file",
                            default = self.DEFAULT_INDEX,
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("-n", "--parse-nodes", action = "store_true",
                            help = "parse C++/Python nodes (slow)")
        parser.add_argument("--env", action = "store_true",
                            help = "use a copy of current environment")
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
        parser.add_argument("--no-cache", action = "store_true",
                            help = "do not use available caches")
        group = parser.add_mutually_exclusive_group()
        group.add_argument("-w", "--whitelist", nargs = "*",
                           help = "execute only these plugins")
        group.add_argument("-b", "--blacklist", nargs = "*",
                           help = "skip these plugins")
        parser.set_defaults(command = self.command_analyse)

    def _export_parser(self, parser):
        parser.add_argument("-v", "--export-viz", action = "store_true",
                            help = "export HTML viz files")
        parser.add_argument("-p", "--project", default = "default",
                            help = "name of project to export")
        parser.add_argument("data_dir", metavar = "dir",
                            help = "where to export data")
        parser.set_defaults(command = self.command_export)

    def _viz_parser(self, parser):
        parser.add_argument("-d", "--data-dir", default = self.VIZ_DIR,
                            help = "served data directory")
        parser.add_argument("-s", "--server-host", default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.add_argument("--headless", action = "store_true",
                            help = "start server without web browser")
        parser.set_defaults(command = self.command_viz)


###############################################################################
#   Base Command Runner
###############################################################################

class HarosRunner(object):
    """This is a base class for the specific commands that HAROS provides."""

    def __init__(self, haros_dir, log, run_from_source):
        self.root               = haros_dir
        self.plugin_dir         = os.path.join(haros_dir, "plugins")
        self.repo_dir           = os.path.join(haros_dir, "repositories")
        self.export_dir         = os.path.join(haros_dir, "export")
        self.project_dir        = os.path.join(haros_dir, "projects")
        self.viz_dir            = os.path.join(haros_dir, "viz")
        self.log                = log or logging.getLogger()
        self.run_from_source    = run_from_source

    def run(self):
        return True

    def _generate_dir(self, path, dir_dict):
        """Recursively create a given directory structure."""
        self.log.debug("HarosRunner._generate_dir %s %s", path, str(dir_dict))
        for name, contents in dir_dict.iteritems():
            new_path = os.path.join(path, name)
            if isinstance(contents, basestring):
                if os.path.exists(new_path) and not os.path.isfile(new_path):
                    raise RuntimeError("Could not create file: " + new_path)
                self.log.info("Creating %s", new_path)
                with open(new_path, "w") as handle:
                    handle.write(contents)
            elif isinstance(contents, dict):
                if not os.path.isdir(new_path):
                    if os.path.exists(new_path):
                        raise RuntimeError("Could not create dir: " + new_path)
                    self.log.info("Creating %s", new_path)
                    os.mkdir(new_path)
                self._generate_dir(new_path, contents)

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


###############################################################################
#   HAROS Command Runner (init)
###############################################################################

class HarosInitRunner(HarosRunner):
    PLUGIN_REPOSITORY = "https://github.com/git-afsantos/haros_plugins.git"

    DIR_STRUCTURE = {
        "index.yaml": "%YAML 1.1\n---\npackages: []\n",
        "configs.yaml": (
            "%YAML 1.1\n---\n"
            " # workspace: '/path/to/ws'\n"
            " # environment: null\n"
            " # plugin_blacklist: []\n"
            " # cpp:\n"
            # " #    parser: clang,\n"
            " #    parser_lib: '/usr/lib/llvm-3.8/lib'\n"
            " #    std_includes: '/usr/lib/llvm-3.8/lib/clang/3.8.0/include'\n"
            " #    compile_db: '/path/to/ws/build'\n"
        ),
        "parse_cache.json": "{}",
        "plugins": {},
        "repositories": {},
        "export": {},
        "projects": {
            "default": {}
        }
        # viz is generated on viz.install
    }

    def run(self):
        print "[HAROS] Running initial setup operations..."
        if os.path.exists(self.root) and not os.path.isdir(self.root):
            raise RuntimeError(("Could not init; " + self.root
                                + " already exists and is not a directory."))
        if not os.path.exists(self.root):
            self.log.info("Creating %s", self.root)
            os.makedirs(self.root)
        has_plugins = os.path.exists(self.plugin_dir)
        self._generate_dir(self.root, self.DIR_STRUCTURE)
        viz.install(self.viz_dir, self.run_from_source, force = True)
        if has_plugins:
            self.log.info("Updating plugin repository.")
            wd = os.getcwd()
            os.chdir(self.plugin_dir)
            with open(os.devnull, "w") as devnull:
                subprocess.check_call(["git", "rev-parse"], stdout = devnull,
                                      stderr = subprocess.STDOUT)
            self.log.debug("Executing git pull at %s.", self.plugin_dir)
            subprocess.check_call(["git", "pull"])
            os.chdir(wd)
        else:
            self.log.info("Cloning plugin repository.")
            subprocess.check_call(["git", "clone",
                                   self.PLUGIN_REPOSITORY, self.plugin_dir])
        return True


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
    # ----- metrics reports
        out_dir = os.path.join(self.json_dir, "metrics")
        self._ensure_dir(out_dir, empty = True)
        exporter.export_measurements(out_dir, report.by_package)


class HarosAnalyseRunner(HarosCommonExporter):
    distro_url = ("https://raw.githubusercontent.com/ros/rosdistro/master/"
                  + os.environ.get("ROS_DISTRO", "kinetic")
                  + "/distribution.yaml")

    def __init__(self, haros_dir, project_file, data_dir, whitelist, blacklist,
                 log = None, run_from_source = False, use_repos = False,
                 parse_nodes = False, copy_env = False, use_cache = True,
                 settings = None):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
        self.project_file = project_file
        self.use_repos = use_repos
        self.parse_nodes = parse_nodes
        self.copy_env = copy_env
        self.use_cache = use_cache
        self.whitelist = whitelist
        self.blacklist = blacklist
        self.settings = settings
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

    def run(self):
        self.database = HarosDatabase()
        plugins = self._load_definitions_and_plugins()
        node_cache = {}
        if self.parse_nodes and self.use_cache:
            parse_cache = os.path.join(self.root, "parse_cache.json")
            try:
                with open(parse_cache, "r") as f:
                    node_cache = json.load(f)
            except IOError as e:
                self.log.warning("Could not read parsing cache: %s", e)
        configs, env = self._extract_metamodel(node_cache)
        self._load_database()
        self._extract_configurations(self.database.project, configs, env)
        self._analyse(plugins)
        self._save_results(node_cache)
        self.database = None
        return True

    def _extract_metamodel(self, node_cache):
        print "[HAROS] Reading project and indexing source code..."
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
            print "  > Parsing nodes might take some time."
        extractor.index_source(settings = self.settings)
        self.project = extractor.project.name
        if not extractor.project.packages:
            raise RuntimeError("There are no packages to analyse.")
        self.database.register_project(extractor.project)
        self.database.register_rules(extractor.rules, prefix = "user:")
        return extractor.configurations, env

    def _extract_configurations(self, project, configs, environment):
        for name, data in configs.iteritems():
            if isinstance(data, list):
                builder = ConfigurationBuilder(name, environment, self.database)
                launch_files = data if isinstance(data, list) else data["launch"]
            else:
                builder = ConfigurationBuilder(name, environment, self.database,
                                               hints = data.get("hints"))
                launch_files = data["launch"]
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
            for msg in builder.errors:
                self.log.warning("Configuration %s: %s",
                                 builder.configuration.name, msg)
            project.configurations.append(builder.configuration)
            self.database.configurations.append(builder.configuration)

    def _load_database(self):
        self.current_dir = os.path.join(self.io_projects_dir, self.project)
        haros_db = os.path.join(self.current_dir, "haros.db")
        try:
            haros_db = HarosDatabase.load_state(haros_db)
        except IOError:
            self.log.info("No previous analysis data for " + self.project)
        else:
            # NOTE: This has a tendency to grow in size.
            # Old reports will have violations and metrics with references
            # to old packages, files, etc. There will be multiple versions
            # of the same projects over time, as long as the history exists.
            # This is why I added "compact" to the database.
            self.database.history = haros_db.history
            self.database.history.append(haros_db.report)

    def _load_definitions_and_plugins(self):
        print "[HAROS] Loading common definitions..."
        self.database.load_definitions(self.definitions_file)
        print "[HAROS] Loading plugins..."
        blacklist = self.blacklist or self.settings.plugin_blacklist
        plugins = Plugin.load_plugins(self.plugin_dir,
                                      whitelist = self.whitelist,
                                      blacklist = blacklist,
                                      common_rules = self.database.rules,
                                      common_metrics = self.database.metrics)
        if not plugins:
            raise RuntimeError("There are no analysis plugins.")
        for plugin in plugins:
            print "  > Loaded " + plugin.name
            prefix = plugin.name + ":"
            self.database.register_rules(plugin.rules, prefix = prefix)
            self.database.register_metrics(plugin.metrics, prefix = prefix)
        return plugins

    def _analyse(self, plugins):
        print "[HAROS] Running analysis..."
        self._empty_dir(self.export_dir)
        temp_path = tempfile.mkdtemp()
        analysis = AnalysisManager(self.database, temp_path, self.export_dir)
        try:
            analysis.run(plugins)
            self.database.report = analysis.report
        finally:
            rmtree(temp_path)

    def _save_results(self, node_cache):
        print "[HAROS] Saving analysis results..."
        if self.export_viz:
            viz.install(self.viz_dir, self.run_from_source)
        self._ensure_dir(self.data_dir)
        self._ensure_dir(self.current_dir)
        self.database.save_state(os.path.join(self.current_dir, "haros.db"))
        self.log.debug("Exporting on-memory data manager.")
        self._prepare_project()
        exporter = JsonExporter()
        self._export_project_data(exporter)
        exporter.export_projects(self.data_dir, (self.database.project,),
                                 overwrite = False)
        if self.parse_nodes and self.use_cache:
            for node in self.database.nodes.itervalues():
                node_cache[node.node_name] = node.to_JSON_object()
            parse_cache = os.path.join(self.root, "parse_cache.json")
            try:
                with open(parse_cache, "w") as f:
                    json.dump(node_cache, f)
            except IOError as e:
                self.log.warning("Could not save parsing cache: %s", e)


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
    def __init__(self, haros_dir, data_dir, export_viz, project,
                 log = None, run_from_source = False):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
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
        print "[HAROS] Exporting analysis results..."
        self._prepare_directory()
        exporter = JsonExporter()
        for project in self._project_list():
            self.project = project
            self._prepare_project()
            if self._load_database():
                self._save_database()
                self._export_project_data(exporter)
        exporter.export_projects(self.data_dir, self.project_data_list)
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
        self.database = HarosDatabase.load_state(self.haros_db)
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
    def __init__(self, haros_dir, server_dir, host_str, headless, log = None,
                 run_from_source = False):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
        self.server_dir = server_dir
        self.host = host_str
        self.headless = headless

    def run(self):
        return viz.serve(self.server_dir, self.host, headless = self.headless)


###############################################################################
#   HAROS Main Function
###############################################################################

def main(argv = None, source_runner = False):
    launcher = HarosLauncher(run_from_source = source_runner)
    if launcher.launch(argv = argv):
        return 0
    return 1
