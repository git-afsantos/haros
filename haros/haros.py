
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

# start with init
# init creates the default data dir
# viz is copied to init dir

# analysis grabs previous db and index from data dir
# analysis may accept import option to use another db

# export receives a dir where it will generate the export files
# export also generates files in the data dir
# export uses db from analysis step or loads it from data dir

# viz uses data from the data dir

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


from argparse import ArgumentParser
import logging
import os
import subprocess
import tempfile

from shutil import copyfile, rmtree
from pkg_resources import Requirement, resource_filename

from .data_manager import DataManager
from . import plugin_manager as plugman
from .analysis_manager import AnalysisManager
from . import export_manager as expoman
from . import visualiser as viz


class HarosLauncher(object):
    """This class contains the necessary methods to launch HAROS.
        It is responsible for initialising directories and data
        structures, as well as parsing program arguments.
    """

    HAROS_DIR       = os.path.join(os.path.expanduser("~"), ".haros")
    DEFAULT_INDEX   = os.path.join(HAROS_DIR, "index.yaml")
    LOG_PATH        = os.path.join(HAROS_DIR, "log.txt")
    REPOSITORY_DIR  = os.path.join(HAROS_DIR, "repositories")
    EXPORT_DIR      = os.path.join(HAROS_DIR, "export")
    PLUGIN_DIR      = os.path.join(HAROS_DIR, "plugins")
    VIZ_DIR         = os.path.join(HAROS_DIR, "viz")
    PROJECTS_DIR    = os.path.join(HAROS_DIR, "projects")
    DEFAULT_PROJECT = os.path.join(PROJECTS_DIR, "default")
    DB_PATH         = os.path.join(DEFAULT_PROJECT, "haros.db")
    ANALYSIS_PATH   = os.path.join(DEFAULT_PROJECT, "analysis.db")

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
                self.command_init(args)
            if args.cwd:
                os.chdir(args.cwd)
            self.log.info("Executing selected command.")
            return args.command(args)
        except RuntimeError as err:
            self.log.error(str(err))
            return False
        finally:
            os.chdir(original_path)

    def command_init(self, args):
        if not self.initialised:
            init = HarosInitRunner(self.HAROS_DIR, self.log,
                                   self.run_from_source)
            if not init.run():
                return False
            self.initialised = True
        return True

    def command_full(self, args):
        pass

    def command_analyse(self, args):
        if not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        if not os.path.isfile(args.pkg_filter):
            raise ValueError("Not a file: " + args.pkg_filter)
        # TODO fix args
        analyse = HarosAnalyseRunner(args.pkg_filter, args.data_dir, log = self.log,
                                     run_from_source = self.run_from_source)
        return analyse.run()

    def command_export(self, args):
        if not os.path.isdir(args.data_dir):
            raise ValueError("Not a directory: " + args.data_dir)
        export = HarosExportRunner()    # TODO fix args
        return export.run()

    def command_viz(self, args):
        pass

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
        # TODO try to remove these dumb `dest`
        parser.add_argument("-r", "--repositories",
                            dest = "use_repos", action = "store_true",
                            help = "use repositories")
        parser.add_argument("-s", "--server-host", dest = "host",
                            default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.add_argument("-p", "--package-index", dest = "pkg_filter",
                            default = self.DEFAULT_INDEX,
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
        group = parser.add_mutually_exclusive_group()
        group.add_argument("-w", "--whitelist", nargs = "*",
                           help = "execute only these plugins")
        group.add_argument("-b", "--blacklist", nargs = "*",
                           help = "skip these plugins")
        parser.set_defaults(command = self.command_full)

    def _analyse_parser(self, parser):
        parser.add_argument("-r", "--repositories", dest = "use_repos",
                            action = "store_true", help = "use repositories")
        parser.add_argument("-p", "--package-index", dest = "pkg_filter",
                            default = self.DEFAULT_INDEX,
                            help = ("package index file (default: "
                                    "packages below current dir)"))
        parser.add_argument("-d", "--data-dir",
                            help = "load/export using the given directory")
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
        parser.add_argument("-s", "--server-host", dest = "host",
                            default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.set_defaults(command = self.command_viz)


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
                if os.path.exists(new_path) and not os.path.isdir(new_path):
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

    def _ensure_dir(self, dir_path):
        if not os.path.isdir(dir_path):
            if os.path.isfile(dir_path):
                raise RuntimeError("Could not create dir: " + dir_path)
            os.makedirs(dir_path)



class HarosInitRunner(HarosRunner):
    PLUGIN_REPOSITORY = "https://github.com/git-afsantos/haros_plugins.git"

    DIR_STRUCTURE = {
        "index.yaml": "%YAML 1.1\n---\npackages: []\n",
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
        viz.install(self.viz_dir, self.run_from_source)
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


class HarosFullRunner(HarosRunner):
    def __init__(self, haros_dir, data_dir, host_str, log = None, run_from_source = False):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
        self.root = data_dir
        self.host = host_str

    def run(self):
        return HarosAnalyseRunner.run(self) and HarosVizRunner.run(self)


class HarosAnalyseRunner(HarosRunner):
    def __init__(self, haros_dir, pkg_filter, data_dir, whitelist, blacklist,
                 log = None, run_from_source = False, use_repos = False):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
        self.project_file = pkg_filter
        self.use_repos = use_repos
        self.whitelist = whitelist
        self.blacklist = blacklist
        self.project = None
        self.dataman = None
        self.anaman = None
        self.analysis_db = None
        self.current_proj_dir = None
        if data_dir:
            self.export_viz = True
            self.data_dir = os.path.join(data_dir, "data")
            self.io_projects_dir = os.path.join(data_dir, "projects")
        else:
            self.export_viz = False
            self.data_dir = os.path.join(self.viz_dir, "data")
            self.io_projects_dir = self.project_dir

    def run(self):
        self.log.debug("Creating new data manager.")
        self.dataman = DataManager()
        plugins = self._load_definitions_and_plugins()
        self._index_source()
        self._analyse()
        
        
        print "[HAROS] Saving analysis results..."
        # TODO this needs refactoring
        if not os.path.isdir(db_path):
            self.log.info("Creating %s...", db_path)
            os.mkdir(db_path)
        dataman.save_state(os.path.join(db_path, "haros.db"))
        anaman.save_state(args.analysis_db)
        index_file = os.path.join(args.data_dir, "index.html")
        args.export_viz = (args.data_dir != VIZ_DIR
                           and not os.path.isfile(index_file))
        command_export(args, dataman, anaman)
    
    
    
        return HarosExportRunner.run(self)

    @property
    def definitions_file(self):
        if self.run_from_source:
            return os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "definitions.yaml"))
        return resource_filename(Requirement.parse("haros"),
                                 "haros/definitions.yaml")

    def _index_source(self):
        print "[HAROS] Indexing source code..."
        self.log.debug("Package index file %s", self.project_file)
        self.dataman.index_source(self.project_file,
                                  self.repo_dir, self.use_repos)
        if not self.dataman.packages:
            raise RuntimeError("There are no packages to analyse.")

    def _load_definitions_and_plugins(self):
        print "[HAROS] Loading common definitions..."
        self.dataman.load_definitions(self.definitions_file)
        print "[HAROS] Loading plugins..."
        plugins = plugman.load_plugins(self.plugin_dir,
                                       self.whitelist, self.blacklist)
        if not plugins:
            raise RuntimeError("There are no analysis plugins.")
        for plugin in plugins:
            self.dataman.extend_definitions(plugin.name,
                                            plugin.rules, plugin.metrics)
        return plugins

    def _load_analysis_manager(self, name):
        self.project = name
        self.current_proj_dir = os.path.join(self.io_projects_dir, name)
        self.analysis_db = os.path.join(self.current_proj_dir, "analysis.db")
        if os.path.isfile(self.analysis_db):
            self.log.info("Loading previous database: " + self.analysis_db)
            self.anaman = AnalysisManager.load_state(args.analysis_db)
        else:
            self.log.info("Creating new analysis manager.")
            self.anaman = AnalysisManager()

    def _analyse(self, plugins):
        print "[HAROS] Running analysis..."
        self._empty_dir(self.export_dir)
        self._load_analysis_manager(self.dataman.project.name)
        temppath = tempfile.mkdtemp()
        self.anaman.run_analysis_and_processing(temppath, plugins,
                                                self.dataman, self.export_dir)
        rmtree(temppath)


class HarosExportRunner(HarosRunner):
    def __init__(self, haros_dir, data_dir, export_viz, project,
                 dataman = None, anaman = None, log = None,
                 run_from_source = False):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
        self.project = project
        self.data_dir = data_dir
        self.export_viz = export_viz
        self.dataman = dataman
        self.anaman = anaman
        if export_viz:
            self.io_projects_dir = os.path.join(data_dir, "projects")
        else:
            self.io_projects_dir = data_dir

    def run(self):
        assert (self.dataman is None) == (self.anaman is None)
        
        # 1. export from memory without viz (default dir)
            # .haros/projects/
            # .haros/viz/data/
        # 2. export from memory with viz (custom dir)
            # dir/projects/
            # dir/data/
        # 3. export without viz
            # dir/projects/
            # dir/projects/
        # 4. export with viz
            # dir/projects/
            # dir/data/
        
        print "[HAROS] Exporting analysis results..."
        if self.export_viz:
            viz.install(self.data_dir, self.run_from_source)
        viz_data_dir = os.path.join(args.data_dir, "data")
        if dataman:
            self.log.debug("Exporting on-memory data manager.")
            json_path   = os.path.join(viz_data_dir, dataman.project.name)
            db_path     = None
            ana_path    = None
            expoman.export_projects(viz_data_dir, [dataman.project])
        else:
            self.log.debug("Exporting data manager from file.")
            db_src = os.path.join(PROJECTS_DIR, args.project)
            db_file = os.path.join(db_src, "haros.db")
            if os.path.isfile(db_file):
                dataman = DataManager.load_state(db_file)
            else:
                self.log.warning("There is no analysis data to export.")
                return False
            ana_file = os.path.join(db_src, "analysis.db")
            if os.path.isfile(ana_file):
                anaman = AnalysisManager.load_state(ana_file)
            else:
                self.log.warning("There is no analysis data to export.")
                return False
            if args.export_viz:
                json_path = os.path.join(viz_data_dir, dataman.project.name)
                expoman.export_projects(viz_data_dir, [dataman.project])
            else:
                json_path = os.path.join(args.data_dir, "json")
                if not os.path.exists(json_path):
                    self.log.info("Creating directory %s", json_path)
                    os.mkdir(json_path)
                expoman.export_projects(json_path, [dataman.project])
            db_path = os.path.join(args.data_dir, "haros.db")
            ana_path = os.path.join(args.data_dir, "analysis.db")
        if not os.path.exists(json_path):
            self.log.info("Creating directory %s", json_path)
            os.mkdir(json_path)
        expoman.export_packages(json_path, dataman.packages)
        expoman.export_rules(json_path, dataman.rules)
        expoman.export_metrics(json_path, dataman.metrics)
        expoman.export_summary(json_path, anaman)
        path = os.path.join(json_path, "compliance")
        if not os.path.exists(path):
            self.log.info("Creating directory %s", path)
            os.mkdir(path)
        else:
            _empty_dir(path)
        expoman.export_violations(path, dataman.packages)
        path = os.path.join(json_path, "metrics")
        if not os.path.exists(path):
            self.log.info("Creating directory %s", path)
            os.mkdir(path)
        else:
            _empty_dir(path)
        expoman.export_measurements(path, dataman.packages)
        path = os.path.join(json_path, "models")
        if not os.path.exists(path):
            self.log.info("Creating directory %s", path)
            os.mkdir(path)
        else:
            _empty_dir(path)
        expoman.export_configurations(path, dataman.packages)
        if db_path:
            self.log.debug("Copying data DB from %s to %s", db_file, db_path)
            copyfile(db_file, db_path)
        if ana_path:
            self.log.debug("Copying analysis DB from %s to %s", ana_file, ana_path)
            copyfile(ana_file, ana_path)
        return True


class HarosVizRunner(HarosRunner):
    def __init__(self, haros_dir, server_dir, host_str, log = None,
                 run_from_source = False):
        HarosRunner.__init__(self, haros_dir, log, run_from_source)
        self.server_dir = server_dir
        self.host = host_str

    def run(self):
        return viz.serve(self.server_dir, self.host)




def command_export(args, dataman = None, anaman = None):
    assert (dataman is None) == (anaman is None)
    _check_haros_directory()
    if not os.path.isdir(args.data_dir):
        self.log.error("%s is not a directory!", args.data_dir)
        return False
    print "[HAROS] Exporting analysis results..."
    if args.export_viz:
        viz.install(args.data_dir, args.source_runner)
    viz_data_dir = os.path.join(args.data_dir, "data")
    if dataman:
        self.log.debug("Exporting on-memory data manager.")
        json_path   = os.path.join(viz_data_dir, dataman.project.name)
        # csv_path    = EXPORT_DIR
        db_path     = None
        ana_path    = None
        expoman.export_projects(viz_data_dir, [dataman.project])
    else:
        self.log.debug("Exporting data manager from file.")
        db_src = os.path.join(PROJECTS_DIR, args.project)
        db_file = os.path.join(db_src, "haros.db")
        if os.path.isfile(db_file):
            dataman = DataManager.load_state(db_file)
        else:
            self.log.warning("There is no analysis data to export.")
            return False
        ana_file = os.path.join(db_src, "analysis.db")
        if os.path.isfile(ana_file):
            anaman = AnalysisManager.load_state(ana_file)
        else:
            self.log.warning("There is no analysis data to export.")
            return False
        if args.export_viz:
            json_path = os.path.join(viz_data_dir, dataman.project.name)
            expoman.export_projects(viz_data_dir, [dataman.project])
        else:
            json_path = os.path.join(args.data_dir, "json")
            if not os.path.exists(json_path):
                self.log.info("Creating directory %s", json_path)
                os.mkdir(json_path)
            expoman.export_projects(json_path, [dataman.project])
        # csv_path = os.path.join(args.data_dir, "csv")
        db_path = os.path.join(args.data_dir, "haros.db")
        ana_path = os.path.join(args.data_dir, "analysis.db")
    if not os.path.exists(json_path):
        self.log.info("Creating directory %s", json_path)
        os.mkdir(json_path)
    # if not os.path.exists(csv_path):
        # self.log.info("Creating directory %s", csv_path)
        # os.mkdir(csv_path)
    expoman.export_packages(json_path, dataman.packages)
    expoman.export_rules(json_path, dataman.rules)
    expoman.export_metrics(json_path, dataman.metrics)
    expoman.export_summary(json_path, anaman)
    path = os.path.join(json_path, "compliance")
    if not os.path.exists(path):
        self.log.info("Creating directory %s", path)
        os.mkdir(path)
    else:
        _empty_dir(path)
    expoman.export_violations(path, dataman.packages)
    path = os.path.join(json_path, "metrics")
    if not os.path.exists(path):
        self.log.info("Creating directory %s", path)
        os.mkdir(path)
    else:
        _empty_dir(path)
    expoman.export_measurements(path, dataman.packages)
    path = os.path.join(json_path, "models")
    if not os.path.exists(path):
        self.log.info("Creating directory %s", path)
        os.mkdir(path)
    else:
        _empty_dir(path)
    expoman.export_configurations(path, dataman.packages)
    if db_path:
        self.log.debug("Copying data DB from %s to %s", db_file, db_path)
        copyfile(db_file, db_path)
    if ana_path:
        self.log.debug("Copying analysis DB from %s to %s", ana_file, ana_path)
        copyfile(ana_file, ana_path)
    return True


def main(argv = None, source_runner = False):
    args = parse_arguments(argv, source_runner)
    if args.debug:
        logging.basicConfig(filename = LOG_PATH, filemode = "w",
                            level = logging.DEBUG)
    else:
        logging.basicConfig(level = logging.WARNING)

    original_path = os.getcwd()
    try:
        if args.dir:
            os.chdir(args.dir)
        self.log.info("Executing selected command.")
        args.func(args)
        return 0

    except RuntimeError as err:
        self.log.error(str(err))
        return 1

    finally:
        os.chdir(original_path)
