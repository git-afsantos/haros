
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
#   -C changes the CWD before running
#   haros init
#       initialises the data directory
#   haros analyse [args]
#       runs update, analysis and export
#       -r  also register and analyse repositories
#       -p  package/project filter
#       -w  whitelist plugins
#       -b  blacklist plugins
#       -t  export results to target directory
#       -a  analysis history to import
#   haros export [args]
#       runs export only
#       -v export viz files too
#       -p project name to export
#   haros viz [args]
#       runs visualiser only
#       -s host
#       -t target dir (input)
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
            if not os.path.isdir(HAROS_DIR):
                print "[HAROS] It seems this is a first run."
                self.command_init(args)
            if args.dir:
                os.chdir(args.dir)
            self.log.info("Executing selected command.")
            return args.command(args)
        except RuntimeError as err:
            self.log.error(str(err))
            return False
        finally:
            os.chdir(original_path)

    def command_init(self, args):
        if not self.initialised:
            init = HarosInitRunner(self.HAROS_DIR, log = self.log,
                                   run_from_source = self.run_from_source)
            if not init.run():
                return False
            self.initialised = True
        return True

    def command_full(self, args):
        pass

    def command_analyse(self, args):
        if not os.path.isdir(args.target_dir):
            raise ValueError("Not a directory: " + args.target_dir)
        if not os.path.isfile(args.pkg_filter):
            raise ValueError("Not a file: " + args.pkg_filter)
        analyse = HarosAnalyseRunner(args.pkg_filter, args.target_dir, log = self.log,
                                     run_from_source = self.run_from_source)
        return analyse.run()

    def command_export(self, args):
        pass

    def command_viz(self, args):
        pass

    def parse_arguments(self, argv = None):
        parser = ArgumentParser(prog = "haros",
                                description = "ROS quality assurance.")
        parser.add_argument("--debug", action = "store_true",
                            help = "set debug logging")
        parser.add_argument("-C", dest = "dir",
                            help = ("change current directory "
                                    "to DIR before running"))
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
        parser.add_argument("-t", "--target-dir", default = self.VIZ_DIR,
                            help = "export reports to target DIR")
        parser.add_argument("-a", "--analysis-db",
                            help = "import analysis history database")
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
        parser.add_argument("-t", "--target-dir", default = self.VIZ_DIR,
                            help = "export reports to target DIR")
        parser.add_argument("-a", "--analysis-db",
                            help = "import analysis history database")
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
        parser.add_argument("target_dir", metavar = "dir",
                           help = "where to export data")
        parser.set_defaults(command = self.command_export)

    def _viz_parser(self, parser):
        parser.add_argument("-t", "--target-dir", default = self.VIZ_DIR,
                            help = "served data directory")
        parser.add_argument("-s", "--server-host", dest = "host",
                            default = "localhost:8080",
                            help = ("visualisation host "
                                    "(default: \"localhost:8080\")"))
        parser.set_defaults(command = self.command_viz)


class HarosRunner(object):
    """This is a base class for the specific commands that HAROS provides."""

    def __init__(self, log = None, run_from_source = False):
        self.log = log or logging.getLogger()
        self.run_from_source = run_from_source

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

    def __init__(self, haros_dir, log = None, run_from_source = False):
        HarosRunner.__init__(self, log = log, run_from_source = run_from_source)
        self.root = haros_dir
        self.plugin_dir = os.path.join(haros_dir, "plugins")
        self.viz_dir = os.path.join(haros_dir, "viz")

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
    def __init__(self, target_dir, host_str, log = None, run_from_source = False):
        HarosRunner.__init__(self, log = log, run_from_source = run_from_source)
        self.root = target_dir
        self.host = host_str

    def run(self):
        return HarosAnalyseRunner.run(self) and HarosVizRunner.run(self)


class HarosAnalyseRunner(HarosRunner):
    def __init__(self, pkg_filter, target_dir, log = None, run_from_source = False):
        HarosRunner.__init__(self, log = log, run_from_source = run_from_source)
        self.projects = [pkg_filter]
        self.target_dir = target_dir

    def run(self):
        print "[HAROS] Indexing source code..."
        self.log.debug("Creating new data manager.")
        dataman = DataManager()
        self.log.debug("Package index file %s", self.projects[0])
        dataman.index_source(self.projects[0], REPOSITORY_DIR, args.use_repos)
        if not dataman.packages:
            self.log.warning("There are no packages to analyse.")
            return False
        print "[HAROS] Loading common definitions..."
        if args.source_runner:
            path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "definitions.yaml"))
        else:
            path = resource_filename(Requirement.parse("haros"),
                                     "haros/definitions.yaml")
        dataman.load_definitions(path)
        print "[HAROS] Loading plugins..."
        plugins = plugman.load_plugins(PLUGIN_DIR, args.whitelist, args.blacklist)
        if not plugins:
            self.log.warning("There are no analysis plugins.")
            return False
        for plugin in plugins:
            dataman.extend_definitions(plugin.name, plugin.rules, plugin.metrics)
        print "[HAROS] Running analysis..."
        _empty_dir(EXPORT_DIR)
        db_path = os.path.join(PROJECTS_DIR, dataman.project.name)
        args.analysis_db = args.analysis_db or os.path.join(db_path, "analysis.db")
        if os.path.isfile(args.analysis_db):
            anaman = AnalysisManager.load_state(args.analysis_db)
        else:
            anaman = AnalysisManager()
        temppath = tempfile.mkdtemp()
        anaman.run_analysis_and_processing(temppath, plugins, dataman, EXPORT_DIR)
        rmtree(temppath)
        print "[HAROS] Saving analysis results..."
        # TODO this needs refactoring
        if not os.path.isdir(db_path):
            self.log.info("Creating %s...", db_path)
            os.mkdir(db_path)
        dataman.save_state(os.path.join(db_path, "haros.db"))
        anaman.save_state(args.analysis_db)
        index_file = os.path.join(args.target_dir, "index.html")
        args.export_viz = (args.target_dir != VIZ_DIR
                           and not os.path.isfile(index_file))
        command_export(args, dataman, anaman)
    
    
    
        return True


class HarosExportRunner(HarosRunner):
    def __init__(self, target_dir, log = None, run_from_source = False):
        HarosRunner.__init__(self, log = log, run_from_source = run_from_source)
        self.target_dir = target_dir

    def run(self):
        return True


class HarosVizRunner(HarosRunner):
    def __init__(self, server_dir, host_str, log = None,
                 run_from_source = False):
        HarosRunner.__init__(self, log = log, run_from_source = run_from_source)
        self.server_dir = server_dir
        self.host = host_str

    def run(self):
        return viz.serve(self.server_dir, self.host)




def _empty_dir(dir_path):
    self.log.debug("Emptying directory %s", dir_path)
    for f in os.listdir(dir_path):
        path = os.path.join(dir_path, f)
        if os.path.isfile(path):
            self.log.debug("Removing file %s", path)
            os.unlink(path)

def command_analyse(args):
    _check_haros_directory()
    if not os.path.isdir(args.target_dir):
        self.log.error("%s is not a directory!", args.target_dir)
        return False
    self.log.debug("Creating new data manager.")
    dataman = DataManager()
    print "[HAROS] Indexing source code..."
    path = args.pkg_filter \
           if args.pkg_filter and os.path.isfile(args.pkg_filter) \
           else os.path.join(HAROS_DIR, "index.yaml")
    self.log.debug("Package index file %s", path)
    dataman.index_source(path, REPOSITORY_DIR, args.use_repos)
    if not dataman.packages:
        self.log.warning("There are no packages to analyse.")
        return False
    print "[HAROS] Loading common definitions..."
    if args.source_runner:
        path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                               "definitions.yaml"))
    else:
        path = resource_filename(Requirement.parse("haros"),
                                 "haros/definitions.yaml")
    dataman.load_definitions(path)
    print "[HAROS] Loading plugins..."
    plugins = plugman.load_plugins(PLUGIN_DIR, args.whitelist, args.blacklist)
    if not plugins:
        self.log.warning("There are no analysis plugins.")
        return False
    for plugin in plugins:
        dataman.extend_definitions(plugin.name, plugin.rules, plugin.metrics)
    print "[HAROS] Running analysis..."
    _empty_dir(EXPORT_DIR)
    db_path = os.path.join(PROJECTS_DIR, dataman.project.name)
    args.analysis_db = args.analysis_db or os.path.join(db_path, "analysis.db")
    if os.path.isfile(args.analysis_db):
        anaman = AnalysisManager.load_state(args.analysis_db)
    else:
        anaman = AnalysisManager()
    temppath = tempfile.mkdtemp()
    anaman.run_analysis_and_processing(temppath, plugins, dataman, EXPORT_DIR)
    rmtree(temppath)
    print "[HAROS] Saving analysis results..."
    # TODO this needs refactoring
    if not os.path.isdir(db_path):
        self.log.info("Creating %s...", db_path)
        os.mkdir(db_path)
    dataman.save_state(os.path.join(db_path, "haros.db"))
    anaman.save_state(args.analysis_db)
    index_file = os.path.join(args.target_dir, "index.html")
    args.export_viz = (args.target_dir != VIZ_DIR
                       and not os.path.isfile(index_file))
    command_export(args, dataman, anaman)
    return True


def command_export(args, dataman = None, anaman = None):
    assert (dataman is None) == (anaman is None)
    _check_haros_directory()
    if not os.path.isdir(args.target_dir):
        self.log.error("%s is not a directory!", args.target_dir)
        return False
    print "[HAROS] Exporting analysis results..."
    if args.export_viz:
        viz.install(args.target_dir, args.source_runner)
    viz_data_dir = os.path.join(args.target_dir, "data")
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
            json_path = os.path.join(args.target_dir, "json")
            if not os.path.exists(json_path):
                self.log.info("Creating directory %s", json_path)
                os.mkdir(json_path)
            expoman.export_projects(json_path, [dataman.project])
        # csv_path = os.path.join(args.target_dir, "csv")
        db_path = os.path.join(args.target_dir, "haros.db")
        ana_path = os.path.join(args.target_dir, "analysis.db")
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
