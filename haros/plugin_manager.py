
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

import importlib
import logging
import os
import sys
import yaml

_log = logging.getLogger(__name__)

class MalformedManifestError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class AnalysisInterface(object):
    def __init__(self, module, languages):
        self.module     = module
        self.languages  = set(languages)
        self.state      = None
        self.f_analysis = hasattr(module, "file_analysis")
        self.p_analysis = hasattr(module, "package_analysis")
        self.r_analysis = hasattr(module, "repository_analysis")

    def analyse_file(self, iface, scope):
        _log.debug("Plugin.analyse_file: " + scope.id)
        if self.f_analysis and scope.language in self.languages:
            _log.debug("Calling module.file_analysis")
            self.module.file_analysis(iface, scope)

    def analyse_package(self, iface, scope):
        _log.debug("Plugin.analyse_package: " + scope.id)
        if self.p_analysis:
            _log.debug("Calling module.package_analysis")
            self.module.package_analysis(iface, scope)

    def analyse_repository(self, iface, scope):
        # Note: path may be None if the repository was not downloaded
        # TODO: what action to take on None path
        _log.debug("Plugin.analyse_repository: " + scope.id)
        if self.r_analysis:
            _log.debug("Calling module.repository_analysis")
            self.module.repository_analysis(iface, scope)

    def pre_analysis(self):
        _log.debug("Plugin.pre_analysis")
        try:
            _log.debug("Calling module.pre_analysis")
            self.state = self.module.pre_analysis()
        except AttributeError as e:
            _log.debug("Module does not perform initialisation.")

    def post_analysis(self, iface):
        _log.debug("Plugin.post_analysis")
        try:
            _log.debug("Calling module.post_analysis")
            self.module.post_analysis(iface)
        except AttributeError as e:
            _log.debug("Module does not perform finalisation.")


class ProcessingInterface(object):
    def __init__(self, module):
        self.module         = module
        self.state          = None
        self.f_violations   = hasattr(module, "process_file_violation")
        self.f_metrics      = hasattr(module, "process_file_metric")
        self.p_violations   = hasattr(module, "process_package_violation")
        self.p_metrics      = hasattr(module, "process_package_metric")
        self.r_violations   = hasattr(module, "process_repository_violation")
        self.r_metrics      = hasattr(module, "process_repository_metric")

    def process_file(self, iface, scope, violations, metrics):
        # Receives a copy of the issues because the actual lists may change
        _log.debug("Plugin.process_file: " + scope.id)
        if self.f_violations:
            _log.debug("Calling module.process_file_violation")
            for datum in violations:
                self.module.process_file_violation(iface, datum)
        if self.f_metrics:
            _log.debug("Calling module.process_file_metric")
            for datum in metrics:
                self.module.process_file_metric(iface, datum)

    def process_package(self, iface, scope, violations, metrics):
        # Receives a copy of the issues because the actual lists may change
        _log.debug("Plugin.process_package: " + scope.id)
        if self.p_violations:
            _log.debug("Calling module.process_package_violation")
            for datum in violations:
                self.module.process_package_violation(iface, datum)
        if self.p_metrics:
            _log.debug("Calling module.process_package_metric")
            for datum in metrics:
                self.module.process_package_metric(iface, datum)

    def process_repository(self, iface, scope, violations, metrics):
        # Receives a copy of the issues because the actual lists may change
        _log.debug("Plugin.process_repository: " + scope.id)
        if self.r_violations:
            _log.debug("Calling module.process_repository_violation")
            for datum in violations:
                self.module.process_repository_violation(iface, datum)
        if self.r_metrics:
            _log.debug("Calling module.process_repository_metric")
            for datum in metrics:
                self.module.process_repository_metric(iface, datum)

    def pre_process(self):
        _log.debug("Plugin.pre_process")
        try:
            _log.debug("Calling module.pre_process")
            self.state = self.module.pre_process()
        except AttributeError as e:
            _log.debug("Module does not perform initialisation.")

    def post_process(self, iface):
        _log.debug("Plugin.post_process")
        try:
            _log.debug("Calling module.post_process")
            self.module.post_process(iface)
        except AttributeError as e:
            _log.debug("Module does not perform finalisation.")


class ExportInterface(object):
    pass


class Plugin(object):
    def __init__(self, name, dir):
        self.name       = name
        self.version    = "0.1"
        self.path       = dir
        self.rules      = None
        self.metrics    = None
        self.analysis   = None
        self.process    = None
        self.export     = None
        self.tmp_path   = None

    def load(self, common_rules = None, common_metrics = None):
        _log.debug("Plugin.load")
        manifest = os.path.join(self.path, "plugin.yaml")
        _log.debug("Plugin manifest at " + manifest)
        with open(manifest, "r") as openfile:
            manifest = yaml.load(openfile)
        if not "version" in manifest or not "name" in manifest or \
                manifest["name"] != self.name:
            raise MalformedManifestError("Malformed plugin manifest: " + \
                    self.name)
        self.version    = manifest["version"]
        self.rules      = manifest.get("rules", {})
        self.metrics    = manifest.get("metrics", {})
        _log.debug("Loaded %s [%s]", self.name, self.version)
        if common_rules:
            rm = [id for id in self.rules if id in common_rules]
            for id in rm:
                _log.warning("Plugin %s cannot override %s", self.name, id)
                del self.rules[id]
        if common_metrics:
            rm = [id for id in self.metrics if id in common_metrics]
            for id in rm:
                _log.warning("Plugin %s cannot override %s", self.name, id)
                del self.metrics[id]
        _log.info("Loading plugin script.")
        _log.debug("Plugin script at %s", self.path)
        module = importlib.import_module(self.name + ".plugin",
                                         package = self.name)
        languages = manifest.get("languages", [])
        self.analysis = AnalysisInterface(module, languages)
        self.process = ProcessingInterface(module)
        self.export = ExportInterface()



def load_plugins(root, whitelist = None, blacklist = None):
    _log.debug("load_plugins(%s, %s, %s)", root, whitelist, blacklist)
    plugins = []
    filter = []
    mode = 0
    if whitelist:
        filter = whitelist
        mode = 1
    elif blacklist:
        filter = blacklist
        mode = -1
    sys.path.insert(0, root)
    for item in os.listdir(root):
        if (mode > 0 and not item in filter) or (mode < 0 and item in filter) \
                or item.startswith(".") or item == "haros_util":
            continue
        d = os.path.join(root, item)
        _log.debug("Checking for plugins at %s", d)
        if os.path.isdir(d) \
                and os.path.isfile(os.path.join(d, "plugin.yaml")) \
                and os.path.isfile(os.path.join(d, "plugin.py")):
            plugin = Plugin(item, d)
            try:
                plugin.load()
                plugins.append(plugin)
            except MalformedManifestError as e:
                _log.warning(e.value)
            except ImportError as e:
                _log.error("Failed to import %s; %s", item, e)
    sys.path.pop(0)
    return plugins
