import imp
import logging
import os
import yaml

_log = logging.getLogger(__name__)

class MalformedManifestError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class Plugin:
    def __init__(self, name, dir):
        self.name       = name
        self.version    = "0.1"
        self.rules      = None
        self.metrics    = None
        self.module     = None
        self.path       = dir
        self.scopes     = set()
        self.languages  = None

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
        self.version = manifest["version"]
        self.rules = manifest.get("rules", {})
        self.metrics = manifest.get("metrics", {})
        self.languages = set(manifest.get("languages", []))
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
        self.module = imp.load_source(self.name,
                                      os.path.join(self.path, "plugin.py"))
        if hasattr(self.module, "file_analysis"):
            self.scopes.add("file")
        if hasattr(self.module, "package_analysis"):
            self.scopes.add("package")
        if hasattr(self.module, "repository_analysis"):
            self.scopes.add("repository")
        _log.debug("Plugin scopes %s", self.scopes)

    def analyse_file(self, scope, iface):
        _log.debug("Plugin.analyse_file: " + scope.id)
        if scope.language in self.languages:
            try:
                _log.debug("Calling module.file_analysis")
                self.module.file_analysis(iface, scope)
            except AttributeError as e:
                _log.debug("Unexpected AttributeError")

    def analyse_package(self, scope, iface):
        _log.debug("Plugin.analyse_package: " + scope.id)
        try:
            _log.debug("Calling module.package_analysis")
            self.module.package_analysis(iface, scope)
        except AttributeError as e:
            _log.debug("Unexpected AttributeError")

    def analyse_repository(self, scope, iface):
        # Note: path may be None if the repo wasn't downloaded
        # TODO: what action to take on None path
        _log.debug("Plugin.analyse_repository: " + scope.id)
        try:
            _log.debug("Calling module.repository_analysis")
            self.module.repository_analysis(iface, scope)
        except AttributeError as e:
            _log.debug("Unexpected AttributeError")



# Returns {Name -> (Manifest, Module)}
def load_plugins(root, whitelist = None, blacklist = None):
    _log.debug("load_plugins(%s, %s, %s)", root, whitelist, blacklist)
    plugins = {}
    filter = []
    mode = 0
    if whitelist:
        filter = whitelist
        mode = 1
    elif blacklist:
        filter = blacklist
        mode = -1
    for item in os.listdir(root):
        if (mode > 0 and not item in filter) \
                or (mode < 0 and item in filter) \
                or item.startswith("."):
            continue
        d = os.path.join(root, item)
        _log.debug("Checking for plugins at %s", d)
        if os.path.isdir(d) \
                and os.path.isfile(os.path.join(d, "plugin.yaml")) \
                and os.path.isfile(os.path.join(d, "plugin.py")):
            plugin = Plugin(item, d)
            try:
                plugin.load()
                plugins[item] = plugin
            except MalformedManifestError as e:
                _log.warning(e.value)
            except ImportError as e:
                _log.error("Failed to import plugin " + item)
    return plugins
