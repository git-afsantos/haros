import imp
import os
import yaml


class MalformedManifestError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class Plugin:
    def __init__(self, name, dir):
        self.name = name
        self.version = "0.1"
        self.rules = None
        self.metrics = None
        self.module = None
        self.path = dir
        self.scopes = set()

    def load(self, common_rules = None, common_metrics = None):
        manifest = os.path.join(self.path, "plugin.yaml")
        with open(manifest, "r") as openfile:
            manifest = yaml.load(openfile)
        if not "version" in manifest or not "name" in manifest or \
                manifest["name"] != self.name:
            raise MalformedManifestError("Malformed plugin manifest: " + \
                    self.name)
        self.version = manifest["version"]
        self.rules = manifest.get("rules", {})
        self.metrics = manifest.get("metrics", {})
        if common_rules:
            rm = [id for id in self.rules if id in common_rules]
            for id in rm:
                print "Plugin", self.name, "cannot override rule", id
                del self.rules[id]
        if common_metrics:
            rm = [id for id in self.metrics if id in common_metrics]
            for id in rm:
                print "Plugin", self.name, "cannot override metric", id
                del self.metrics[id]
        self.module = imp.load_source(self.name, \
                os.path.join(self.path, "plugin.py"))
        if hasattr(self.module, "file_analysis"):
            self.scopes.add("file")
        if hasattr(self.module, "package_analysis"):
            self.scopes.add("package")
        if hasattr(self.module, "repository_analysis"):
            self.scopes.add("repository")

    def analyse_file(self, scope, iface):
        try:
            self.module.file_analysis(iface, scope.get_path())
        except AttributeError as e:
            pass

    def analyse_package(self, scope, iface):
        try:
            self.module.package_analysis(iface, scope.path)
        except AttributeError as e:
            pass

    def analyse_repository(self, scope, iface):
        # Note: path may be None if the repo wasn't downloaded
        # TODO: what action to take on None path
        try:
            self.module.repository_analysis(iface, scope.path)
        except AttributeError as e:
            pass



# Returns {Name -> (Manifest, Module)}
def load_plugins(root, whitelist = None, blacklist = None):
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
        if (mode > 0 and not item in filter) or (mode < 0 and item in filter):
            continue
        d = os.path.join(root, item)
        if os.path.isdir(d) and
                os.path.isfile(os.path.join(d, "plugin.yaml")) and \
                os.path.isfile(os.path.join(d, "plugin.py")):
            plugin = Plugin(item, d)
            try:
                plugin.load()
                plugins[item] = plugin
            except (MalformedManifestError, ImportError) as e:
                print "Failed to load plugin: " + item
    return plugins
