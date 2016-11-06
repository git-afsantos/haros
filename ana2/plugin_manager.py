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

    def load(self):
        manifest = os.path.join(self.path, "plugin.yaml")
        with open(manifest, "r") as openfile:
            manifest = yaml.load(openfile)
        if not "version" in manifest or not "name" in manifest or
                manifest["name"] != self.name:
            raise MalformedManifestError("Malformed plugin manifest: " +
                    self.name)
        self.version = manifest["version"]
        self.rules = manifest.get("rules", {})
        self.metrics = manifest.get("metrics", {})
        # TODO: check that rule ids do not clash with common list
        # for key, item in self.rules.iteritems():
        # for key, item in self.metrics.iteritems():
        self.module = imp.load_source(self.name,
                os.path.join(self.path, "plugin.py"))



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
                os.path.isfile(os.path.join(d, "plugin.yaml")) and
                os.path.isfile(os.path.join(d, "plugin.py")):
            plugin = Plugin(item, d)
            try:
                plugin.load()
                plugins[item] = plugin
            except (MalformedManifestError, ImportError):
                print "Failed to load plugin: " + item
    return plugins
