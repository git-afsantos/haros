
from datamanage import db_manager as dbm

import os
import shutil

class PluginContext:
    def __init__(self, db):
        self.db                 = db
        self.metric_ids         = None
        self.rule_info          = None
        self.package_info       = None
        self.package_buffer     = []
        self.file_info          = None
        self.file_buffer        = []
        self.class_buffer       = []
        self.function_buffer    = []
        self.compliance_buffer  = []
        self.compliance_id      = db.getNextId("Non_Compliance")

    def getRoot(self):
        return os.path.join(os.path.expanduser("~"), "ros", "repos")

    def getPath(self, relative, file_name = None):
        if relative[0] == "/":
            relative = relative[1:]
        if not file_name is None:
            relative = os.path.join(relative, file_name)
        return os.path.join(self.getRoot(), relative)

    def getMetricIds(self):
        if self.metric_ids is None:
            self.metric_ids = self.db.get("Metrics", ["id", "name"])
        return self.metric_ids

    def getRuleInfo(self):
        if self.rule_info is None:
            self.rule_info = self.db.get("Rules", ["id", "name", "scope"])
        return self.rule_info

    def getPackageInfo(self):
        if self.package_info is None:
            self.package_info = self.db.get("Packages", ["id", "name", "path"])
        return self.package_info

    def getFileInfo(self):
        if self.file_info is None:
            self.file_info = self.db.get("Files", ["id", "name", "path",
                    "package_id"])
        return self.file_info

    def writePackageMetric(self, package, metric, value):
        self.package_buffer.append((package, metric, value))
        if len(self.package_buffer) == 100:
            self._commit()

    def writeFileMetric(self, file_id, metric, value):
        self.file_buffer.append((file_id, metric, value))
        if len(self.file_buffer) == 100:
            self._commit()

    def writeClassMetric(self, file_id, cname, line, metric, value):
        self.class_buffer.append((file_id, cname, line, metric, value))
        if len(self.class_buffer) == 100:
            self._commit()

    def writeFunctionMetric(self, file_id, fname, line, metric, value):
        self.function_buffer.append((file_id, fname, line, metric, value))
        if len(self.function_buffer) == 100:
            self._commit()

    def writeNonCompliance(self, rule_id, package_id, file_id = None,
            line = None, function = None, comment = None):
        self.compliance_buffer.append((self.compliance_id, rule_id, package_id
                file_id, line, function, comment))
        self.compliance_id += 1
        if len(self.compliance_buffer) == 100:
            self._commit()


    def _commit(self):
        if len(self.package_buffer) > 0:
            self.db.insert("Package_Metrics",
                    ["package_id", "metric_id", "value"],
                    self.package_buffer)
            self.package_buffer = []
        if len(self.file_buffer) > 0:
            self.db.insert("File_Metrics",
                    ["file_id", "metric_id", "value"],
                    self.file_buffer)
            self.file_buffer = []
        if len(self.class_buffer) > 0:
            self.db.insert("File_Class_Metrics",
                    ["file_id", "class_name", "line", "metric_id", "value"],
                    self.class_buffer)
            self.class_buffer = []
        if len(self.function_buffer) > 0:
            self.db.insert("File_Function_Metrics",
                    ["file_id", "function_name", "line", "metric_id", "value"],
                    self.function_buffer)
            self.function_buffer = []
        if len(self.compliance_buffer) > 0:
            self.db.insert("Non_Compliance",
                    ["id", "rule_id", "package_id", "file_id", "line",
                        "function", "comment"],
                    self.compliance_buffer)
            self.compliance_buffer = []



def analyse_metrics(plugin_list, truncate):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    if truncate:
        db.truncate("File_Metrics")
        db.truncate("File_Class_Metrics")
        db.truncate("File_Function_Metrics")
        db.truncate("Package_Metrics")
    ctx = PluginContext(db)
    if not os.path.exists("plugin_out"):
        os.makedirs("plugin_out")
    try:
        for p in plugin_list:
            p.plugin_run(ctx)
            ctx._commit()
    finally:
        shutil.rmtree("plugin_out")
        db.disconnect()


def analyse_rules(plugin_list, truncate):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    if truncate:
        db.truncate("Non_Compliance")
    ctx = PluginContext(db)
    if not os.path.exists("plugin_out"):
        os.makedirs("plugin_out")
    try:
        for p in plugin_list:
            p.plugin_run(ctx)
            ctx._commit()
    finally:
        shutil.rmtree("plugin_out")
        db.disconnect()

