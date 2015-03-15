
from datamanage import db_manager as dbm

class PluginContext:
    def __init__(self, db):
        self.db = db
        self.metric_ids = None
        self.package_info = None
        self.package_buffer = []
        self.file_info = None
        self.file_buffer = []

    def getRoot(self):
        return os.path.join(os.path.expanduser("~"), "ros", "repos")

    def getMetricIds(self):
        if self.metric_ids is None:
            self.metric_ids = self.db.get("Metrics", ["id", "name"])
        return self.metric_ids

    def getPackageInfo(self):
        if self.package_info is None:
            self.package_info = self.db.get("Packages", ["id", "name", "path"])
        return self.package_info

    def getFileInfo(self):
        if self.file_info is None:
            self.file_info = self.db.get("Files", ["id", "name", "path"])
        return self.file_info

    def writePackageMetric(self, package, metric, value):
        self.package_buffer.append((package, metric, value))
        if len(self.package_buffer) == 100:
            self._commit()

    def writeFileMetric(self, file_id, metric, value):
        self.file_buffer.append((file_id, metric, value))
        if len(self.file_buffer) == 100:
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



def analyse_metrics(plugin_list):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    ctx = PluginContext(db)
    for p in plugin_list:
        p.plugin_run(ctx)
        ctx._commit()
    db.disconnect()

