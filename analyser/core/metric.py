from yaml import load

class Metric:
    def __init__(self, metric_id, name, desc, minv=None, maxv=None):
        self.id             = metric_id
        self.name           = name
        self.description    = desc
        self.minimum        = minv
        self.maximum        = maxv

    def asTuple(self):
        return (self.id, self.name, self.description)


def load_metrics_from_file(metrics_file):
    with open(metrics_file, "r") as mf:
        metrics_data = load(mf)
    metrics = []
    for metric in metrics_data:
        metrics.append(Metric(int(metric["id"]), metric["name"],
                metric["description"]))
    return metrics

