
import os
import subprocess

def plugin_run(ctx):
    files = ctx.getFileInfo(ext="py")
    handlers = get_metric_handlers()
    for f in files:
        file_path = ctx.getPath(f[2], file_name = f[1])
        metrics = get_metrics(file_path)
        for m in metrics:
            if m in handlers:
                handlers[m](ctx, f[3], f[0], metrics[m])



def get_metrics(file_path):
    out = os.path.join(".", "plugin_out", "output_radon.txt")
    with open(out, "w") as radon_file:
        subprocess.call(["radon", "raw", file_path], stdout = radon_file)
    if os.path.exists(out):
        return parse_radon_metrics(out)


def parse_radon_metrics(radon_file):
    with open(radon_file, "r") as rf:
        lines = [line.rstrip("\n") for line in rf]
    metrics = {}
    metrics["loc"] = int(lines[1].split(": ")[1])
    metrics["lloc"] = int(lines[2].split(": ")[1])
    metrics["sloc"] = int(lines[3].split(": ")[1])
    metrics["com"] = int(lines[4].split(": ")[1]) + int(lines[5].split(": ")[1])
    metrics["com_ratio"] = int(lines[10].split(": ")[1][:-1])
    return metrics


def get_metric_handlers():
    return {
        "com_ratio": handle_com_ratio
    }


def handle_com_ratio(ctx, package_id, file_id, value):
    if value < 20:
        ctx.writeNonCompliance(1, package_id, file_id=file_id,
            comment="Comment ratio is below 20%")
    if value > 30:
        ctx.writeNonCompliance(2, package_id, file_id=file_id,
            comment="Comment ratio is above 30%")
    if value > 40:
        ctx.writeNonCompliance(3, package_id, file_id=file_id,
            comment="Comment ratio is above 40%")

