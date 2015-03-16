
import re
import os
import subprocess
import xml.etree.ElementTree as ET


def plugin_run(context):
    # Retrieve list of files in database.
    # [(id, name, path)]
    files = context.getFileInfo()
    # Get the metric ids from database.
    # [(id, name)]
    metrics = context.getMetricIds()
    # Change list of tuples to mapping of name to id.
    # {metric_name -> metric_id}
    metrics = metric_id_map(metrics)
    for f in files:
        # Get absolute path for file.
        file_path = context.getPath(f[2], file_name = f[1])
        if file_path[:-2] == "py":
            analyse_python_file(context, f[0], file_path, metrics)
        else:
            analyse_cpp_file(context, f[0], file_path, metrics)



def analyse_python_file(db, file_id, file_path, metrics):
    out = os.path.join(".", "plugin_out", "output_radon.txt")
    with open(out, "w") as radon_file:
        subprocess.call(["radon", "raw", file_path], stdout = radon_file)
    if os.path.exists(out):
        parse_radon_metrics(db, file_id, metrics, out)
    else:
        db.writeFileMetric(file_id, metrics["LOC_PY"], 0)



def parse_radon_metrics(db, file_id, metrics, radon_file):
    with open(radon_file, "r") as rf:
        lines = [line.rstrip("\n") for line in rf]
    re_loc = re.compile(r"\s*LOC:\s+(\d+)")
    for line in lines:
        loc_match = re_loc.match(line)
        if loc_match:
            value = int(loc_match.group(1))
            db.writeFileMetric(file_id, metrics["LOC_PY"], value)



def analyse_cpp_file(db, file_id, file_path, metrics):
    outdir = os.path.join(".", "plugin_out", "cccc", str(file_id))
    os.makedirs(outdir)
    subprocess.call(["cccc", file_path, "--outdir=" + outdir])
    parse_cccc_metrics(db, file_id, metrics, outdir)
    for f in os.listdir(outdir):
        if f.endswith(".xml") and not f == "cccc.xml":
            parse_cccc_function_metrics(f, db, file_id, metrics, outdir)
    


def parse_cccc_metrics(db, file_id, metrics, outdir):
    xml = os.path.join(outdir, "cccc.xml")
    if os.path.exists(xml):
        tree    = ET.parse(xml)
        root    = tree.getroot()
        project = root.find("project_summary")
        loc     = project.find("lines_of_code").get("value")
        com     = project.find("lines_of_comment").get("value")
        db.writeFileMetric(file_id, metrics["LOC_CPP"], int(loc))
        db.writeFileMetric(file_id, metrics["COM"], int(com))
    else:
        db.writeFileMetric(file_id, metrics["LOC_CPP"], 0)
        db.writeFileMetric(file_id, metrics["COM"], 0)



def parse_cccc_function_metrics(xml_file, db, file_id, metrics, outdir):
    module_name = xml_file[:-4]
    xml_file    = os.path.join(outdir, xml_file)
    tree        = ET.parse(xml_file)
    root        = tree.getroot()
    summary     = root.find("module_summary")
    cbo = summary.find("coupling_between_objects").get("value")
    noc = summary.find("number_of_children").get("value")
    wmc = summary.find("weighted_methods_per_class_unity").get("value")
    dit = summary.find("depth_of_inheritance_tree").get("value")
    mac = summary.find("weighted_methods_per_class_visibility").get("value")
    db.writeClassMetric(file_id, module_name, 0, metrics["CBO"], int(cbo))
    db.writeClassMetric(file_id, module_name, 0, metrics["NOC"], int(noc))
    db.writeClassMetric(file_id, module_name, 0, metrics["WMC"], int(wmc))
    db.writeClassMetric(file_id, module_name, 0, metrics["DIT"], int(dit))
    db.writeClassMetric(file_id, module_name, 0, metrics["MAC"], int(mac))
    summary = root.find("procedural_detail")
    for function in summary.findall("member_function"):
        split_name  = function.find("name").text.split("(")
        name        = split_name[0]
        params      = split_name[1][:-1]
        if params == "":
            params  = 0
        else:
            params  = len(params.split(","))
        line    = function.find("source_reference").get("line")
        cyclo   = function.find("McCabes_cyclomatic_complexity").get("value")
        loc     = function.find("lines_of_code").get("value")
        com     = function.find("lines_of_comment").get("value")
        db.writeFunctionMetric(file_id, name, line,
                metrics["LOC_CPP"], int(loc))
        db.writeFunctionMetric(file_id, name, line,
                metrics["COM"], int(com))
        db.writeFunctionMetric(file_id, name, line,
                metrics["CC"], int(cyclo))
        db.writeFunctionMetric(file_id, name, line,
                metrics["FUN_PARAM"], params)



def metric_id_map(metrics):
    metric_map = {}
    for m in metrics:
        metric_map[m[1]] = m[0]
    return metric_map

