
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



def parse_radon_metrics(db, file_id, metrics, radon_file):
    with open(radon_file, "r") as rf:
        lines = [line.rstrip("\n") for line in rf]
    re_loc = re.compile(r"\s*LOC:\s+(\d+)")
    for line in lines:
        loc_match = re_loc.match(line)
        if loc_match:
            value = int(loc_match.group(1))
            if value > 0:
                db.writeFileMetric(file_id, metrics["LOC_PY"], value)



def analyse_cpp_file(db, file_id, file_path, metrics):
    outdir = os.path.join(".", "plugin_out", "cccc", str(file_id))
    os.makedirs(outdir)
    FNULL = open(os.devnull, 'w')
    try:
        subprocess.call(["cccc", file_path, "--outdir=" + outdir],
                stdout=FNULL, stderr=subprocess.STDOUT)
        parse_cccc_metrics(db, file_id, metrics, outdir)
        for f in os.listdir(outdir):
            if f.endswith(".xml") and not f == "cccc.xml":
                parse_cccc_function_metrics(f, db, file_id, metrics, outdir)
    finally:
        FNULL.close()
    


def parse_cccc_metrics(db, file_id, metrics, outdir):
    xml = os.path.join(outdir, "cccc.xml")
    if os.path.exists(xml):
        tree    = ET.parse(xml)
        root    = tree.getroot()
        project = root.find("project_summary")
        loc     = int(project.find("lines_of_code").get("value"))
        com     = int(project.find("lines_of_comment").get("value"))
        if loc > 0:
            db.writeFileMetric(file_id, metrics["LOC_CPP"], loc)
        if com > 0:
            db.writeFileMetric(file_id, metrics["COM"], com)



def parse_cccc_function_metrics(xml_file, db, file_id, metrics, outdir):
    module_name = xml_file[:-4]
    xml_file    = os.path.join(outdir, xml_file)
    tree        = ET.parse(xml_file)
    root        = tree.getroot()
    summary     = root.find("module_summary")
    write_cccc_class_metrics(summary, db, file_id, module_name, metrics)
    summary = root.find("procedural_detail")
    for function in summary.findall("member_function"):
        line = function.find("extent").find("source_reference")
        if not line is None:
            line = int(line.get("line"))
        else:
            # Skip functions with no line number (not sure if this happens).
            continue
        write_cccc_function_metrics(function, db, file_id, line, metrics)



def write_cccc_class_metrics(xml, db, file_id, cname, metrics):
    cbo = int(xml.find("coupling_between_objects").get("value"))
    noc = int(xml.find("number_of_children").get("value"))
    wmc = int(xml.find("weighted_methods_per_class_unity").get("value"))
    dit = int(xml.find("depth_of_inheritance_tree").get("value"))
    mac = int(xml.find("weighted_methods_per_class_visibility").get("value"))
    if cbo > 0:
        db.writeClassMetric(file_id, cname, 0, metrics["CBO"], cbo)
    if noc > 0:
        db.writeClassMetric(file_id, cname, 0, metrics["NOC"], noc)
    if wmc > 0:
        db.writeClassMetric(file_id, cname, 0, metrics["WMC"], wmc)
    if dit > 0:
        db.writeClassMetric(file_id, cname, 0, metrics["DIT"], dit)
    if mac > 0:
        db.writeClassMetric(file_id, cname, 0, metrics["MAC"], mac)



def write_cccc_function_metrics(xml, db, file_id, line, metrics):
    split_name  = xml.find("name").text.split("(")
    fname       = split_name[0]
    params      = split_name[1][:-1]
    if params == "":
        params  = 0
    else:
        params  = len(params.split(","))
    cyclo   = int(xml.find("McCabes_cyclomatic_complexity").get("value"))
    loc     = int(xml.find("lines_of_code").get("value"))
    com     = int(xml.find("lines_of_comment").get("value"))
    if loc > 0:
        db.writeFunctionMetric(file_id, fname, line, metrics["LOC_CPP"], loc)
    if com > 0:
        db.writeFunctionMetric(file_id, fname, line, metrics["COM"], com)
    if cyclo > 0:
        db.writeFunctionMetric(file_id, fname, line, metrics["CC"], cyclo)
    if params > 0:
        db.writeFunctionMetric(file_id, fname, line,
                metrics["FUN_PARAM"], params)



def metric_id_map(metrics):
    metric_map = {}
    for m in metrics:
        metric_map[m[1]] = m[0]
    return metric_map

