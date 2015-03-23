
import os
import subprocess
import xml.etree.ElementTree as ET


def plugin_run(ctx):
    files = get_files(ctx)
    handlers = get_metric_handlers()
    for f in files:
        file_path = ctx.getPath(f[2], file_name = f[1])
        process_metrics(ctx, f[0], file_path, f[3], handlers)



def process_metrics(ctx, file_id, file_path, package_id, handlers):
    outdir = os.path.join(".", "plugin_out", "cccc", str(file_id))
    os.makedirs(outdir)
    FNULL = open(os.devnull, 'w')
    try:
        subprocess.call(["cccc", file_path, "--outdir=" + outdir],
                stdout=FNULL, stderr=subprocess.STDOUT)
        parse_cccc_metrics(ctx, file_id, package_id, handlers, outdir)
        for f in os.listdir(outdir):
            if f.endswith(".xml") and not f == "cccc.xml":
                parse_cccc_function_metrics(f, ctx, file_id,
                        package_id, handlers, outdir)
    finally:
        FNULL.close()



def parse_cccc_metrics(ctx, file_id, package_id, handlers, outdir):
    xml = os.path.join(outdir, "cccc.xml")
    if os.path.exists(xml):
        tree    = ET.parse(xml)
        root    = tree.getroot()
        project = root.find("project_summary")
        loc     = int(project.find("lines_of_code").get("value"))
        com     = int(project.find("lines_of_comment").get("value"))
        if loc > 0:
            ratio = (com * 100) / loc
            handlers["com_ratio"](ctx, package_id, file_id, ratio)


def parse_cccc_function_metrics(xml_file, ctx, file_id, package_id,
        handlers, outdir):
    module_name = xml_file[:-4]
    xml_file    = os.path.join(outdir, xml_file)
    tree        = ET.parse(xml_file)
    root        = tree.getroot()
    summary     = root.find("module_summary")
    handle_cccc_class_metrics(summary, ctx, file_id, package_id,
            module_name, handlers)
    summary = root.find("procedural_detail")
    for function in summary.findall("member_function"):
        line = function.find("extent").find("source_reference")
        if not line is None:
            line = int(line.get("line"))
        else:
            # Skip functions with no line number (not sure if this happens).
            continue
        handle_cccc_function_metrics(function, ctx, file_id, package_id,
                line, handlers)



def handle_cccc_class_metrics(xml, ctx, file_id, package_id, cname, handlers):
    cbo = int(xml.find("coupling_between_objects").get("value"))
    noc = int(xml.find("number_of_children").get("value"))
    wmc = int(xml.find("weighted_methods_per_class_unity").get("value"))
    dit = int(xml.find("depth_of_inheritance_tree").get("value"))
    mac = int(xml.find("weighted_methods_per_class_visibility").get("value"))
    handlers["cbo"](ctx, package_id, file_id, cname, cbo)
    handlers["noc"](ctx, package_id, file_id, cname, noc)
    handlers["wmc"](ctx, package_id, file_id, cname, wmc)
    handlers["dit"](ctx, package_id, file_id, cname, dit)
    handlers["mac"](ctx, package_id, file_id, cname, mac)



def handle_cccc_function_metrics(xml, ctx, file_id, package_id, line, handlers):
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
    handlers["cc"](ctx, package_id, file_id, fname, line, cyclo)



def get_files(ctx):
    cpp = ctx.getFileInfo(ext="cpp")
    hpp = ctx.getFileInfo(ext="hpp")
    files = []
    for f in hpp:
        files.append(f)
    for f in cpp:
        files.append(f)
    return files


def get_metric_handlers():
    return {
        "cc": handle_cc,
        "cbo": handle_cbo,
        "noc": handle_noc,
        "wmc": handle_wmc,
        "dit": handle_dit,
        "mac": handle_mac,
        "com_ratio": handle_com_ratio
    }



def handle_cc(ctx, package_id, file_id, function, line, value):
    if value < 1:
        ctx.writeNonCompliance(4, package_id, file_id=file_id,
                line=line, function=function, comment="CC is less than 1")
    if value > 10:
        ctx.writeNonCompliance(5, package_id, file_id=file_id,
                line=line, function=function, comment="CC is greater than 10")
    if value > 15:
        ctx.writeNonCompliance(6, package_id, file_id=file_id,
                line=line, function=function, comment="CC is greater than 15")


def handle_cbo(ctx, package_id, file_id, class_name, value):
    if value > 5:
        ctx.writeNonCompliance(18, package_id, file_id=file_id,
                comment="CBO is greater than 5, " + class_name + ", " + str(value))


def handle_noc(ctx, package_id, file_id, class_name, value):
    if value > 10:
        ctx.writeNonCompliance(19, package_id, file_id=file_id,
                comment="NOC is greater than 10")


def handle_wmc(ctx, package_id, file_id, class_name, value):
    if value < 1:
        ctx.writeNonCompliance(20, package_id, file_id=file_id,
                comment="{0}: WMC < 1 ({1})".format(class_name, str(value)))
    if value > 50:
        ctx.writeNonCompliance(21, package_id, file_id=file_id,
                comment="WMC is greater than 50")
    if value > 100:
        ctx.writeNonCompliance(22, package_id, file_id=file_id,
                comment="WMC is greater than 100")


def handle_dit(ctx, package_id, file_id, class_name, value):
    if value > 5:
        ctx.writeNonCompliance(23, package_id, file_id=file_id,
                comment="DIL is greater than 5")


def handle_mac(ctx, package_id, file_id, class_name, value):
    if value < 1:
        ctx.writeNonCompliance(24, package_id, file_id=file_id,
                comment="MAC is less than 1")
    if value > 20:
        ctx.writeNonCompliance(25, package_id, file_id=file_id,
                comment="MAC is greater than 20")


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

