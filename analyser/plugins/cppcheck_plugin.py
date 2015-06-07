
import os
import subprocess
import xml.etree.ElementTree as ET

def plugin_run(ctx):
    outdir = os.path.join(".", "plugin_out", "cppcheck")
    os.makedirs(outdir)
    packages = ctx.getPackageInfo()
    rules = ruleIds()
    for p in packages:
        files = ctx.getFileInfo(package_id=p[0])
        path = ctx.getPath(p[2])
        process_package(ctx, p[0], path, outdir)
        parse_xml(ctx, p[0], outdir, rules, files)



def process_package(ctx, pid, path, outdir):
    outpath = os.path.join(outdir, str(pid) + ".xml")
    outfile = open(outpath, "w")
    FNULL = open(os.devnull, "w")
    try:
        subprocess.call(["cppcheck", "--xml-version=2", "--enable=all", path],
                stdout=FNULL, stderr=outfile)
    finally:
        FNULL.close()
        outfile.close()



def parse_xml(ctx, pid, outdir, rules, files):
    fpath = os.path.join(outdir, str(pid) + ".xml")
    if os.path.isfile(fpath) and os.path.getsize(fpath) > 0:
        xml     = ET.parse(fpath)
        root    = xml.getroot()
        errors  = root.find("errors")
        for e in errors:
            handleReport(ctx, pid, rules, files, e)



def ruleIds():
    return {
        "uninitMemberVar":      10400,
        "unusedFunction":       10401,
        "redundantAssignment":  10402,
        "unreadVariable":       10402,
        "variableScope":        10403
    }



def handleReport(ctx, pid, rules, files, error):
    eid = error.get("id")
    if not eid in rules:
        return
    rid = rules[eid]
    fid = None
    line = None
    loc = error.find("location")
    if not loc is None:
        fpath = loc.get("file")
        line = int(loc.get("line", default="0"))
        fid = getFileId(files, fpath)
    ctx.writeNonCompliance(rid, pid, file_id=fid, line=line,
            comment=error.get("verbose"))



def getFileId(files, path):
    for f in files:
        fpath = os.path.join(f[2], f[1])
        if path.endswith(fpath):
            return f[0]
    return None

