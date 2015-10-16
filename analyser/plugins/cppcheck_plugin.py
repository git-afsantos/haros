
import os
import subprocess
import xml.etree.ElementTree as ET

def plugin_run(ctx):
    outdir = os.path.join(".", "plugin_out", "cppcheck")
    os.makedirs(outdir)
    packages = ctx.getPackageInfo()
    rules = ruleIds()
    args = ctx.getPluginArguments()
    for p in packages:
        files = ctx.getFileInfo(package_id=p[0])
        if not p[2] is None:
            path = ctx.getPath(p[2])
            process_package(ctx, p[0], path, outdir, args)
            parse_xml(ctx, p[0], outdir, rules, files)
            # manual_checks(ctx, p[0], outdir, rules, files)



def process_package(ctx, pid, path, outdir, args):
    outpath = os.path.join(outdir, str(pid) + ".xml")
    # simplepath = os.path.join(outdir, str(pid) + ".simple")
    outfile = open(outpath, "w")
    # simplified = open(simplepath, "w")
    FNULL = open(os.devnull, "w")
    try:
        cmd = ["cppcheck", "--xml-version=2", "--enable=all"]
        if len(args) > 0:
            cmd.append("--rule-file=" + args[0])
        cmd.append(path)
        subprocess.call(cmd, stdout=FNULL, stderr=outfile)
        # subprocess.call(["cppcheck", '--rule=.+',
                # "--template={file}\n{message}", path],
                # stdout=FNULL, stderr=simplified)
    finally:
        FNULL.close()
        outfile.close()
        # simplified.close()



def parse_xml(ctx, pid, outdir, rules, files):
    fpath = os.path.join(outdir, str(pid) + ".xml")
    if os.path.isfile(fpath) and os.path.getsize(fpath) > 0:
        xml     = ET.parse(fpath)
        root    = xml.getroot()
        errors  = root.find("errors")
        for e in errors:
            handleReport(ctx, pid, rules, files, e)



def manual_checks(ctx, pid, outdir, rules, files):
    fpath = os.path.join(outdir, str(pid) + ".simple")
    fmap = {}
    with open(fpath, "r") as s:
        lines = [line.rstrip('\n') for line in s]
        for i in range(0, len(lines), 2):
            # Two lines at once
            # First line is the file
            # Second line is the source
            fid = getFileId(files, lines[i])
            source = lines[i+1][7:-1]
            fmap[fid] = source
    for k, v in fmap.iteritems():
        print k, v



def ruleIds():
    return {
        "uninitMemberVar":          10400,
        "unusedFunction":           10401,
        "redundantAssignment":      10402,
        "unreadVariable":           10402,
        "variableScope":            10403,
        "harosDeprecatedSTL":       10212,
        "harosRegisterKeyword":     10213,
        "harosThrowSpecification":  10214,
        "harosCaseWithoutBreak":    10404,
        "harosBooleanCase":         10405,
        "harosEnumWithoutBase":     10406,
        "harosInlineAssembly":      10407,
        "harosDoublePointer":       10408,
        "harosTriplePointer":       10409,
        "harosSixParams":           1000,
        "harosConstUniquePtr":      10410,
        "harosDefaultArguments":    10411,
        "harosBooleanVector":       10412,
        "harosUnion":               10413,
        "harosIntegerTypes":        10414,
        "harosAssignIncrement":     10415,
        "harosAssignAssignment":    10415,
        "harosFloatEquality":       10416,
        "harosOverrideUnaryAnd":    10417,
        "harosOverrideBinaryAnd":   10417,
        "harosOverrideBinaryOr":    10417,
        "harosOverrideComma":       10417,
        "harosUsingErrno":          10215,
        "harosRvalueStdArray":      10418
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
        msg = error.get("verbose") or error.get("msg")
    ctx.writeNonCompliance(rid, pid, file_id=fid, line=line, comment=msg)



def getFileId(files, path):
    for f in files:
        fpath = os.path.join(f[2], f[1])
        if path.endswith(fpath):
            return f[0]
    return None

