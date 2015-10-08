#!/usr/bin/env python

# Modified by
# Andre Santos, December 2014

import db_extract as dbe
import db_manager as dbm

import itertools
import math
import os

# Takes a string
def jsonifyItem(i):
	return "            \"" + i + "\""

# Takes an iterator (over an iterable of strings)
def jsonifyMultiItem(it):
	s = ""
	try:
		s += jsonifyItem(next(it))
		for string in it:
			s += ",\n" + jsonifyItem(string)
		s += "\n"
	except StopIteration:
		pass
	return s;


# Takes a package object
def jsonifyPackage(pkg_id, cur):
	s = ""
	s += "  {\n"

	name = dbe.getMatchVal(cur, "Packages", ["name"], "id", pkg_id)
	s += "    \"Name\": \"" + name + "\",\n"

	isMeta = dbe.getMatchVal(cur, "Packages", ["metapackage"], "id", pkg_id)
	s += "    \"Metapackage\": " + ("true" if isMeta == 1 else "false") + ",\n"
	
	dep_ids = dbe.getMatchSet(cur, "Package_Dependencies", ["dependency_id", "type_id"], "package_id", pkg_id)
	run_key = dbe.getMatchVal(cur, "Package_Dependency_Types", ["id"], "dependency_type", "run_depend")
	run_ids = set(d[0] for d in dep_ids if d[1] == run_key)
	run_deps = dbe.getMatchValFromSet(cur, "Packages", ["name"], "id", run_ids)
	if isMeta == 1:
		s += "    \"Contains\": [\n" # Only used for metapackages
		s += jsonifyMultiItem(iter(run_deps))
		s += "    ],\n"

	dscrp = dbe.getMatchVal(cur, "Packages", ["description"], "id", pkg_id)
	s += "    \"Description\": \"" + dscrp + "\",\n"
	
	wiki = dbe.getMatchVal(cur, "Packages", ["wiki"], "id", pkg_id)
	if wiki is None: wiki = ""
	s += "    \"Wiki\": \"" + wiki + "\",\n"

	pkg_metrics = dbe.getMatch(cur, "Package_Metrics",
			["metric_id", "value"], "package_id", pkg_id)
	s += "    \"Metrics\": {\n"
	for i, m in enumerate(pkg_metrics):
		s += "            \"" + str(m[0]) + "\": " + str(m[1])
		if i < len(pkg_metrics) - 1:
			s += ",\n"
		else:
			s += "\n"
	s += "    },\n"

	s += "    \"Repositories\": [\n"
	repo_ids = dbe.getMatchSet(cur, "Repository_Packages", ["repository_id"], "package_id", pkg_id)
	repo_names = dbe.getMatchValFromSet(cur, "Repositories", "name", "id", repo_ids)
	s += jsonifyMultiItem(iter(repo_names))
	s += "    ],\n"

	#s += "    \"Authors\": [\n"
	#author_ids = dbe.getMatchSet(cur, "Package_Authors", ["person_id"], "package_id", pkg_id)
	#author_names = dbe.getMatchValFromSet(cur, "People", "name", "id", author_ids)
	#if None in author_names: author_names.remove(None)
	#s += jsonifyMultiItem(iter(author_names))
	#s += "    ],\n"

	#s += "    \"Maintainers\": [\n"
	#mntnr_ids = dbe.getMatchSet(cur, "Package_Maintainers", ["person_id"], "package_id", pkg_id)
	#mntnr_names = dbe.getMatchValFromSet(cur, "People", "name", "id", mntnr_ids)
	#if None in mntnr_names: mntnr_names.remove(None)
	#s += jsonifyMultiItem(iter(mntnr_names))
	#s += "    ],\n"

	s += "    \"Analysis\": {\n"
	s += "      \"Noncompliance\": {\n"
	non_cpl = dbe.getNonComplianceIdSummary(cur, pkg_id)
	c = len(non_cpl)
	for i, n in non_cpl.iteritems():
		s += "        \"" + str(i) + "\": " + str(n)
		c -= 1
		if c > 0:
			s += ",\n"
		else:
			s += "\n"
	s += "      }\n"
	s += "    },\n"
	
	s += "    \"Edge\": [\n"
	# dep_ids defined above, in case meta
	all_deps = dbe.getMatchValFromSet(cur, "Packages", ["name"], "id", dep_ids)
	s += jsonifyMultiItem(iter(all_deps))
	s += "    ]\n"
	s += "  }"
	return s


def jsonifyPackages(cur):
	s = "[\n"
	pkg_ids = dbe.getTable(cur, "Packages", ["id"])
	s += jsonifyPackage(pkg_ids[0][0], cur)
	for pkg_id in pkg_ids[1:]:
		s += ",\n" + jsonifyPackage(pkg_id[0], cur)
	s += "\n]"
	return s


def export_packages(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    f = open(out_file, "w")
    f.write(jsonifyPackages(db.cur))
    f.close()
    db.disconnect()



def export_metrics_analysis(out_dir, format="json"):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    pkgs = db.get("Packages", ("id", "name"))
    mts = db.getMap("Metrics", ("id", "description"))
    exp = jsonifyMetricsAnalysis
    if format == "csv":
        exp = csvifyMetricsAnalysis
    for pkg in pkgs:
        fnmts = dbe.getFunctionMetricsByPackage(db.cur, package_id=pkg[0])
        clmts = dbe.getClassMetricsByPackage(db.cur, package_id=pkg[0])
        flmts = dbe.getFileMetricsByPackage(db.cur, package_id=pkg[0])
        pkmts = dbe.getPackageMetricsByPackage(db.cur, package_id=pkg[0])
        entries = list(itertools.chain(fnmts, clmts, flmts, pkmts))
        with open(os.path.join(out_dir, pkg[1] + "." + format), "w") as f:
            f.write(exp(entries, mts))
    db.disconnect()


def jsonifyMetricsAnalysis(entries, metrics):
    s = "[\n"
    n = len(entries) - 1
    for i, v in enumerate(entries):
        s += "  {\n"
        s += '    "metric": "' + metrics[v[1]][1].replace('"', "'") + '",\n'
        s += '    "minimum": ' + str(v[2] if not v[2] is None else "null") + ",\n"
        s += '    "maximum": ' + str(v[3] if not v[3] is None else "null") + ",\n"
        s += '    "average": ' + str(v[4] if not v[4] is None else "null") + "\n"
        if i < n:
            s += "  },\n"
        else:
            s += "  }\n"
    s += "]"
    return s

def csvifyMetricsAnalysis(entries, metrics):
    s = "metric,minimum,maximum,average\n"
    for e in entries:
        s += metrics[e[1]][1].replace(",", "_") + ","
        s += str(e[2]) + ","
        s += str(e[3]) + ","
        s += str(e[4]) + "\n"
    return s



def export_analysis(out_dir, format="json"):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    if format == "csv":
        ncpl = dbe.getNonComplianceCompact(db.cur)
        pkgs = db.getMap("Packages", ("id", "name"))
        rules = db.getMap("Rules", ("id", "name"))
        csvifyNonComplianceAnalysis(out_dir, ncpl, pkgs, rules)
    else:
        pkgs = db.get("Packages", ("id", "name"))
        rules = dbe.getRulesWithTags(db.cur)
        for pkg in pkgs:
            ncpl = db.get("Non_Compliance",
                    ("rule_id", "file_id", "line", "function", "comment"),
                    match=("package_id", pkg[0]))
            files = db.get("Files", ("id", "name"), match=("package_id", pkg[0]))
            file_dict = {}
            for f in files:
                file_dict[f[0]] = f[1]
            files = file_dict
            with open(os.path.join(out_dir, pkg[1] + ".json"), "w") as f:
                f.write(jsonifyNonCompliance(ncpl, rules, files))
    db.disconnect()


def jsonifyNonCompliance(violations, rules, files):
    s = "[\n"
    for i, v in enumerate(violations):
        s += "  {\n"
        s += '    "rule": "' + rules[v[0]][0].replace('"', "'") + '",\n'
        s += '    "file": ' + ('"'+files[v[1]]+'"' if v[1] else "null") + ",\n"
        s += '    "line": ' + str(v[2] or "null") + ",\n"
        s += '    "function": ' + ('"'+v[3]+'"' if v[3] else "null") + ",\n"
        s += '    "comment": "' + (str(v[4] or "")).replace('"', "'").replace("\n", "") + "\",\n"
        s += '    "tags": ["' + '","'.join(rules[v[0]][1]) + '"]\n'
        if i < len(violations) - 1:
            s += "  },\n"
        else:
            s += "  }\n"
    s += "]"
    return s

def csvifyNonComplianceAnalysis(out_dir, violations, packages, rules):
    with open(os.path.join(out_dir, "package_noncompliance.csv"), "w") as f:
        f.write("package,rule,violations\n")
        for v in violations:
            s = packages[v[0]][1].replace(",", "_") + ","
            s += rules[v[1]][1].replace(",", "_") + ","
            s += str(v[2]) + "\n"
            f.write(s)




def export_rules(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    f = open(out_file, "w")
    rules = dbe.getRulesWithTags(db.cur)
    f.write(jsonifyRules(rules))
    f.close()
    db.disconnect()


def jsonifyRules(rules):
    c = len(rules)
    s = "[\n"
    for k, v in rules.iteritems():
        s += "  {\n"
        s += '    "id": ' + str(k) + ',\n'
        s += '    "description": "' + (str(v[0] or "")).replace('"', "'").replace("\n", "") + "\",\n"
        s += '    "tags": ["' + '","'.join(v[1]) + '"]\n'
        c -= 1
        if c > 0:
            s += "  },\n"
        else:
            s += "  }\n"
    s += "]"
    return s



def export_metrics(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    f = open(out_file, "w")
    metrics = db.get("Metrics", ["id", "name", "description"])
    f.write(jsonifyMetrics(metrics))
    f.close()
    db.disconnect()

def jsonifyMetrics(metrics):
    c = len(metrics)
    s = "[\n"
    for m in metrics:
        s += "  {\n"
        s += '    "id": ' + str(m[0]) + ',\n'
        s += '    "name": "' + (str(m[1] or "")).replace('"', "'").replace("\n", "") + "\",\n"
        s += '    "description": "' + (str(m[2] or "")).replace('"', "'").replace("\n", "") + "\"\n"
        c -= 1
        if c > 0:
            s += "  },\n"
        else:
            s += "  }\n"
    s += "]"
    return s


# Hard-coded for Cyclomatic Complexity
def csv_export_package_cc(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    packages = db.getMap("Packages", ("id", "name"))
    metrics = dbe.getFunctionMetricsByPackage(db.cur, metric_id=4)
    with open(out_file, "w") as f:
        f.write("package,cc_min,cc_max,cc_avg\n")
        for m in metrics:
            s = packages[m[0]][1] + ","
            s += str(m[2]) + ","
            s += str(m[3]) + ","
            s += str(m[4]) + "\n"
            f.write(s)
    db.disconnect()

# Hard-coded for C++ Lines of Code
def csv_export_package_cpp_loc(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    packages = db.getMap("Packages", ("id", "name"))
    metrics = dbe.getFileMetricsByPackage(db.cur, metric_id=2, inc_sum=True)
    with open(out_file, "w") as f:
        f.write("package,loc_min,loc_max,loc_avg,loc_sum\n")
        for m in metrics:
            s = packages[m[0]][1] + ","
            s += str(m[2]) + ","
            s += str(m[3]) + ","
            s += str(m[4]) + ","
            s += str(m[5]) + "\n"
            f.write(s)
    db.disconnect()

# Hard-coded for Lines of Comments
def csv_export_package_com(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    packages = db.getMap("Packages", ("id", "name"))
    metrics = dbe.getFileMetricsByPackage(db.cur, metric_id=3, inc_sum=True)
    with open(out_file, "w") as f:
        f.write("package,com_min,com_max,com_avg,com_sum\n")
        for m in metrics:
            s = packages[m[0]][1] + ","
            s += str(m[2]) + ","
            s += str(m[3]) + ","
            s += str(m[4]) + ","
            s += str(m[5]) + "\n"
            f.write(s)
    db.disconnect()

# Hard-coded for Ratio of Lines of Comments
def csv_export_package_com_ratio(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    packages = db.getMap("Packages", ("id", "name"))
    metrics = dbe.getFileMetricsByPackage(db.cur, metric_id=14)
    with open(out_file, "w") as f:
        f.write("package,com_ratio_min,com_ratio_max,com_ratio_avg\n")
        for m in metrics:
            s = packages[m[0]][1] + ","
            s += str(m[2]) + ","
            s += str(m[3]) + ","
            s += str(m[4]) + "\n"
            f.write(s)
    db.disconnect()


# Hard-coded for Halstead Volume
def csv_export_package_hal_volume(out_file):
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    packages = db.getMap("Packages", ("id", "name"))
    metrics = dbe.getFileMetricsByPackage(db.cur, metric_id=15, inc_sum=True)
    with open(out_file, "w") as f:
        f.write("package,hvol_min,hvol_max,hvol_avg,hvol_sum\n")
        for m in metrics:
            s = packages[m[0]][1] + ","
            s += str(m[2]) + ","
            s += str(m[3]) + ","
            s += str(m[4]) + ","
            s += str(m[5]) + "\n"
            f.write(s)
    db.disconnect()


# http://www.verifysoft.com/en_maintainability.html
# MIwoc = 171 - 5.2 * ln(aveV) -0.23 * aveG -16.2 * ln(aveLOC)
# MIcw = 50 * sin(sqrt(2.4 * perCM))
# MI = MIwoc + MIcw

# aveV = average Halstead Volume
# aveG = average extended cyclomatic complexity
# aveLOC = average count of lines
# perCM = average percent of lines of comments

if __name__ == "__main__":
    out_file = os.path.join("export", "package_repo_metrics.csv")
    db = dbm.DbManager()
    db.connect("dbuser.txt")
    metrics = dict()
    ncpl = dict()
    idx = dict()
    packages = db.getMap("Packages", ("id", "name", "repo_id", "level"))
    repos = db.getMap("Repositories", ("id", "distro_name", "contributors_count", "commits_count"))
    issues = db.getMap("Repository_Issues", ("repo_id", "open_issues", "closed_issues"), key = "repo_id")
    rules = db.getMap("Rules", ("id", "name"))
    for p in packages.values():
        r = repos[p[2]]
        i = issues[p[2]]
        metrics[p[0]] = [p[1], p[3], r[1], r[2], r[3],
                            0, i[1], i[2], 0, 0,
                            0, 0, i[1] + i[2], 0, 0,
                            0, 0, 0, 0, 0]
        ncpl[p[0]] = [p[1], p[3], r[1], r[2], r[3], 0, i[1], i[2], [], []]
        # (loc, com, cc, vol)
        idx[p[0]] = [0, 0, 0, 0]
    ms = dbe.getPackageDependencyCount(db.cur)
    for m in ms:
        metrics[m[0]][5] = m[1]
        ncpl[m[0]][5] = m[1]
    # Cyclomatic Complexity
    ms = dbe.getFunctionMetricsByPackage(db.cur, metric_id=4)
    for m in ms:
        metrics[m[0]][8] = m[4]
        idx[m[0]][2] = m[4]
    # C++ Lines of Code
    ms = dbe.getFileMetricsByPackage(db.cur, metric_id=2, inc_sum=True)
    for m in ms:
        metrics[m[0]][9] = m[5]
        idx[m[0]][0] = m[4]
    # C++ Lines of Comments
    ms = dbe.getFileMetricsByPackage(db.cur, metric_id=3, inc_sum=True)
    for m in ms:
        metrics[m[0]][10] = m[5]
        if metrics[m[0]][9] > 0:
            metrics[m[0]][11] = m[5] / metrics[m[0]][9]
        idx[m[0]][1] = m[4]
    # Halstead Volume & Maintainability Index
    ms = dbe.getFileMetricsByPackage(db.cur, metric_id=15, inc_sum=True)
    for m in ms:
        idx[m[0]][3] = m[4]
    for k, m in metrics.iteritems():
        if idx[k][0] > 0 and idx[k][3] > 0:
            i = (171 - (5.2 * math.log(idx[k][3])) - (0.23 * m[8]) - (16.2 * math.log(idx[k][0]))) * 100 / 171
            i += 50 * math.sin(math.sqrt(2.4 * m[11]))
        else:
            i = 0
        m[14] = max(0, i)
    # Coupling Between Objects
    ms = dbe.getClassMetricsByPackage(db.cur, metric_id=8)
    for m in ms:
        metrics[m[0]][15] = m[4]
    # Number of Children
    ms = dbe.getClassMetricsByPackage(db.cur, metric_id=9)
    for m in ms:
        metrics[m[0]][16] = m[4]
    # Weighted Methods per Class
    ms = dbe.getClassMetricsByPackage(db.cur, metric_id=10)
    for m in ms:
        metrics[m[0]][17] = m[4]
    # Depth of Inheritance Tree
    ms = dbe.getClassMetricsByPackage(db.cur, metric_id=11)
    for m in ms:
        metrics[m[0]][18] = m[4]
    # Number of Methods Available in Class
    ms = dbe.getClassMetricsByPackage(db.cur, metric_id=12)
    for m in ms:
        metrics[m[0]][19] = m[4]
    # Only ROS rules
    ruleset = [1,6,9,12,14,17,18,19,20,22,23,24,25,10200,10202]
    ms = dbe.getNonComplianceCompact(db.cur)
    for m in ms:
        if m[1] in ruleset:
            metrics[m[0]][13] += m[2]
            ncpl[m[0]][8].append(rules[m[1]][1])
            ncpl[m[0]][9].append(str(m[2]))
    ms = None
    # Group by repository
    idx = dict()
    for r in repos.values():
        idx[r[1]] = []
    for m in metrics.values():
        idx[m[2]].append(m)
    metrics = None
    # Output to file
    with open(out_file, "w") as f:
        f.write("Package,Level,Repository,Contributors,Commits,Dependency of,Open issues,Closed issues,CC (avg),Cpp LoC,Cpp LoCom,Com Ratio,Total Issues,Violations,Maintainability,CBO,NoC,WMC,DIT,MAC\n")
        for r in idx.values():
            for m in r:
                if m[9] > 0:
                    s = m[0] + ","
                    s += str(m[1]) + ","
                    s += m[2] + ","
                    s += str(m[3]) + ","
                    s += str(m[4]) + ","
                    s += str(m[5]) + ","
                    s += str(m[6]) + ","
                    s += str(m[7]) + ","
                    s += "{:.2f}".format(m[8]) + ","
                    s += str(int(m[9])) + ","
                    s += str(int(m[10])) + ","
                    s += "{:.2f}".format(m[11]) + ","
                    s += str(m[12]) + ","
                    s += str(m[13]) + ","
                    s += str(m[14]) + ","
                    s += str(m[15]) + ","
                    s += str(m[16]) + ","
                    s += str(m[17]) + ","
                    s += str(m[18]) + ","
                    s += str(m[19]) + "\n"
                    f.write(s)
    out_file = os.path.join("export", "package_compliance.csv")
    for r in repos.values():
        idx[r[1]] = []
    for n in ncpl.values():
        idx[n[2]].append(n)
    ncpl = None
    with open(out_file, "w") as f:
        f.write("Package,Level,Repository,Contributors,Commits,Dependency of,Open Issues,Closed Issues,Rule,Violations\n")
        for r in idx.values():
            for n in r:
                rs = n[8]
                for i, v in enumerate(rs):
                    s = n[0] + ","
                    s += str(n[1]) + ","
                    s += n[2] + ","
                    s += str(n[3]) + ","
                    s += str(n[4]) + ","
                    s += str(n[5]) + ","
                    s += str(n[6]) + ","
                    s += str(n[7]) + ","
                    s += v + ","
                    s += n[9][i] + "\n"
                    f.write(s)
    db.disconnect()

