#!/usr/bin/env python

# Modified by
# Andre Santos, December 2014

import db_extract as dbe
import db_manager as dbm

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
	s += "    {\n"

	name = dbe.getMatchVal(cur, "Packages", ["name"], "id", pkg_id)
	s += "        \"Name\": \"" + name + "\",\n"

	isMeta = dbe.getMatchVal(cur, "Packages", ["metapackage"], "id", pkg_id)
	s += "        \"Metapackage\": " + ("true" if isMeta == 1 else "false") + ",\n"
	
	dep_ids = dbe.getMatchSet(cur, "Package_Dependencies", ["dependency_id", "type_id"], "package_id", pkg_id)
	run_key = dbe.getMatchVal(cur, "Package_Dependency_Types", ["id"], "dependency_type", "run_depend")
	run_ids = set(d[0] for d in dep_ids if d[1] == run_key)
	run_deps = dbe.getMatchValFromSet(cur, "Packages", ["name"], "id", run_ids)
	if isMeta == 1:
		s += "        \"Contains\": [\n" # Only used for metapackages
		s += jsonifyMultiItem(iter(run_deps))
		s += "        ],\n"

	dscrp = dbe.getMatchVal(cur, "Packages", ["description"], "id", pkg_id)
	s += "        \"Description\": \"" + dscrp + "\",\n"
	
	wiki = dbe.getMatchVal(cur, "Packages", ["wiki"], "id", pkg_id)
	if wiki is None: wiki = ""
	s += "        \"Wiki\": \"" + wiki + "\",\n"

	pkg_metrics = dbe.getMatch(cur, "Package_Metrics",
			["metric_id", "value"], "package_id", pkg_id)
	s += "        \"Metrics\": {\n"
	for i, m in enumerate(pkg_metrics):
		s += "            \"" + str(m[0]) + "\": " + str(m[1])
		if i < len(pkg_metrics) - 1:
			s += ",\n"
		else:
			s += "\n"
	s += "        },\n"

	s += "        \"Repositories\": [\n"
	repo_ids = dbe.getMatchSet(cur, "Repository_Packages", ["repository_id"], "package_id", pkg_id)
	repo_names = dbe.getMatchValFromSet(cur, "Repositories", "name", "id", repo_ids)
	s += jsonifyMultiItem(iter(repo_names))
	s += "        ],\n"

	s += "        \"Authors\": [\n"
	author_ids = dbe.getMatchSet(cur, "Package_Authors", ["person_id"], "package_id", pkg_id)
	author_names = dbe.getMatchValFromSet(cur, "People", "name", "id", author_ids)
	if None in author_names: author_names.remove(None)
	s += jsonifyMultiItem(iter(author_names))
	s += "        ],\n"

	s += "        \"Maintainers\": [\n"
	mntnr_ids = dbe.getMatchSet(cur, "Package_Maintainers", ["person_id"], "package_id", pkg_id)
	mntnr_names = dbe.getMatchValFromSet(cur, "People", "name", "id", mntnr_ids)
	if None in mntnr_names: mntnr_names.remove(None)
	s += jsonifyMultiItem(iter(mntnr_names))
	s += "        ],\n"

	s += "        \"Analysis\": {\n"
	s += "            \"Noncompliance\": {\n"
	non_cpl = dbe.getNonComplianceSummary(cur, pkg_id)
	for i, n in enumerate(non_cpl):
		s += "                \"" + n[0] + "\": " + str(n[1]) + ",\n"
    s += "                \"total\": " + str(len(non_cpl)) + "\n"
	s += "            }\n"
	s += "        },\n"
	
	s += "        \"Edge\": [\n"
	# dep_ids defined above, in case meta
	all_deps = dbe.getMatchValFromSet(cur, "Packages", ["name"], "id", dep_ids)
	s += jsonifyMultiItem(iter(all_deps))
	s += "        ]\n"
	s += "    }"
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

