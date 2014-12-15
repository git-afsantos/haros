# Modified by
# Andre Santos, November 2014

import repo
import package

import clone_repos as crep
import source_finder as sf

import extractor
# import scorer

import db_control as dbc
import db_extract as dbe

# import chronicle as chro

import json_exp_db as j_exp

import os




class DbUpdater:
    def __init__(self):
        self.con = None
        self.cur = None

    def connect(self, db_user_file):
        db_user = load_db_user(db_user_file)
        con, cur = dbc.conCur(user = db_user["user"],
                passwd = db_user["passwd"],
                db = db_user["db"],
                host = db_user["host"],
                port = db_user["port"])
        self.con = con
        self.cur = cur

    def insert(self, table, cols, data, truncate=False):
        dbc.insertRecords(self.con, self.cur,
                table, cols, data, truncate=truncate)

    def updateTable(self, table, cols, types, data = None,
                    pk = None, fk = None, fk_ref = None):
        dbc.upTruncTable(self.con, self.cur, table,
                cols, types, data,
                pk = pk, fk = fk, fk_ref = fk_ref)

    def get(self, table, cols):
        return dbe.getTable(self.cur, table, cols)

    def truncate(self, table):
        dbc.truncateTable(self.cur, table)
        self.con.commit()




def load_db_user(db_user_file):
    lines = [line.strip() for line in open(db_user_file)]
    data = {}
    data["user"] = lines[0]
    data["passwd"] = lines[1]
    data["db"] = lines[2]
    data["host"] = lines[3]
    data["port"] = int(lines[4])
    return data



def update_source_files(dbu, trunc_deps = False):
    src_root = os.path.join(os.path.expanduser("~"), "ros", "repos")
    pkg_info = dbu.get("Packages", ["name", "path"])
    print "Finding source files..."
    # Find files on local storage
    src_info = sf.find_source_files(src_root, pkg_info, tuples = True)
    pkg_info = dbu.get("Packages", ["id", "name"])
    # Associate file ids with package ids
    src_info = extractor.getPkgFiles(pkg_info, src_info)
    print "Updating Files table, rows:", len(src_info)
    dbu.updateTable("Files", ["id", "pkg_id", "name", "path"],
            ["SMALLINT(6)", "SMALLINT(6)", "VARCHAR(50)", "VARCHAR(200)"],
            src_info, pk = "id", fk = ["pkg_id"], fk_ref = ["Packages(id)"])
    if trunc_deps:
        # Truncate tables depending on Files
        dbu.truncate("FileMetrics")
        dbu.truncate("FileClassMetrics")
        dbu.truncate("FileFunctionMetrics")
        dbu.truncate("FileNonCompliance")


def update_code_metrics(dbu, update_deps = False, calculate = False):
    # Read metrics from file
    print "Reading code metrics from file..."
    # TODO
    # Recalculate metric values
    if calculate:
        print "Calculating metric values..."
        # TODO
    # Update all metrics tables
    metric_info = None
    print "Updating Metrics table, rows:", 0
    dbu.updateTable("Metrics", ["id", "name", "description"],
            ["SMALLINT(6)", "VARCHAR(50)", "VARCHAR(2000)"],
            metric_info, pk = "id")
    if update_deps:
        specific_metrics = None
        dbu.updateTable("FileMetrics", ["file_id", "metric_id", "value"],
                ["SMALLINT(6)", "SMALLINT(6)", "FLOAT"],
                specific_metrics, pk = "file_id, metric_id",
                fk = ["file_id", "metric_id"],
                fk_ref = ["Files(id)", "Metrics(id)"])
        specific_metrics = None
        dbu.updateTable("PackageMetrics", ["pkg_id", "metric_id", "value"],
                ["SMALLINT(6)", "SMALLINT(6)", "FLOAT"],
                specific_metrics, pk = "pkg_id, metric_id",
                fk = ["pkg_id", "metric_id"],
                fk_ref = ["Packages(id)", "Metrics(id)"])
        specific_metrics = None
        dbu.updateTable("FileClassMetrics",
                ["file_id", "class_name", "line", "metric_id", "value"],
                ["SMALLINT(6)", "VARCHAR(50)", "MEDIUMINT(9)",
                    "SMALLINT(6)", "FLOAT"],
                specific_metrics,
		        pk = "file_id, class_name, line, metric_id",
                fk = ["file_id", "metric_id"],
                fk_ref = ["Files(id)", "Metrics(id)"])
        specific_metrics = None
        dbu.updateTable("FileFunctionMetrics",
                ["file_id", "function_name", "line", "metric_id", "value"],
                ["SMALLINT(6)", "VARCHAR(100)", "MEDIUMINT(9)",
                    "SMALLINT(6)", "FLOAT"],
                specific_metrics,
		        pk = "file_id, function_name, line, metric_id",
                fk = ["file_id", "metric_id"],
                fk_ref = ["Files(id)", "Metrics(id)"])


def update_coding_standards(dbu, update_deps = False, calculate = False):
    # Read standards from file
    print "Reading coding standards from file..."
    # TODO
    # Recalculate non-compliance to standards
    if calculate:
        print "Determining code compliance..."
        # TODO
    # Update all standards tables
    rules_info = None
    print "Updating Rules table, rows:", 0
    dbu.updateTable("Rules", ["id", "category", "isVerifiable", "compliance"],
            ["SMALLINT(6)", "VARCHAR(100)", "TINYINT(1)", "VARCHAR(30)"],
            rules_info, pk = "id")
    if update_deps:
        noncompliance = None
        dbu.updateTable("FileNonCompliance", ["file_id", "rule_id", "count"],
                ["SMALLINT(6)", "SMALLINT(6)", "MEDIUMINT(9)"],
                noncompliance, pk = "file_id, rule_id",
                fk = ["file_id", "rule_id"],
                fk_ref = ["Files(id)", "Rules(id)"])



def export_packages(dbu):
    print "Writing package data to packages.json"
    f = open('packages.json', 'w')
    f.write(j_exp.jsonifyPackages(dbu.cur))
    f.close()




def updateDbAndExport(chronicle = False, writeJson = False,
        download = True):
    print "Updating Repos dictionary"
    Repos_dict = repo.makeReposDict("distribution.core.yaml")

    # Download source code from the repositories
    src_root = os.path.join(os.path.expanduser("~"), "ros", "repos")
    if download:
        print "Cloning repositories to", src_root
        crep.clone_repos(Repos_dict, src_root)

    # Recreate package objects dictionary from manifests and a set of all people
    print "Retrieving packages and people"
    packages = package.get_packages_from_repos(src_root, Repos_dict)
    people = extractor.allPkgPpl(packages)

    # Map packages to local source files
    print "Finding source files"
    src_dict = sf.find_source_files(src_root, packages)

    # Establish database connection
    print "\nConnecting to database"
    # con, cur = dbc.conCur(user='smartroswiki@az.engr.oregonstate.edu',
        # passwd='p0tyYjIr',
        # db='smartroswiki',host='engr-db.engr.oregonstate.edu',port=3307)
    db_user = load_db_user("dbuser.txt")
    con, cur = dbc.conCur(user = db_user["user"],
            passwd = db_user["passwd"],
            db = db_user["db"],
            host = db_user["host"],
            port = db_user["port"])

    # Define what information will be extracted for pkgs & ppl and subsequently the column names
    pkg_cols = ['name','isMetapackage','description',
            'wiki','git','branch', 'path']
    ppl_cols = ['name','email']
    # Extract desired information
    pkg_info = extractor.getPkgInfo(packages, pkg_cols)
    ppl_info = extractor.getPplInfo(packages, people, ppl_cols)
    # Append the id column to the front of the col lists
    pkg_cols.insert(0,'id')
    ppl_cols.insert(0,'id')
    # Define table column data types, update table changes col types if they differ, includes id
    pkg_types = ['SMALLINT(6)','VARCHAR(100)','TINYINT(1)','VARCHAR(2000)',
                    'VARCHAR(100)','VARCHAR(100)','VARCHAR(50)',
                    'VARCHAR(200)']
    ppl_types = ['SMALLINT(6)','VARCHAR(150)','VARCHAR(50)']
    
    

    # Information tables:


    
    # Update the Packages table, no foreign keys
    print "\nUpdating Packages table, rows:", len(pkg_info)
    dbc.upTruncTable(con, cur, 'Packages', pkg_cols, pkg_types, pkg_info, pk='id')
    # Update the Licenses table, no foreign keys
    licenses = extractor.getLicenses(packages)
    print "\nMaking Licenses table, rows:", len(licenses)
    dbc.upTruncTable(con,cur,'Licenses',['id','name'],
                    ['SMALLINT(6)','VARCHAR(150)'], licenses, pk='id')
    # Update the People table, no foreign keys
    print "\nUpdating People table, rows:", len(ppl_info)
    dbc.upTruncTable(con, cur, 'People', ppl_cols, ppl_types, ppl_info, pk='id')
    
    

    # Link Packages table to People through mapping tables, and Packages to Packages
    # Retrieve package/people primary key id and name
    pkg_ids = dbe.getTable(cur, 'Packages', ['id','name'])
    license_ids = dbe.getTable(cur, 'Licenses',['id','name'])
    ppl_ids = dbe.getTable(cur, 'People', ['id','name'])
    # Use primary keys of Packages and People for linking and mapping tables
    dep_type_ids = [(1,'buildtool_depend'),(2,'build_depend'),(3,'run_depend')]
    pkg_deps = extractor.mapPkgDeps(pkg_ids, dep_type_ids, packages)
    pkg_licenses = extractor.mapPkgLicenses(pkg_ids, license_ids, packages)
    pkg_authors, pkg_mntnrs = extractor.mapPkgAMs(pkg_ids, ppl_ids, packages)
    # Create the foreign key mapping or linked tables
    print "Updating PkgDepTypes, dependency key table for PkgDeps"
    dbc.upTruncTable(con,cur,'PkgDepTypes', ['id','dep_type'],
                    ['SMALLINT(6)','VARCHAR(16)'],dep_type_ids,pk='id')
    print "Updating PkgDeps table, mapping table for Packages, rows:", len(pkg_deps)
    dep_cols = ['pkg_id','dep_id','type_id']
    dep_col_t = ['SMALLINT(6)', 'SMALLINT(6)', 'SMALLINT(6)']
    dbc.upTruncTable(con,cur,'PkgDeps',dep_cols,dep_col_t,pkg_deps,
					pk='pkg_id, dep_id, type_id',
					fk=dep_cols,
                    fk_ref=['Packages(id)','Packages(id)','PkgDepTypes(id)'])
    print "Updating PkgLicenses table, mapping table for Packages & Licenses, rows:", len(pkg_licenses)
    lic_cols = ['pkg_id','lic_id']
    lic_col_t = ['SMALLINT(6)', 'SMALLINT(6)']
    dbc.upTruncTable(con,cur,'PkgLicenses',lic_cols, lic_col_t, pkg_licenses,
					 pk='pkg_id, lic_id',
                     fk=lic_cols, fk_ref=['Packages(id)','Licenses(id)'])
    print "Updating PkgAuthors & PkgMntnrs tables, mapping tables for Packages & People"
    am_cols = ['pkg_id','ppl_id']
    am_col_t = ['SMALLINT(6)', 'SMALLINT(6)']
    am_fk_refs = ['Packages(id)','People(id)']
    dbc.upTruncTable(con,cur, 'PkgAuthors', am_cols, am_col_t, pkg_authors,
					 pk = 'pkg_id, ppl_id',
                     fk = am_cols, fk_ref = am_fk_refs)
    dbc.upTruncTable(con,cur, 'PkgMntnrs', am_cols, am_col_t, pkg_mntnrs,
					 pk = 'pkg_id, ppl_id',
                     fk = am_cols, fk_ref = am_fk_refs)

    # Set up Repos table first, to link with Packages table after
    # Retrieve all unique repository names of all the packages
    print "\nRetrieving repository information"
    repo_names = extractor.getRepoNames(packages)
    # Define the keys that specify what values will be retrieved for each repository
    repo_keys = [['owner', 'type'], 'created_at', 'updated_at', 'pushed_at', 
                  'size', 'forks_count', 'watchers_count', 'subscribers_count']
    repo_info = extractor.getRepoInfo(repo_names, repo_keys)
    # Define the column names and types for the Repos table, includes id
    repo_cols = ['id','name','owner_type','created_at','updated_at','pushed_at',
                 'size','forks_count','watchers_count','subscribers_count']
    repo_types = ['SMALLINT(6)','VARCHAR(60)','VARCHAR(30)','CHAR(10)','CHAR(10)',
                  'CHAR(10)','MEDIUMINT(9)','SMALLINT(6)','SMALLINT(6)','SMALLINT(6)']
    # Create the Repos table
    print "Updating Repos table, rows:", len(repo_info)
    dbc.upTruncTable(con, cur, 'Repos', repo_cols, repo_types, repo_info, pk='id')



    repo_ids = dbe.getTable(cur, table='Repos', cols=['id','name'])
    # Mapping table for Packages and Repos tables, many to many
    pkg_repos = extractor.mapPkgRepos(pkg_ids, repo_ids, packages)
    print "Updating PkgRepos, mapping table for Packages and Repos, rows:", len(pkg_repos)
    pr_cols = ['pkg_id','repo_id']
    pr_types = ['SMALLINT(6)', 'SMALLINT(6)']
    dbc.upTruncTable(con, cur, 'PkgRepos', pr_cols, pr_types, pkg_repos,
					pk = 'pkg_id, repo_id',
                    fk = pr_cols, fk_ref = ['Packages(id)', 'Repos(id)'])
    


    if download:
        # Set up to make Issues table linked to Repos table
        issues_info_keys = [['user', 'login'],
                            ['assignee', 'login'],
                            'created_at', 'closed_at', 
                            ['labels', 'name']]
        print "\nRetrieving repository issues"
        issues_info = extractor.getIssuesLinkRepos(repo_ids, issues_info_keys)
        # cols used for api queries, labels later removed for separate table
        issue_cols = ['id', 'created_by', 'assigned_to', 'created_at',
                'closed_at','labels','repo_id']
        (issue_cols, issues_info, 
         label_ids, issues_labels) = extractor.getIssuesLabels(issue_cols, 
                                                               issues_info)
        issue_types = ['MEDIUMINT(9)','VARCHAR(50)','VARCHAR(50)',
                'CHAR(10)','CHAR(10)','SMALLINT(6)']
        print "Updating Issues table, linked to Repos, rows:", len(issues_info)
        dbc.upTruncTable(con,cur, 'Issues', issue_cols, issue_types, issues_info,
                        pk='id', fk='repo_id', fk_ref='Repos(id)')
        print "Updating Labels table, rows:", len(label_ids)
        dbc.upTruncTable(con, cur, 'Labels', ['id','name'],
                        ['SMALLINT(6)','VARCHAR(50)'], label_ids, pk='id')
        print "Updating IssuesLabels table, rows:", len(issues_labels)
        il_cols = ['issue_id','label_id']
        il_types = ['MEDIUMINT(9)', 'SMALLINT(6)']
        dbc.upTruncTable(con, cur, 'IssuesLabels',
                        il_cols, il_types, issues_labels,
					    pk='issue_id, label_id',
                        fk=il_cols, fk_ref=['Issues(id)','Labels(id)'])


        # Record the email and username of distinct users that created issues
        print "Finding emails for Git Users"
        git_names = dbe.getDistinct(cur, 'Issues', ['created_by'])
        git_user_info = extractor.getGitEmails(git_names)
        print "Updating GitUsers Table, rows:", len(git_user_info)
        dbc.upTruncTable(con,cur, 'GitUsers',
                ['id','username','email'],
                ['SMALLINT(6)','VARCHAR(50)','VARCHAR(50)'],
                git_user_info, pk='id')

        # Link the git users to the people from package manifests
        git_emails = dbe.getTable(cur, 'GitUsers',['id','email'])
        ppl_emails = dbe.getTable(cur, 'People', ['id', 'email'])
        git_ppl = extractor.mapGitPpl(git_emails, ppl_emails)
        print "Updating GitPpl, mapping table GitUsers to People, rows:", len(git_ppl)
        git_ppl_cols = ['id', 'git_id','ppl_id']
        gp_col_t = ['SMALLINT(6)', 'SMALLINT(6)','SMALLINT(6)']
        dbc.upTruncTable(con,cur,'GitPpl',
                git_ppl_cols, gp_col_t, git_ppl, 
                pk='id', fk=git_ppl_cols[1:],
                fk_ref = ['GitUsers(id)','People(id)'])



    # Tables for source files, code metrics and coding standards:

    # Use primary keys of Packages to map files
    src_info = extractor.getPkgFiles(pkg_ids, src_dict)
    dbc.upTruncTable(con,cur,'Files',['id','pkg_id','name','path'],
        ['SMALLINT(6)','SMALLINT(6)','VARCHAR(50)','VARCHAR(200)'],
        src_info, pk='id', fk=['pkg_id'], fk_ref=['Packages(id)'])

    dbc.upTruncTable(con,cur,'Metrics',['id','name','description'],
        ['SMALLINT(6)','VARCHAR(50)','VARCHAR(2000)'], None, pk='id')
    dbc.upTruncTable(con,cur,'FileMetrics',['file_id','metric_id','value'],
        ['SMALLINT(6)','SMALLINT(6)','FLOAT'], None,
		pk='file_id, metric_id',
        fk=['file_id','metric_id'], fk_ref=['Files(id)','Metrics(id)'])
    dbc.upTruncTable(con,cur,'PackageMetrics',['pkg_id','metric_id','value'],
        ['SMALLINT(6)','SMALLINT(6)','FLOAT'], None,
		pk='pkg_id, metric_id',
        fk=['pkg_id','metric_id'], fk_ref=['Packages(id)','Metrics(id)'])
    dbc.upTruncTable(con,cur,'FileClassMetrics',
        ['file_id','class_name','line','metric_id','value'],
        ['SMALLINT(6)','VARCHAR(50)','MEDIUMINT(9)','SMALLINT(6)','FLOAT'], None,
		pk='file_id, class_name, line, metric_id',
        fk=['file_id','metric_id'], fk_ref=['Files(id)','Metrics(id)'])
    dbc.upTruncTable(con,cur,'FileFunctionMetrics',
        ['file_id','function_name','line','metric_id','value'],
        ['SMALLINT(6)','VARCHAR(100)','MEDIUMINT(9)','SMALLINT(6)','FLOAT'], None,
		pk='file_id, function_name, line, metric_id',
        fk=['file_id','metric_id'], fk_ref=['Files(id)','Metrics(id)'])

    dbc.upTruncTable(con,cur,'Rules',['id','category','isVerifiable','compliance'],
        ['SMALLINT(6)','VARCHAR(100)','TINYINT(1)','VARCHAR(30)'], None, pk='id')
    dbc.upTruncTable(con,cur,'FileNonCompliance',['file_id','rule_id','count'],
        ['SMALLINT(6)','SMALLINT(6)','MEDIUMINT(9)'], None,
		pk='file_id, rule_id',
        fk=['file_id','rule_id'], fk_ref=['Files(id)','Rules(id)'])



    # Scoring tables:


    # Score packages on impact
    # pkg_only_ids = dbe.getTable(cur, 'Packages', ['id'])
    # repo_only_ids = dbe.getTable(cur, 'Repos', ['id'])
    # ppl_only_ids = dbe.getTable(cur, 'People', ['id'])
    # git_user_ids = dbe.getTable(cur,'GitUsers',['id'])
    # git_ppl_ids = dbe.getTable(cur,'GitPpl',['id'])
    


    # print "\nScoring package impact"
    # pkg_impact = scorer.getPkgsImpact(pkg_only_ids, cur)
    # print "Scoring package health"
    # pkg_health = scorer.getPkgsHealth(pkg_only_ids, cur)
    # print "Scoring repo health"
    # repo_health = scorer.getReposHealth(repo_only_ids, cur)
    # print "Scoring git user Activity"
    # user_activity = scorer.getGitUsersActivity(git_user_ids, cur)



    # print "\nUpdating PkgImpact table, linked to Packages"
    # dbc.upTruncTable(con, cur,'PkgImpact',['pkg_id','impact'],
                    # ['SMALLINT(6)','FLOAT'], pkg_impact, 
                    # fk='pkg_id', fk_ref='Packages(id)')
    
    # print "Updating PkgHealth table, linked to Packages"
    # dbc.upTruncTable(con, cur, 'PkgHealth', ['pkg_id','health'],
                    # ['SMALLINT(6)','FLOAT'], pkg_health,
                    # fk='pkg_id', fk_ref='Packages(id)')

    # print "Updating RepoHealth table, linked to Repos"
    # dbc.upTruncTable(con, cur, 'RepoHealth', ['repo_id','health'],
                    # ['SMALLINT(6)','FLOAT'],repo_health,
                    # fk='repo_id',fk_ref='Repos(id)')

    # print "Updating GitActivity table, linked to GitUsers"
    # dbc.upTruncTable(con, cur,'GitActivity',['git_id','activity'],
                    # ['SMALLINT(6)','FLOAT'], user_activity, 
                    # fk='git_id', fk_ref='GitUsers(id)')



    # Metrics below use the values in the tables above
    # print "\nScoring people impact"
    # ppl_impact = scorer.getPplImpact(ppl_only_ids, cur)
    # print "Scoring repo package composite health"
    # pkg_repo_health = scorer.getPkgsReposHealth(pkg_only_ids, repo_only_ids, cur)
    

    # print "\nUpdating PplImpact table, linked to People"
    # dbc.upTruncTable(con, cur, 'PplImpact', ['ppl_id','impact'],
                    # ['SMALLINT(6)','FLOAT'], ppl_impact, 
                    # fk='ppl_id', fk_ref='People(id)')


    # print "\nScoring git ppl impactivity composite ppl impact & git activity"
    # impactivity_scores = scorer.getImpactivity(git_ppl_ids, cur)
    # print "\nUpdating GitPplImpactivity, composite score table of PplImpact & GitActivity"
    # dbc.upTruncTable(con,cur,'GitPplImpactivity',['git_ppl_id','impactivity'],
                     # ['SMALLINT(6)','FLOAT'], impactivity_scores, 
                     # fk='git_ppl_id', fk_ref='GitPpl(id)')
    
    # print "Updating PkgRepoHealth table, linked to Packages",
    # print "a weighted combination of PkgHealth and RepoHealth scores"
    # dbc.upTruncTable(con, cur, 'PkgRepoHealth', ['pkg_id','health'],
                    # ['SMALLINT(6)','FLOAT'],pkg_repo_health, 
                    # fk = 'pkg_id', fk_ref = 'Packages(id)')


    # if chronicle:
        # print "\nRecording _Hist tables"
        # chro.chronicle(con, cur)


    if writeJson:
        print "\nWriting package data to packages.json"
        f = open('packages.json', 'w')
        f.write(j_exp.jsonifyPackages(cur))
        f.close()





def update_local_packages():
    print "Updating Repos dictionary"
    Repos_dict = repo.makeReposDict("distribution.core.yaml")

    src_root = os.path.join(os.path.expanduser("~"), "ros", "repos")

    # Recreate package objects dictionary from manifests and a set of all people
    print "Retrieving packages and people"
    packages = package.get_packages_from_repos(src_root, Repos_dict)
    people = extractor.allPkgPpl(packages)

    # Map packages to local source files
    print "Finding source files"
    src_dict = sf.find_source_files(src_root, packages)

    # Establish database connection
    print "\nConnecting to database"
    db_user = load_db_user("dbuser.txt")
    con, cur = dbc.conCur(user = db_user["user"],
            passwd = db_user["passwd"],
            db = db_user["db"],
            host = db_user["host"],
            port = db_user["port"])

    # Define what information will be extracted for pkgs & ppl and subsequently the column names
    pkg_cols = ['name','isMetapackage','description',
                'wiki','git','branch', 'path']
    ppl_cols = ['name','email']
    # Extract desired information
    pkg_info = extractor.getPkgInfo(packages, pkg_cols)
    ppl_info = extractor.getPplInfo(packages, people, ppl_cols)
    # Append the id column to the front of the col lists
    pkg_cols.insert(0,'id')
    ppl_cols.insert(0,'id')
    # Define table column data types, update table changes col types if they differ, includes id
    pkg_types = ['SMALLINT(6)','VARCHAR(100)','TINYINT(1)','VARCHAR(2000)',
                    'VARCHAR(100)','VARCHAR(100)','VARCHAR(50)',
                    'VARCHAR(200)']
    ppl_types = ['SMALLINT(6)','VARCHAR(150)','VARCHAR(50)']

    # Update the Packages table, no foreign keys
    print "\nUpdating Packages table, rows:", len(pkg_info)
    dbc.upTruncTable(con, cur, 'Packages', pkg_cols, pkg_types,
            pkg_info, pk='id')
    # Update the Licenses table, no foreign keys
    licenses = extractor.getLicenses(packages)
    print "\nMaking Licenses table, rows:", len(licenses)
    dbc.upTruncTable(con,cur,'Licenses',['id','name'],
                    ['SMALLINT(6)','VARCHAR(150)'], licenses, pk='id')
    # Update the People table, no foreign keys
    print "\nUpdating People table, rows:", len(ppl_info)
    dbc.upTruncTable(con, cur, 'People', ppl_cols, ppl_types,
            ppl_info, pk='id')

    # Link Packages table to People through mapping tables, and Packages to Packages
    # Retrieve package/people primary key id and name
    pkg_ids = dbe.getTable(cur, 'Packages', ['id','name'])
    license_ids = dbe.getTable(cur, 'Licenses',['id','name'])
    ppl_ids = dbe.getTable(cur, 'People', ['id','name'])
    # Use primary keys of Packages and People for linking and mapping tables
    dep_type_ids = [(1,'buildtool_depend'),(2,'build_depend'),(3,'run_depend')]
    pkg_deps = extractor.mapPkgDeps(pkg_ids, dep_type_ids, packages)
    pkg_licenses = extractor.mapPkgLicenses(pkg_ids, license_ids, packages)
    pkg_authors, pkg_mntnrs = extractor.mapPkgAMs(pkg_ids, ppl_ids, packages)
    # Create the foreign key mapping or linked tables
    print "Updating PkgDepTypes, dependency key table for PkgDeps"
    dbc.upTruncTable(con,cur,'PkgDepTypes', ['id','dep_type'],
                    ['SMALLINT(6)','VARCHAR(16)'],dep_type_ids,pk='id')
    print "Updating PkgDeps table, mapping table for Packages, rows:", len(pkg_deps)
    dep_cols = ['pkg_id','dep_id','type_id']
    dep_col_t = ['SMALLINT(6)', 'SMALLINT(6)', 'SMALLINT(6)']
    dbc.upTruncTable(con,cur,'PkgDeps',dep_cols,dep_col_t,pkg_deps,
					pk='pkg_id, dep_id, type_id',
					fk=dep_cols,
                    fk_ref=['Packages(id)','Packages(id)','PkgDepTypes(id)'])
    print "Updating PkgLicenses table, mapping table for Packages & Licenses, rows:", len(pkg_licenses)
    lic_cols = ['pkg_id','lic_id']
    lic_col_t = ['SMALLINT(6)', 'SMALLINT(6)']
    dbc.upTruncTable(con,cur,'PkgLicenses',lic_cols, lic_col_t, pkg_licenses,
					 pk='pkg_id, lic_id',
                     fk=lic_cols, fk_ref=['Packages(id)','Licenses(id)'])
    print "Updating PkgAuthors & PkgMntnrs tables, mapping tables for Packages & People"
    am_cols = ['pkg_id','ppl_id']
    am_col_t = ['SMALLINT(6)', 'SMALLINT(6)']
    am_fk_refs = ['Packages(id)','People(id)']
    dbc.upTruncTable(con,cur, 'PkgAuthors', am_cols, am_col_t, pkg_authors,
					 pk = 'pkg_id, ppl_id',
                     fk = am_cols, fk_ref = am_fk_refs)
    dbc.upTruncTable(con,cur, 'PkgMntnrs', am_cols, am_col_t, pkg_mntnrs,
					 pk = 'pkg_id, ppl_id',
                     fk = am_cols, fk_ref = am_fk_refs)

    # Tables for source files, code metrics and coding standards:

    # Use primary keys of Packages to map files
    src_info = extractor.getPkgFiles(pkg_ids, src_dict)
    print "Updating Files table, rows:", len(src_info)
    dbc.upTruncTable(con,cur,'Files',['id','pkg_id','name','path'],
        ['SMALLINT(6)','SMALLINT(6)','VARCHAR(50)','VARCHAR(200)'],
        src_info, pk='id', fk=['pkg_id'], fk_ref=['Packages(id)'])

    dbc.upTruncTable(con,cur,'Metrics',['id','name','description'],
        ['SMALLINT(6)','VARCHAR(50)','VARCHAR(2000)'], None, pk='id')
    dbc.upTruncTable(con,cur,'FileMetrics',['file_id','metric_id','value'],
        ['SMALLINT(6)','SMALLINT(6)','FLOAT'], None,
		pk='file_id, metric_id',
        fk=['file_id','metric_id'], fk_ref=['Files(id)','Metrics(id)'])
    dbc.upTruncTable(con,cur,'PackageMetrics',['pkg_id','metric_id','value'],
        ['SMALLINT(6)','SMALLINT(6)','FLOAT'], None,
		pk='pkg_id, metric_id',
        fk=['pkg_id','metric_id'], fk_ref=['Packages(id)','Metrics(id)'])
    dbc.upTruncTable(con,cur,'FileClassMetrics',
        ['file_id','class_name','line','metric_id','value'],
        ['SMALLINT(6)','VARCHAR(50)','MEDIUMINT(9)','SMALLINT(6)','FLOAT'], None,
		pk='file_id, class_name, line, metric_id',
        fk=['file_id','metric_id'], fk_ref=['Files(id)','Metrics(id)'])
    dbc.upTruncTable(con,cur,'FileFunctionMetrics',
        ['file_id','function_name','line','metric_id','value'],
        ['SMALLINT(6)','VARCHAR(100)','MEDIUMINT(9)','SMALLINT(6)','FLOAT'], None,
		pk='file_id, function_name, line, metric_id',
        fk=['file_id','metric_id'], fk_ref=['Files(id)','Metrics(id)'])

    dbc.upTruncTable(con,cur,'Rules',['id','category','isVerifiable','compliance'],
        ['SMALLINT(6)','VARCHAR(100)','TINYINT(1)','VARCHAR(30)'], None, pk='id')
    dbc.upTruncTable(con,cur,'FileNonCompliance',['file_id','rule_id','count'],
        ['SMALLINT(6)','SMALLINT(6)','MEDIUMINT(9)'], None,
		pk='file_id, rule_id',
        fk=['file_id','rule_id'], fk_ref=['Files(id)','Rules(id)'])





if __name__ == '__main__':
    import sys
    from datetime import datetime
    startTime = datetime.now()
    if len(sys.argv) > 1 and sys.argv[1] == "local":
        update_local_packages()
    else:
        updateDbAndExport(chronicle = True, 
                writeJson = True, download = True)
    print "\nExecution time: ", (datetime.now()-startTime), '\n'
    
    # dbu = DbUpdater()
    # dbu.connect("dbuser.txt")
    # if dbu.con is None or dbu.cur is None:
        # print "Failure!"
    # update_source_files(dbu, trunc_deps = True)
    # update_code_metrics(dbu, update_deps = True, calculate = True)
    # update_coding_standards(dbu, update_deps = True, calculate = True)
    # export_packages(dbu)
