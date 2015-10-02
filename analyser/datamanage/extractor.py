# Modified by
# Andre Santos, November 2014

import query_git as qg

import time


# Retrieves all people of every package
def allPkgPpl(packages):
    mntnrs = reduce(set.union, [p.maintainers.people for p in packages.values()])
    authors = reduce(set.union, [p.authors.people for p in packages.values()])
    people = mntnrs.union(authors)

    return people


# Data retrieval to build tables...

# Tables with only primary key, no foreign key/direct linking:

def getPkgInfo(packages, info):
    # Returns a list of tuples, each tuple describes a single package
    lot = []
    for idx,p in enumerate(packages.values(), start=1):
        l = [idx] # starts as a list, will be converted to tuple and appended
        for inf in info:
            l.append(eval('p.'+inf))
        lot.append(tuple(l))
    return lot

def getPplNames(people):
    # Retrieves general information of people based on package dict (maintainers 
    # or authors), as a list of tuples; each tuple describes a specific person
    lot = []
    for idx,p in enumerate(people, start=1):
        lot.append((idx, p.name))
    return lot

def getPplInfo(people, info):
    # Retrieves general information of people based on package dict (maintainers 
    # or authors), as a list of tuples; each tuple describes a specific person
    lot = []
    for idx,p in enumerate(people, start=1):
        l = [idx]
        for i in info:
            try:
                l.append(eval('p.'+i))
            except:
                l.append(None) # append None if attribute is absent
        lot.append(tuple(l))
    return lot

def getLicenses(packages):
    lot = []
    c = 1
    for idx,p in enumerate(packages.values(), start=1):
        for license in p.licenses:
            if not any(license in t for t in lot):
                lot.append((c,license))
                c += 1
    return lot

def getRepoNames(packages):
    # Creates a list of the unique repository names of all the packages
    l = []
    for p in packages.values():
        for repo_name in p.repo.repo_names:
            if repo_name not in l:
                l.append(repo_name)
    return l

def getRepoInfo(repo_names, repo_info_keys):
    # Retrieves the information for each repository's dictionary based on provided keys
    lot = []
    c = 1
    for repo_name in repo_names:
        repo_info = qg.getRepoInfo(repo_name, repo_info_keys)
        if repo_info.count(None) == len(repo_info_keys):
            continue # If this repo name is invalid it returns a list of None's
        t = [c, repo_name] + repo_info
        c += 1
        lot.append(t)
    return lot

def getGitEmails(git_names):
    lot = []
    for idx, git_name in enumerate(git_names, start=1):
        lot.append((idx, git_name, qg.getUserEmail(git_name)))
    return lot



# Linked tables, both a primary key and a foreign key (many to one)

def getIssuesLinkRepos(repo_ids, issues_info_keys):
    lol = []
    c = 1
    for repo_id,repo_name in repo_ids:
        issues_info = qg.getIssuesInfo(repo_name, issues_info_keys)
        for issue_info in issues_info:
            if issue_info == None: # This repo name is invalid, don't append anything
                print "   No Issues Found  :  ",repo_id, repo_name
                break
            issue_info.insert(0,c) # Id inserted in the front of the row
            c += 1
            issue_info.append(repo_id)
            lol.append(issue_info)
    return lol

def getIssuesLabels(issue_cols, issues_info):
    label_ids = []
    issues_labels = []
    c = 0
    label_idx = issue_cols.index('labels')
    del issue_cols[label_idx]
    for idx,issue_info in enumerate(issues_info):
        issue_id = issue_info[0]
        issue_labels = issue_info[label_idx]
        del issues_info[idx][label_idx]
        
        if issue_labels == None: continue
        issue_labels = issue_labels.split(',')
        
        for issue_label in issue_labels:
            if not any(issue_label in t for t in label_ids): # Add new label
                c += 1
                label_ids.append((c,issue_label))
            for label_id,label in label_ids:
                if label == issue_label:
                    issues_labels.append((issue_id,label_id))
    
    # issues_cols = column list for Issues tables after label removal
    # issues_info = info list for Issues tables after label removal
    # label_ids = id label pairing for Labels table
    # issues_labels = label id, issue id pairing for IssuesLabels map table
    return issue_cols, issues_info, label_ids, issues_labels



# Receives [(id, "owner/repo")]
# Returns [(id, # open, # closed)]
def getIssuesCount(repo_ids):
    lot = []
    i = 0
    for repo_id,repo_name in repo_ids:
        i += 1
        if i == 10:
            i = 0
            #time.sleep(60) # Sleep one minute to avoid GitHub X-rate-limit
        oissues, cissues = qg.getIssuesCount(repo_name)
        lot.append((repo_id, oissues, cissues))
    return lot



def getPkgFiles(pkg_ids, src_dict):
    lot = []
    idx = 1
    for pkg_id, pkg_name in pkg_ids:
        srcs = src_dict.get(pkg_name)
        if srcs is None:
            continue
        for src in srcs:
            lot.append((idx, pkg_id, src.name, src.path))
            idx += 1
    return lot


# Mapping Tables, only foreign keys

def mapPkgDeps(pkg_ids, dep_type_ids, packages):
    # Retrieves package dependencies as a list of tuples, each tuple describes a direct
    # dependency relation. Many to many
    lot = []
    for pkg_id, pkg_name in pkg_ids:
        for dep_type_id,dep_type in dep_type_ids:
            deps = eval("packages['"+pkg_name+"']."+dep_type)
            for dep in deps:
                dep_ids = [p[0] for p in pkg_ids if p[1] == dep]
                for dep_id in dep_ids:
                    lot.append((pkg_id,dep_id,dep_type_id))
    return lot

def mapPkgLicenses(pkg_ids, license_ids, packages):
    # Maps the many to many relation of package and license type
    lot = []
    for pkg_id,pkg_name in pkg_ids:
        for license_id,license_name in license_ids:
            for pkg_license in packages[pkg_name].licenses:
                if pkg_license == license_name:
                    lot.append((pkg_id,license_id))
    return lot


def mapPkgAMs(pkg_ids, ppl_ids, packages):
    # Retrieves package maintainers/authors as a list of tuples, each tuple 
    # describes a package maintainer/author. Many to many
    a_lot = []
    m_lot = []
    for pkg_id,pkg_name in pkg_ids:
        for ppl_id,ppl_name in ppl_ids:
            if packages[pkg_name].authors.find(ppl_name) != None:
                a_lot.append((pkg_id,ppl_id))
            if packages[pkg_name].maintainers.find(ppl_name) != None:
                m_lot.append((pkg_id,ppl_id))
    return a_lot, m_lot

def mapPkgRepos(pkg_ids, repo_ids, packages):
    # Retrieves package repository relations as a list of tuples, each tuple
    # describes a package repository relation. Many to many
    lot = []
    for pkg_id, pkg_name in pkg_ids:
        for repo_id, repo_name in repo_ids:
            if repo_name in packages[pkg_name].repo.repo_names:
                lot.append((pkg_id,repo_id))
    return lot

def mapGitPpl(git_emails, ppl_emails):
    # Maps matching github account emails with people's listed emails in
    # package manifests
    lot = []
    idx = 1
    for git_id, git_email in git_emails:
        for ppl_id, ppl_email in ppl_emails:
            if git_email == ppl_email and git_email != None:
                lot.append((idx, git_id,ppl_id))
                idx += 1
    
    return lot

