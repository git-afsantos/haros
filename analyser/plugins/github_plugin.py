
import json
import os
import requests
from requests.auth import HTTPBasicAuth
import subprocess
import time
import yaml


g_rate = 0
g_user = "gituser.txt"


def plugin_run(ctx):
    global g_rate
    global g_user
    #outdir = os.path.join(".", "plugin_out", "github")
    #os.makedirs(outdir)
    repos = ctx.getRepositoryInfo()
    args = ctx.getPluginArguments()
    g_user = args[0]
    meta = get_metadata("filter.yaml")
    rl, g_rate = getIssuesRateLimit()
    for r in repos:
        # TODO contributors through API
        measure_contributor_count(ctx, meta, r)
        measure_commit_count(ctx, r)
        measure_issue_count(ctx, r)
    


def measure_commit_count(ctx, repo):
    path = ctx.getPath(repo[2])
    origWD = os.getcwd()
    if os.path.exists(path):
        os.chdir(path)
        value = int(subprocess.check_output(["git", "rev-list", "HEAD", "--count"]).rstrip())
        ctx.writeRepositoryMetric(repo[0], 19, value)
        os.chdir(origWD)


def get_metadata(filter_file):
    with open(filter_file, "r") as open_file:
        filter_data = yaml.load(open_file)
    meta_data = dict()
    if "meta" in filter_data:
        meta_data = filter_data["meta"]
    return meta_data


def measure_contributor_count(ctx, meta_data, repo):    
    if repo[2] in meta_data:
        value = int(meta_data[repo[2]]["contributors"])
        ctx.writeRepositoryMetric(repo[0], 18, value)


def measure_issue_count(ctx, repo):
    global g_rate
    try:
        while g_rate == 0:
            print "[Network] Reached GitHub rate limit. Waiting for availability..."
            time.sleep(65)
            rl, g_rate = getIssuesRateLimit()
        oissues, cissues = getIssuesCount(repo[1])
        ctx.writeRepositoryMetric(repo[0], 20, oissues)
        ctx.writeRepositoryMetric(repo[0], 21, cissues)
        g_rate -= 2
    except:
        pass




# Query GitHub ---------------------------------------------

def executeQuery(query):
    with open(g_user) as f:
        lines = [line.strip() for line in f]
    req = requests.get(query,
            auth = HTTPBasicAuth(lines[0], lines[1]), verify = False)
    result = json.loads(req.text)
    return result

# Receives "owner/repo".
# Returns (# open issues, # closed issues).
def getIssuesCount(repo_name):
    oissues = 0
    cissues = 0
    issue_query = "https://api.github.com/search/issues?q=repo:" + repo_name
    issues = executeQuery(issue_query + "+state:open")
    if "total_count" in issues:
        oissues = issues["total_count"]
    else:
        print "could not find issues for", repo_name
        print issue_query + "+state:open"
        print issues
    issues = executeQuery(issue_query + "+state:closed")
    if "total_count" in issues:
        cissues = issues["total_count"]
    else:
        print "could not find issues for", repo_name
        print issue_quer + "+state:closed"
        print issues
    return (oissues, cissues)


# Returns object; "core" for normal rate, "search" for search rate.
def getRateLimit():
    return executeQuery("https://api.github.com/rate_limit")


# Returns (limit, remaining)
def getIssuesRateLimit():
    r = getRateLimit()
    r = r["resources"]["search"]
    return (r["limit"], r["remaining"])

