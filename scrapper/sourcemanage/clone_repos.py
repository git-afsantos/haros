import os
import shutil
import subprocess

# mkdir $BRANCH
# cd $BRANCH
# git init
# git remote add -t $BRANCH -f origin $REMOTE_REPO
# git checkout $BRANCH

def clone_repo(repo, clone):
    if clone:
        print "  Cloning {0}...".format(repo.name)
        subprocess.check_call(["git", "init"])
        subprocess.check_call(["git", "remote", "add", "-t",
                repo.source_version, "-f", "origin", repo.source_url])
        subprocess.check_call(["git", "checkout", repo.source_version])
    else:
        print "  Updating {0}...".format(repo.name)
        subprocess.check_call(["git", "pull"])


def clone_repo_to_path(repo, repos_path):
    path = os.path.join(repos_path, repo.name)
    clone = False
    if not os.path.exists(path):
        os.makedirs(path)
        clone = True
    if not repo.source_url is None:
        os.chdir(path)
        clone_repo(repo, clone)


def remove_previous_repos(repos_path):
    if os.path.exists(repos_path):
        shutil.rmtree(repos_path)


def clone_repos(repos_dict, repos_path):
    # remove_previous_repos(repos_path)
    origWD = os.getcwd()
    for repo in repos_dict.values():
        if not repo.source_url is None:
            clone_repo_to_path(repo, repos_path)
    os.chdir(origWD)

