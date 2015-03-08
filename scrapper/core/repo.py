# Modified by
# Andre Santos, November 2014

from yaml import load

# Perhaps just a set of sets?

class Repo:
    def __init__(self, name):
        self.id = None
        self.name = name # The name listed in the distribution file
        self.urls = None 
        self.repo_names = None # The repositories names on github
        self.status = None
        self.release_version = None
        self.source_url = None
        self.source_version = None
        self.subpackages = []
        self.filtered_packages = []

    def addUrlAndRepoName(self, url): 
        if 'git' not in url: return # Git API query won't work on non git
        
        repo_name = url.replace('https://github.com/','').replace('.git','')
        if self.urls == None:
            self.urls = set([url])
            self.repo_names = set([repo_name])
        else:
            self.urls.add(url)
            self.repo_names.add(repo_name)

    def hasSubpackages(self):
        return len(self.subpackages) > 0


"""
def get_repos_from_dist(dist_file):
    with open(dist_file, 'r') as open_file:
        dist_data = load(open_file)
    repo_data = dist_data['repositories']
    repo_dict = {}
    for repo_name, data in repo_data.iteritems():
        repo = Repo(repo_name)
        for to_check in ['doc','source','release']:
            if to_check in data:
                nstd = data[to_check]
                if 'url' in nstd:
                    repo.addUrlAndRepoName(nstd['url'])
                    if to_check == 'source':
                        repo.source_url = nstd['url']
                if 'version' in nstd:
                    if to_check == 'release':
                        repo.release_version = nstd['version']
                    elif to_check == 'source':
                        repo.source_version = nstd['version']
                if to_check == 'release':
                    subs = nstd.get('packages')
                    if not subs is None:
                        repo.subpackages = subs
        if 'status' in data:
            repo.status = data['status']
        if not repo.urls is None:
            repo_dict[repo_name] = repo
    return repo_dict
"""

def get_repos_from_dist(dist_file, filter_file = None):
    with open(dist_file, "r") as open_file:
        dist_data = load(open_file)
    dist_data = dist_data["repositories"]
    if filter_file is None:
        return repos_from_dist(dist_data)
    else:
        with open(filter_file, "r") as open_file:
            filter_data = load(open_file)
        filter_data = filter_data["packages"]
        return filter_distribution(dist_data, filter_data)


def filter_distribution(dist, filt):
    repos = {}
    for key, val in filt.iteritems():
        if key in dist:
            repo = repo_from_dist(key, dist)
            for p in repo.subpackages:
                if p in val:
                    repo.filtered_packages.append(p)
            repos[key] = repo
    return repos


def repos_from_dist(dist):
    repos = {}
    for key in dist:
        repo = repo_from_dist(key, dist)
        repo.filtered_packages = repo.subpackages
        repos[key] = repo
    return repos


def repo_from_dist(repo_name, dist):
    data = dist[repo_name]
    repo = Repo(repo_name)
    for to_check in ['doc','source','release']:
        if to_check in data:
            nstd = data[to_check]
            if 'url' in nstd:
                repo.addUrlAndRepoName(nstd['url'])
                if to_check == 'source':
                    repo.source_url = nstd['url']
            if 'version' in nstd:
                if to_check == 'release':
                    repo.release_version = nstd['version']
                elif to_check == 'source':
                    repo.source_version = nstd['version']
            if to_check == 'release':
                subs = nstd.get('packages')
                if not subs is None:
                    repo.subpackages = subs
    if 'status' in data:
        repo.status = data['status']
    if not repo.urls is None:
        return repo
    return None

