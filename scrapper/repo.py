# Modified by
# Andre Santos, November 2014

from yaml import load

# Perhaps just a set of sets?

class Repo:
    def __init__(self, name):
        self.name = name # The name listed in the distribution file
        self.urls = None 
        self.repo_names = None # The repositories names on github
        self.status = None
        self.release_version = None
        self.source_url = None
        self.source_version = None
        self.subpackages = []

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



def getRepoData(dist_file):
    with open(dist_file, 'r') as open_file:
        data = load(open_file)
    return data['repositories']


def makeReposDict(dist_file):
    repo_data = getRepoData(dist_file)
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



if __name__ == '__main__':
    Repos_dict = makeReposDict('distribution.yaml')
    print len(Repos_dict)
    c = [0,0,0]
    for r in Repos_dict.values():
        rel = False
        norel = False
        for url in r.urls:
            if 'release' in url:
                rel = True
                print url
            else:
                norel = True
        if rel == True and norel == False:
            c[0] += 1
        elif rel == True and norel == True:
            c[1] +=1
        elif rel == False and norel == True:
            c[2] += 1
        else:
            print "NOOOOO"
    print c
