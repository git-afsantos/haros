from core import repo

def plugin_run():
    print "Plugin running!"
    repos = repo.get_repos_from_dist("distribution.yaml", "filter.yaml")
    print "Repositories:", len(repos)
    for key, val in repos.iteritems():
        print "  ", key
        print "    ", ", ".join(val.filtered_packages)

print "Plugin imported!"

