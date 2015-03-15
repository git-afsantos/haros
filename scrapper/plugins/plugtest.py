from core import repo

def plugin_run(db):
    print "Plugin running!"
    pkgs = db.getPackageInfo()
    for pkg in pkgs:
        print "  ", pkg[0], pkg[1]

print "Plugin imported!"

