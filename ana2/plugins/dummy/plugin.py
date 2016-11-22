print "[dummy] loading plugin"

def file_analysis(iface, scope):
    print "Analysing file", scope.id
    iface.report_violation("roscpp_4.3", "Dummy violation for " + scope.id, line = 1)
