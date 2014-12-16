import os
import xml.etree.ElementTree as ET


def read_compliance_from_xml(cmp_file):
    with open(cmp_file) as f:
        tree = ET.parse(f)
    root = tree.getroot()
    files = {}
    for warning in root:
        src_file = warning.find("absFile").text
        prefix = src_file.find("/ros/repos/")
        # Ignore system and library files.
        if prefix > -1:
            src_file = src_file[(prefix + 11):]
            check = warning.find("checkName").text
            if not src_file in files:
                files[src_file] = {}
            rules = files[src_file]
            rules[check] = rules.get(check, 0) + 1
    return files


def read_compliance_files(path_to_dir):
    results = {}
    for root, subdirs, xml_files in os.walk(path_to_dir):
        for xml_file in xml_files:
            cmp_dict = read_compliance_from_xml(os.path.join(root, xml_file))
            for src_file, rules in cmp_dict.iteritems():
                for rule, value in rules.iteritems():
                    results[src_file] = results.get(src_file, 0) + value
    return results




if __name__ == "__main__":
    src_root    = os.path.join(os.path.expanduser("~"), "ros", "goanna")
    print "Reading XML files..."
    files       = read_compliance_files(src_root)
    for f, v in files.iteritems():
        print "\t[{0}]:\t{1}".format(files[f], f)

