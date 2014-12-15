import xml.etree.ElementTree as ET


def read_compliance_from_xml(cmp_file):
    with open(cmp_file) as f:
        tree = ET.parse(f)
    root = tree.getroot()
    files = {}
    for warning in root:
        src_file = warning.find("absFile").text
        src_file = src_file[(src_file.find("/ros/repos/") + 11):]
        check = warning.find("checkName").text
        if not src_file in files:
            files[src_file] = {}
        rules = files[src_file]
        rules[check] = rules.get(check, 0) + 1
    return files

