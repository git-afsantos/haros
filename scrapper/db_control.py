# Modified by
# Andre Santos, December 2014

import pymysql as mdb # export PATH=$PATH:/usr/local/mysql/bin
import db_extract as dbe
import sys

prefix = 'eco_'

def namePref(name):
	if name is None:
		return None
	elif type(name) == list:
		return [prefix + table_name for table_name in name if prefix not in table_name]
	elif prefix not in name:
		return prefix + name
	else:
		return name

def conCur(user='root',passwd='root_pwd',db='ROSdb',host='localhost',port=None):
	# establishes a connection and cursor with a database
	if port != None:
		print user, passwd, db, host, port
		con = mdb.connect(user=user, passwd=passwd, db=db, host=host, port=port)
	else:
		con = mdb.connect(user=user, passwd=passwd, db=db, host=host)
	cur = con.cursor() # cursor is used to traverse the records from the result set
	return con, cur

def tableExists(cur, name):
	name = namePref(name)

	cmd = """SHOW TABLES """
	all_tables = dbe.exFetch(cur, cmd)

	for t in all_tables:
		if t[0].lower() == name.lower(): # Name at zeroth index
			return True
	return False

def setFKChecks(cur, set_to):
	cmd = """SET FOREIGN_KEY_CHECKS = {0}""".format(set_to)
	cur.execute(cmd)

def truncateTable(cur, name):
	name = namePref(name)

	setFKChecks(cur, 0)
	cmd = """TRUNCATE {0}""".format(name)
	cur.execute(cmd)
	setFKChecks(cur, 1)

def insertCmd(name, col_names):
	name = namePref(name)

	cmd = """INSERT INTO {0} (""".format(name)
	cmd2 = ""
	for idx,name in enumerate(col_names):
		cmd += name + ", "
		cmd2 += "%s, "
	return cmd[:-2] + ") VALUES(" + cmd2[:-2] + ")"

def insertOrUpdateCmd(name, col_names, pk_cols):
	name = namePref(name)

	cmd = """INSERT INTO {0} (""".format(name)
	cmd2 = ""
	cmd3 = ""
	sep3 = ""
	for idx,name in enumerate(col_names):
		cmd += name + ", "
		cmd2 += "%s, "
		if not name in pk_cols:
		    cmd3 += sep3 + name + "=VALUES(" + name + ")"
		    sep3 = ", "
	return cmd[:-2] + ") VALUES(" + cmd2[:-2] + (") "
	        "ON DUPLICATE KEY UPDATE ") + cmd3

def createCmd(name, col_names, col_types, pk=None, fk=None, fk_ref=None):
	name = namePref(name)
	
	cmd = """CREATE TABLE {0} (""".format(name)
	for idx,col_name in enumerate(col_names):
		cmd += "{0} {1}, ".format(col_name, col_types[idx])
	
	if pk != None: 
		cmd += "PRIMARY KEY ({0}), ".format(pk)
	
	if fk != None and fk_ref != None:
		if type(fk) == type(['list']):
			for idx,fk_name in enumerate(fk):
				cmd += "FOREIGN KEY ({0}) REFERENCES {1}, ".format(fk_name, fk_ref[idx])
		else:
			cmd += "FOREIGN KEY ({0}) REFERENCES {1}, ".format(fk, fk_ref)
	
	return cmd[:-2] + ")"

def updateCols(con, cur, name, col_names, col_types, fk_ref):
	name = namePref(name)
	fk_ref = namePref(fk_ref)

	cmd = """SHOW COLUMNS FROM """ + name
	cols_info = dbe.exFetch(cur,cmd)
	
	for idx,info in enumerate(cols_info):
		# Name of type discrepancy
		if info[0] != col_names[idx] or info[1] != col_types[idx].lower():
			cmd = """ALTER TABLE {0} CHANGE {1} {2} {3}""".format(
				     name, info[0], col_names[idx], col_types[idx])
			print "  Updating {0} {1} --> {2} {3}".format(
				     info[0], info[1], col_names[idx], col_types[idx])
			cur.execute(cmd)
		# Foreign key discrepancy
		if idx == len(col_names)-1 and '_id' in col_names[idx] \
		and fk_ref != None and info[3] == '':
			cmd = """ALTER TABLE {0} ADD FOREIGN KEY ({1}) REFERENCES {2}""".format(
				     name, col_names[-1], fk_ref)
			print "  Updating Foreign Key {0} Ref {1}".format(col_names[-1], fk_ref)
			cur.execute(cmd)

	if len(cols_info) < len(col_names): # only supports adding columns
		cmd = "ALTER TABLE {0} ADD {1} {2} NULL AFTER {3}".format(
			      name, col_names[-1], col_types[-1], cols_info[-1][0])
		cur.execute(cmd)

	con.commit()

def upTruncTable(con, cur, name, col_names, col_types, data, 
	            pk = None, fk = None, fk_ref = None):
	
	name = namePref(name)
	fk_ref = namePref(fk_ref)

	if tableExists(cur, name) is False:
		print "  {0} does not exist, creating table...".format(name)
		cmd = createCmd(name, col_names, col_types, pk, fk, fk_ref)
		print cmd
		cur.execute(cmd)
	else:
		truncateTable(cur, name)
	
	updateCols(con, cur, name, col_names, col_types, fk_ref)
	
	if data != None: # Only insert if we actually have data
		cmd = insertCmd(name, col_names)
		cur.executemany(cmd, data)

	con.commit()

def recordTable(con, cur, name, col_names, col_types, data):
	name = namePref(name)

	if tableExists(cur, name) is False:
		print "  {0} does not exist, creating table...".format(name)
		cmd = createCmd(name, col_names, col_types)
		print cmd
		cur.execute(cmd)

	updateCols(con, cur, name, col_names, col_types, fk_ref=None)
	
	if data != None: # Only insert if we actually have data
		cmd = insertCmd(name, col_names)
		cur.executemany(cmd, data)

	con.commit()

def insertRecords(con, cur, name, col_names, data, truncate=False):
    name = namePref(name)

    if data is None or tableExists(cur, name) is False:
        return

    if truncate:
        truncateTable(cur, name)

    cmd = insertCmd(name, col_names)
    cur.executemany(cmd, data)

    con.commit()


if __name__ == '__main__':
	# con, cur = concur(user='smartroswiki', passwd='p0tyYjIr', db='smartroswiki', host='engr-db.engr.oregonstate.edu', port=3307)
	con, cur = conCur(user='dbuser',passwd='dbpass',db='test3',host='localhost',port=None)
	# print updateColTypes(con,cur,'People',['id','name','emails'],['SMALLINT(6)','VARCHAR(100)','VARCHAR(50)'],None)
	print insertCmd('Test',['id','name','emails'])
	print createCmd('Test',['id','name','emails'], ['SMALLINT(6)','VARCHAR(100)','VARCHAR(50)'],'id','ppl_id', 'People(id)')
