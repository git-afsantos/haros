import db_control as dbc
import sys

prefix = 'eco_'

def tablePref(table):
	if prefix not in table: 
		return prefix + table
	else:
		return table

# Helper retriever
def exFetch(cur, cmd, single = False):
	try:
		cur.execute(cmd)
	except:
		print "INVALID COMMAND:", cmd
		sys.exit()
	ret = cur.fetchall()
	if single:
		return ret[0][0]
	else:
		return ret

def selectCmd(table, cols, cnt = False, dstnct = False, sums = False):
	tablePref(table)

	cmd = "SELECT "
	if cnt:    cmd += "CAST(COUNT("
	elif sums: cmd += "CAST(SUM("
	if dstnct: cmd += "DISTINCT "
	
	cmd += "{0}".format(", ".join(cols))
	if sums or cnt:   cmd += ")AS UNSIGNED)"
	cmd += " FROM {0}".format(table)

	return cmd

def whereCmd(mcol, val, like = False):
	if like:
		cmd = " WHERE {0} LIKE '%{1}%'".format(mcol, val)
	elif val == 'NULL' or val == 'NOT NULL':
		cmd = " WHERE {0} IS {1}".format(mcol, val)
	elif type(val) == str:
		cmd = " WHERE {0} = '{1}'".format(mcol, val)
	elif type(val) == int:
		cmd = " WHERE {0} = {1}".format(mcol, val)
	return cmd

def getTable(cur, table, cols, cnt = False, sums = False):
	table = tablePref(table)

	cmd = selectCmd(table, cols, cnt = cnt, sums = sums)
	if cnt or sums: return exFetch(cur, cmd, single = True)
	else: return exFetch(cur, cmd)

def getMatch(cur, table, cols, mcol, val, cnt = False, dstnct = False):
	table = tablePref(table)

	cmd = selectCmd(table, cols, cnt, dstnct) + whereCmd(mcol, val)
	ret = exFetch(cur, cmd, single = cnt)
	return ret

def getLike(cur, table, cols, mcol, val, cnt = False, dstnct = False):
	table = tablePref(table)

	cmd = selectCmd(table, cols, cnt, dstnct) + whereCmd(mcol, val, like = True)
	ret = exFetch(cur, cmd, single = cnt)
	return ret

def getDistinct(cur, table, cols, cnt = False):
	table = tablePref(table)

	cmd = selectCmd(table, cols, cnt, dstnct = True)
	ret = exFetch(cur, cmd, cnt)
	return ret

def get2Match(cur, table, mcol1, mcol2, val1, val2, cnt = False):
	table = tablePref(table)

	cmd = selectCmd(table, ['*'], cnt) + whereCmd(mcol1, val1)
	cmd += " AND" + whereCmd(mcol2, val2).replace('WHERE ','') # remove extra where
	ret = exFetch(cur, cmd, single = cnt)
	return ret

def getMatchRow(cur, table, cols, mcol, val):
	table = tablePref(table)

	cmd = selectCmd(table, cols) + whereCmd(mcol, val)
	ret = exFetch(cur, cmd)
	assert len(ret) == 1
	return ret[0]

def getMatchVal(cur, table, col, mcol, val):
	table = tablePref(table)

	if type(col) == str: col = [col]
	cmd = selectCmd(table, col) + whereCmd(mcol, val)
	ret = exFetch(cur, cmd, single = True)
	return ret

def getMatchSet(cur, table, cols, mcol, val):
	table = tablePref(table)

	s = set()
	tot = getMatch(cur, table, cols, mcol, val)
	for t in tot:
		if len(t) == 1: s.add(t[0])
		else: s.add(t)
	return s

def getMatchValFromSet(cur, table, col, mcol, val_set):
	table = tablePref(table)

	s = set()
	for val in val_set:
		if type(val) == tuple: s.add(getMatchVal(cur, table, col, mcol, val[0]))
		else: s.add(getMatchVal(cur, table, col, mcol, val))
	return s

def getMostCom(cur, table, col, top_num):
	table = tablePref(table)

	cmd = "SELECT {0}, COUNT({0}) AS c FROM {1} GROUP BY {0} ORDER BY c DESC".format(col, table)
	ret = exFetch(cur, cmd)
	return ret[:top_num]




# Used for finding dependency number orders
def getDepOrders(cur, dep_id, table = 'PkgDeps'):
	table = tablePref(table)

	# Used for impact scoring
	dep_orders = []
	cmd = """SELECT DISTINCT pkg_id FROM {0} WHERE dep_id = {1}""".format(table, dep_id)
	result = exFetch(cur, cmd)

	def addOrder(dep_orders, result):
		this_dep_order = set()
		for dep_id in result:
			if not any(dep_id in dep_order for dep_order in dep_orders):
				this_dep_order.add(dep_id[0])
		dep_orders.append(this_dep_order)
		return dep_orders

	# Seed with direct dependencies
	dep_orders = addOrder(dep_orders, result)

	while_count = 0
	while while_count < 10:
		if while_count == 0:
			cmd = cmd.replace('=','!=') + ' AND dep_id IN (' + cmd + ')'
		else:
			cmd = cmd.replace(' AND dep_id IN ', ' AND dep_id NOT IN ') + ' AND dep_id IN (' + cmd + ')'
		result = exFetch(cur, cmd)
		if len(result) == 0:
			break
		dep_orders = addOrder(dep_orders, result)
		while_count += 1

	# Only return the number of dependency for each order
	dep_order_nums = []
	if len(dep_orders) == 0:
		dep_order_nums.append(0)
	else:
		for dep_order in dep_orders:
			dep_order_nums.append(len(dep_order))
	return dep_order_nums

def writeTable(cur, table):
	table = tablePref(table)

	cols = exFetch(cur, """SHOW COLUMNS FROM {0}""".format(table))
	s = ','.join([col[0] for col in cols])
	s += '\n'
	
	lot = getTable(cur, table, '*')
	for t in lot:
		s += ','.join(str(elem).replace(',','') for elem in t) + '\n'
	
	f = open(table + '.txt', 'w')
	f.write(s)

if __name__ == '__main__':
	# con, cur = concur(user='smartroswiki', passwd='p0tyYjIr', db='smartroswiki', host='engr-db.engr.oregonstate.edu', port=3307)
	con, cur = dbc.conCur(db='test2')
	# selectCmd('PkgMntnrs', ['ppl_id'], cnt = True, dstnct = True)
	# print getTable(cur, 'PkgMntnrs', ['ppl_id'], cnt = True)