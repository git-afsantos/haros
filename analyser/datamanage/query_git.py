import requests
from requests.auth import HTTPBasicAuth
import json
from unidecode import unidecode
from difflib import get_close_matches
import sys

def makeRepoQuery(repo_name):
	repo_query = 'https://api.github.com/repos/' + repo_name
	return repo_query

def makeIssuesQuery(repo_name, page_num):
	issue_query = 'https://api.github.com/repos/' + repo_name
	issue_query += '/issues?state=all&per_page=100&page=' + str(page_num)
	return issue_query

def makeUserQuery(username):
	email_query = 'https://api.github.com/users/'+username
	return email_query

def makeEventsQuery(username):
	email_query = 'https://api.github.com/users/'+username+'/events/public'
	return email_query

def executeQuery(query, git_user_file = "gituser.txt"):
	lines = [line.strip() for line in open(git_user_file)]
	req = requests.get(query,
			auth = HTTPBasicAuth(lines[0], lines[1]), verify = False)
	result = json.loads(req.text)
	return result

def didQueryFail(a_dict, keys):
	# If the query failed
	if len(a_dict) == 2:
		if 'message' in a_dict:
			if a_dict['message'] == 'Not Found' or \
		 	   a_dict['message'] == 'Issues are disabled for this repo':
				return True
	return False

def makeNoneList(keys):
	return [None for i in range(len(keys))]

def getRepoVals(repo_dict, keys):
	if didQueryFail(repo_dict, keys):
		return makeNoneList(keys)

	l = []
	for key in keys:
		if type(key) == type(['list']):
			# a nested value
			val = repo_dict[key[0]][key[1]]
		else:
			val = repo_dict[key]
		
		# convert unicode to ascii equivalent
		if type(val) == type(u'unicode'): val = unidecode(val)
		# cut off time in timedate strings
		if type(val) == type('str') and val.count(':') == 2: val = val[:val.find('T')]

		l.append(val)

	return l

def getIssuesVals(issues_list, keys):
	if didQueryFail(issues_list, keys):
		return makeNoneList(keys)

	# Return a list of lists (rather than a list of tuples) because we will need to add a
	# linking forieng key from Repos table to each issue
	lol = []
	for issue_dict in issues_list:
		l = []
		for key in keys:
			if type(key) == type(['list']):
				nstd = issue_dict[key[0]]
				if type(nstd) == type(['list']):
					if len(nstd) == 0: val = None
					else:
						val = ''
						for nstd_d in nstd:
							val += nstd_d[key[1]].lower() + ',' # comma sep issue labels
						val = val[:-1]
				elif type(nstd) == type({'a':'dict'}):
					val = nstd[key[1]]
				elif nstd == None:
					val = None
				else:
					print "Unknown format for ", key
					sys.exit()
			else:
				val = issue_dict[key]

			# Unidecode conversion and date from datetime extraction
			if type(val) == type(u'unicode'): val = unidecode(val)
			if type(val) == type('str') and val.count(':') == 2: val = val[:val.find('T')]

			l.append(val)
		lol.append(l)
	return lol

def getEmailFromUser(username):
	user_query = makeUserQuery(username)
	user_dict = executeQuery(user_query)
	if 'email' in user_dict:
		email = user_dict['email']
		if email != None:
			if len(email) != 0 and email.isspace() == False:
				return email

def getEmailFromEvents(username):
	user_query = makeUserQuery(username)
	user_dict = executeQuery(user_query)
	if 'name' in user_dict: 
		name = user_dict['name']
	else: name = None

	events_query = makeEventsQuery(username)
	events_list = executeQuery(events_query)
	aliases = set() # Find all aliases (names + emails)
	for event_dict in events_list:
		try:
			for com in event_dict['payload']['commits']:
	 			alias_email = com['author']['email']
	 			alias_name = com['author']['name']
	 			
	 			if alias_name == username or alias_name == name:
	 				if alias_email != '' and email.isspace() == False:
	 					return alias_email
	 			
	 			aliases.add(alias_email)
	 			aliases.add(alias_name)
	 	except:
	 		pass
	# Find the closest matches (email or name) to this username
	match = get_close_matches(username, aliases, cutoff=0.3)
	
	if len(match) > 0:
		best_match = match[0]
		if '@' in best_match:
			return best_match
		else: # Loop back hrough and find the email for this name
			for event_dict in events_list:
				try:
					for com in event_dict['payload']['commits']:
			 			if com['author']['name'] == best_match:
			 				e = com['author']['email']
			 				if e != '' and email.isspace() == False:
			 					return com['author']['email']
			 	except:
			 		pass

def getRepoInfo(repo_name, keys):
	# print '  ', repo_name
	repo_query = makeRepoQuery(repo_name)
	repo_dict = executeQuery(repo_query)
	repo_info = getRepoVals(repo_dict, keys)
	return repo_info

def getIssuesInfo(repo_name, keys):
	# print '  ', repo_name
	repo_issues_info = []
	this_issues_info = []
	page_num = 1
	last_page = False # Stopping condition 1: if we run out of issues
	none_list = makeNoneList(keys) # Stopping condition 2: if the repo_name is invalid
	while last_page == False and this_issues_info != none_list:
		issue_query = makeIssuesQuery(repo_name, page_num)
		issues_list = executeQuery(issue_query)
		if len(issues_list) == 0:
			last_page = True
			# Just break before going through value retrieval if there are no issues
			break
		this_issues_info = getIssuesVals(issues_list, keys)
		repo_issues_info.extend(this_issues_info)
		page_num += 1

	return repo_issues_info

def getUserEmail(username):
	user_email = getEmailFromUser(username)
	if user_email != None: 
		# print '   ', username, ':', user_email, '  (Info)'
		return str(user_email)
	else: # If they're email isn't public, try to find it in public events
		user_email = getEmailFromEvents(username)
		# print '   ', username, ':', user_email, '  (Events)'
		return str(user_email)

def getRemaininder():
	result = executeQuery('https://api.github.com/rate_limit')
	print result['rate']['remaining']

