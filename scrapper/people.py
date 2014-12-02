#people.py

#!/usr/bin/env python

# A person, with a name and an email
# Along with any other names and any other emails
class Person:
    def __init__(self, name):
        # Set the canonical name for this person.  Also add it as an
        # alias, so we don't have to special-case it in the future.
        self.name = name
        self.email = None

        self.all_names = set([name])
        self.all_emails = set()

    def __str__(self):
        return self.name + ' (' + str(self.all_names) + ') ' + str(self.email) + ' (' + str(self.all_emails) + ') '

    # Define equality as having either at least one name or at least
    # one email address in common.  Or, if we're comparing to a
    # string, define equality as that string being either an name or
    # an email address.
    def __eq__(self, other):
        try:
            if self.all_names.isdisjoint(other.all_names) and \
               self.all_emails.isdisjoint(other.all_emails):
                return False
            else:
                return True
        except:
            if other in self.all_names or other in self.all_emails:
                return True
            else:
                return False

    # Bogus hash function to make sets work.  A unity hash like this
    # is super-inefficient, but we're only dealing with small sets of
    # names, so we can live with it.
    def __hash__(self):
        return 1
        
    def addName(self, name):
        self.all_names.add(name)

    def addNames(self, names):
        for name in names:
            self.all_names(name)

    def addEmail(self, email):
        if email == None: return # Don't add None as an email
        if self.email == None:
            self.email = email
        self.all_emails.add(email)

    def addEmails(self, emails):
        for email in emails:
            self.addEmail(email)

    def merge(self, other):
        self.all_names |= other.all_names
        self.all_emails |= other.all_emails

# Sets of people.  This is a thin wrapper around a set that gives us
# some extra functionality.
class PersonSet:
    def __init__(self):
        self.people = set()

    def __str__(self):
        s = ''
        for person in self.people:
            s += str(person) + '\n'
        return s

    def __len__(self):
        return len(self.people)
    
    def add(self, person):
        for p in self.people:
            if p == person:
                p.merge(person)
                return
        self.people.add(person)

    def find(self, person):
        for p in self.people:
            if p == person:
                return p
        return None
    

if __name__ == '__main__':
    people = PersonSet()

    p1 = Person('one')
    p1.addName('wun')
    p1.addEmails(['one@foo.com', '1@2.3'])

    p2 = Person('two')
    p2.addEmail('one@foo.com')
    
    people.add(p1)
    people.add(p2)

    print people
    
    print people.find(p2)
    print people.find('1@2.3')
