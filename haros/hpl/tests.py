# -*- coding: utf-8 -*-

#Copyright (c) 2018 André Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.


###############################################################################
# Imports
###############################################################################

from lark import Lark, Transformer
from lark.exceptions import (
    UnexpectedCharacters, UnexpectedToken, GrammarError, VisitError
)
import logging
from sys import exit

from .hpl_parser import (
    PROPERTY_GRAMMAR, ASSUMPTION_GRAMMAR, PropertyTransformer,
    hpl_property_parser, hpl_assumption_parser
)


###############################################################################
# Helper Functions
###############################################################################


###############################################################################
# Predicate Examples
###############################################################################

BAD_PREDICATES = [
    "a + b + c",
    "{1} and [2 to f.g.h]",
    "(1 and 2) + (not x)",
    "forall 42 in D: phi",
    "forall x in 42: phi",
    "forall x in D: 42",
    "a implies iff b",
    "a implies 42",
    "a and 42",
    "a or 42",
    "not 42",
    "not a + b",
    "-(a and b)",
    "a < b < c",
    "(a < b) < c",
    "x = -{1,2,3}",
    "a implies forall x in xs: b",
    "a[1][2]",
]

GOOD_PREDICATES = [
    "@a < 3",
    "a + b < c",
    "forall x in xs: @x",
    "a implies iff iff iff",
    "not a + b < c",
    "---42 = -42",
    "a + b * c ** d = e ** -(f - g) / h",
    "(not ((a or b) implies c) and d)",
    "a[1] = a[@i + 1]",
]


###############################################################################
# Property Examples
###############################################################################

BAD_PROPERTIES = [
    # missing scope
    "some topic",

    # using comma instead of 'and' to separate filters
    'globally: some topic {int < 1, float < 2, string = "hello"}',

    # filters must be non-empty
    "globally: some topic {}",

    # do not allow spaces in index operator
    "globally: some topic {int_array [1] > 0}",

    # cannot compare numbers to strings
    'globally: some topic {int > "42"}',

    # cannot duplicate aliases
    "globally: input as M causes output1 as M"
]


GOOD_PROPERTIES = [
    'globally: some topic {int < 1 and float < 2 and string = "hello"}',

    "globally: no topic within 1s",

    "globally: input causes output",

    "globally: input causes output within 1s",

    "globally: output requires input",

    "globally: output requires input within 100 ms",

    """after ~events/bumper {state = PRESSED}:
        some ~cmd_vel {linear.x < 0.0 and angular.z = 0.0}""",

    "after input: no output",

    "globally: some topic {m.int in 0 to 10!}",

    "globally: some topic {not int in 0 to 10}",

    "globally: some topic {float_array[0] < float_array[1]}",

    "globally: some topic {forall i in [0 to len(int_array)!]: int_array[@i] > 0}",

    r"globally: some topic {exists x in int_array: @x > 0}",

    "globally: some topic {len(twist_array) > 0}",

    "until input: some output",

    "after input as M: some output {x = @M.x}",
]


###############################################################################
# Test Code
###############################################################################

def test_routine(parser, good, bad):
    transformer = PropertyTransformer()
    for test_str in bad:
        print "\n  #", repr(test_str)
        try:
            tree = parser.parse(test_str)
            print "\n[ Parsing ] OK"
            tree = transformer.transform(tree)
            print "[Transform] OK (unexpected)"
            print ""
            print repr(tree)
            return 1
        except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as e:
            print "[ Parsing ] FAIL (expected)"
            print "  >>", str(e)
        except VisitError as e:
            # e.orig_exc
            print "[Transform] FAIL (expected)"
            print "  >>", str(e)
    for test_str in good:
        print "\n  #", repr(test_str)
        try:
            tree = parser.parse(test_str)
            print "\n[ Parsing ] OK"
            tree = transformer.transform(tree)
            print "[Transform] OK"
            print ""
            print repr(tree)
        except (UnexpectedToken, UnexpectedCharacters, SyntaxError) as e:
            print "[ Parsing ] FAIL"
            print "  >>", str(e)
            return 1
        except VisitError as e:
            # e.orig_exc
            print "[Transform] FAIL"
            print " >>", str(e)
            print ""
            print repr(tree)
            return 1
    print "\nAll", str(len(bad) + len(good)), "tests passed."
    return 0


def test_predicates():
    parser = Lark(PROPERTY_GRAMMAR, parser="lalr",
                  start="top_level_condition", debug=True)
    return test_routine(parser, GOOD_PREDICATES, BAD_PREDICATES)
    #return test_routine(parser, (), BAD_PREDICATES)

def test_properties():
    parser = Lark(PROPERTY_GRAMMAR, parser="lalr",
                  start="hpl_property", debug=True)
    return test_routine(parser, GOOD_PROPERTIES, BAD_PROPERTIES)


def main():
    logging.basicConfig(level=logging.DEBUG)
    try:
        if test_predicates():
            assert False
        # test_properties()
    except GrammarError as e:
        logging.error(str(e))
        return 1
    return 0


if __name__ == "__main__":
    exit(main())
