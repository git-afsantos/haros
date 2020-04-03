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
from lark.exceptions import UnexpectedCharacters, UnexpectedToken, GrammarError
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

PREDICATES = [
    "a + b + c",
    "@a < 3",
    "{1} and [2 to f.g.h]",
    "(1 and 2) + (not x)"
]


###############################################################################
# Property Examples
###############################################################################

FAILING_TESTS = [
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


PASSING_TESTS = [
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

def test_predicates():
    parser = Lark(PROPERTY_GRAMMAR, parser="lalr",
                  start="condition", debug=True)
    transformer = PropertyTransformer()
    for phi in PREDICATES:
        print "\n>>", phi
        tree = parser.parse(phi)
        tree = transformer.transform(tree)
        print ""
        print repr(tree)

def test_properties():
    parser = Lark(PROPERTY_GRAMMAR, parser="lalr",
                  start="hpl_property", debug=True)


def main():
    logging.basicConfig(level=logging.DEBUG)
    try:
        test_predicates()
        # test_properties()
    except GrammarError as e:
        logging.error(str(e))
        return 1

    """transformer = PropertyTransformer()

    for test_str in FAILING_TESTS:
        try:
            tree = parser.parse(test_str)
            tree = transformer.transform(tree)
            print ""
            print test_str
            assert False, "expected failure"
        except (UnexpectedToken, UnexpectedCharacters, TypeError, SyntaxError,
                HplSanityError):
            pass

    for test_str in PASSING_TESTS:
        print ""
        print test_str
        tree = parser.parse(test_str)
        tree = transformer.transform(tree)
        print tree

    print "All", str(len(FAILING_TESTS) + len(PASSING_TESTS)), "tests passed."
"""
    return 0


if __name__ == "__main__":
    exit(main())
