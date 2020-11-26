# -*- coding: utf-8 -*-

#Copyright (c) 2020 André Santos
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

import logging
import unittest

from hypothesis import given, settings
import hypothesis.strategies as st

from .hpl_parser import hpl_predicate_parser, hpl_property_parser
from .hpl_ast import (
    HplAssumption, HplProperty, HplScope, HplPattern, HplEvent,
    HplPredicate, HplVacuousTruth, HplQuantifier, HplUnaryOperator,
    HplBinaryOperator, HplSet, HplRange, HplLiteral, HplVarReference,
    HplFunctionCall, HplFieldAccess, HplArrayAccess, HplThisMessage
)


###############################################################################
# Strategies
###############################################################################

ALPHA = "abcdefghijklmnopqrstuvwxyz"
NUM_VARS = ("@i", "@j", "@k")
MSG_VARS = ("@P", "@A")
NUM_FIELDS = ("a", "b")
BOOL_FIELDS = ("p", "q")
MSG_FIELDS = ("msg1", "msg2")
ARRAY_FIELDS = ("xs", "ys")
TOPICS = ("topic_a", "topic_b")

@st.composite
def hpl_booleans(draw):
    b = draw(st.booleans())
    return HplLiteral(repr(b), b)

@st.composite
def hpl_integers(draw):
    n = draw(st.integers(min_value=0))
    return HplLiteral(repr(n), n)

@st.composite
def hpl_floats(draw):
    n = draw(st.floats(min_value=0.0))
    return HplLiteral(repr(n), n)

def hpl_numbers():
    return st.one_of(hpl_integers(), hpl_floats())

@st.composite
def hpl_strings(draw):
    s = draw(st.text(alphabet=ALPHA))
    return HplLiteral(s, s)

def hpl_this():
    return st.builds(HplThisMessage)

def hpl_vars():
    return st.builds(HplVarReference, st.sampled_from(NUM_VARS))

def _num_vars():
    return st.builds(HplVarReference, st.sampled_from(NUM_VARS))

def _msg_vars():
    return st.builds(HplVarReference, st.sampled_from(MSG_VARS))

def _num_fields():
    msg = hpl_this()
    fnum = st.sampled_from(NUM_FIELDS)
    s1 = _field(msg, fnum)
    fmsg = st.sampled_from(MSG_FIELDS)
    s2 = _field(_field(msg, fmsg), fnum)
    farr = st.sampled_from(ARRAY_FIELDS)
    s3 = _array(_field(msg, farr))
    return st.one_of(s1, s2, s3)

def _bool_fields():
    msg = hpl_this()
    fbool = st.sampled_from(BOOL_FIELDS)
    s1 = _field(msg, fbool)
    fmsg = st.sampled_from(MSG_FIELDS)
    s2 = _field(_field(msg, fmsg), fbool)
    return st.one_of(s1, s2)

def _base_nums():
    return st.one_of(hpl_numbers(), _num_vars(), _num_fields())

def hpl_ranges():
    lb = ub = _base_nums()
    el = eu = st.booleans()
    return st.builds(HplRange, lb, ub, exc_min=el, exc_max=eu)

def hpl_sets():
    return st.builds(HplSet, st.lists(_base_nums(), min_size=1))

def hpl_values():
    return st.one_of(hpl_booleans(), hpl_numbers(), hpl_strings(),
        hpl_this(), hpl_vars(), hpl_ranges(), hpl_sets())

def _field(msg, field):
    return st.builds(HplFieldAccess, msg, field)

def _array(msg):
    return st.builds(HplArrayAccess, msg, _indices())

def _indices():
    base = hpl_integers() | _num_vars()
    return st.recursive(base,
        lambda x: _num_unops(x) | _num_binops(x),
        max_leaves=2)

def hpl_fields(ext=True):
    field = st.sampled_from(FIELDS)
    base = hpl_this()
    if ext:
        base = base | hpl_vars()
    base = _field(base, field)
    return st.recursive(base,
        lambda msg: _field(msg, field) | _array(_field(msg, field)),
        max_leaves=3)

def hpl_functions():
    pass # TODO

def _num_unops(x):
    return st.builds(HplUnaryOperator, st.just("-"), x)

def _num_binops(x):
    ops = ("+", "-", "*", "/", "**")
    return st.builds(HplBinaryOperator, st.sampled_from(ops), x, x)

def hpl_num_exprs():
    return st.deferred(lambda: st.recursive(_base_nums(),
        lambda x: _num_unops(x) | _num_binops(x),
        max_leaves=2))

def _cmp_exprs():
    ops = ("=", "!=", "<", "<=", ">", ">=")
    return st.builds(HplBinaryOperator,
        st.sampled_from(ops), _num_fields(), hpl_num_exprs())

def _inc_exprs():
    sf = _num_fields()
    ranges = st.builds(HplBinaryOperator,
        st.just("in"), sf, hpl_ranges())
    sets = st.builds(HplBinaryOperator,
        st.just("in"), sf, hpl_sets())
    return st.one_of(ranges, sets)

def _atomic_bool_exprs():
    return st.one_of(_bool_fields(), _cmp_exprs(), _inc_exprs())

def _bool_unops(p):
    return st.builds(HplUnaryOperator, st.just("not"), p)

def _bool_binops(p):
    ops = ("and", "or", "implies", "iff")
    return st.builds(HplBinaryOperator, st.sampled_from(ops), p, p)

def hpl_bool_exprs():
    return st.recursive(_atomic_bool_exprs(),
        lambda p: _bool_unops(p) | _bool_binops(p),
        max_leaves=2)

def hpl_quantifiers():
    pass # TODO

def hpl_predicates():
    return st.builds(HplPredicate, hpl_bool_exprs())

def hpl_events():
    topic = st.sampled_from(TOPICS)
    phi = hpl_predicates()
    return st.builds(HplEvent.publish, topic, predicate=phi)

def hpl_patterns():
    t = st.just(float("inf")) | st.floats(min_value=0.0, max_value=1.0)
    return st.builds(HplPattern.existence, hpl_events(), max_time=t)

def hpl_scopes():
    return st.builds(HplScope.globally)

def hpl_properties():
    return st.builds(HplProperty, hpl_scopes(), hpl_patterns())


###############################################################################
# Test Cases
###############################################################################

class TestEncoding(unittest.TestCase):
    def setUp(self):
        self.prop_parser = hpl_property_parser(debug=True)

    @settings(max_examples=1000)
    @given(hpl_properties())
    def test_parse_str_trees(self, prop):
        s = str(prop)
        ast = self.prop_parser.parse(s)
        self.assertEqual(ast, prop)


###############################################################################
# Entry Point
###############################################################################

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    unittest.main()
