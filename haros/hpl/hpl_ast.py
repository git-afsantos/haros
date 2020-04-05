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

from collections import deque, namedtuple

from .ros_types import (
    ROS_NUMBER_TYPES, ROS_PRIMITIVE_TYPES, possible_types
)


###############################################################################
# Constants
###############################################################################

INF = float("inf")


###############################################################################
# Exceptions
###############################################################################

class HplSanityError(Exception):
    pass

class HplLogicError(Exception):
    pass

class HplTypeError(Exception):
    def __init__(self, msg, orig):
        Exception.__init__(self, msg)
        self.orig_exc = orig

    def __str__(self):
        return "{}: {}".format(
            Exception.__str__(self), self.orig_exc.message)


###############################################################################
# Top-level Classes
###############################################################################

class HplAstObject(object):
    @property
    def is_property(self):
        return False

    @property
    def is_assumption(self):
        return False

    @property
    def is_scope(self):
        return False

    @property
    def is_pattern(self):
        return False

    @property
    def is_event(self):
        return False

    @property
    def is_expression(self):
        return False

    def children(self):
        return ()

    def iterate(self):
        yield self
        queue = deque(self.children())
        while queue:
            obj = queue.popleft()
            yield obj
            queue.extend(obj.children())


class HplAssumption(HplAstObject):
    __slots__ = ("topic", "predicate")

    def __init__(self, topic, phi):
        self.topic = topic # string
        self.predicate = phi # HplExpression

    @property
    def is_assumption(self):
        return True

    def children(self):
        return (self.predicate)

    def __eq__(self, other):
        if not isinstance(other, HplAssumption):
            return False
        return (self.topic == other.topic
                and self.predicate == other.predicate)

    def __hash__(self):
        return 31 * hash(self.topic) + hash(self.predicate)

    def __str__(self):
        return "{} {}".format(self.topic, self.predicate)

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__,
            repr(self.topic), repr(self.predicate))


class HplProperty(HplAstObject):
    __slots__ = ("scope", "pattern")

    def __init__(self, scope, pattern):
        self.scope = scope # HplScope
        self.pattern = pattern # HplPattern

    @property
    def is_property(self):
        return True

    @property
    def is_safety(self):
        return self.pattern.is_safety

    @property
    def is_liveness(self):
        return self.pattern.is_liveness

    def children(self):
        return (self.scope, self.pattern)

    def events(self):
        if self.scope.activator is not None:
            yield self.scope.activator
        yield self.pattern.behaviour
        if self.pattern.trigger is not None:
            yield self.pattern.trigger
        if self.scope.terminator is not None:
            yield self.scope.terminator

    def sanity_check(self):
        initial = self._check_activator()
        aliases = self._check_trigger(initial)
        self._check_behaviour(aliases)
        self._check_terminator(initial)

    def _check_activator(self):
        p = self.scope.activator
        if p is not None:
            refs = p.external_references()
            if refs:
                raise HplSanityError(
                    "references to undefined events: " + repr(refs))
            return p.aliases()
        return ()

    def _check_trigger(self, available):
        s = self.pattern
        if s.is_absence or s.is_existence:
            return available
        a = s.trigger
        assert a is not None
        ext = a.external_references()
        if s.is_response or s.is_prevention:
            self._check_refs_defined(ext, available)
        elif s.is_requirement:
            aliases = available + s.behaviour.aliases()
            self._check_refs_defined(ext, aliases)
        else:
            assert False, "unexpected pattern type: " + repr(s.pattern_type)
        aliases = a.aliases()
        self._check_duplicates(aliases, available)
        return aliases + available

    def _check_behaviour(self, available):
        b = self.pattern.behaviour
        self._check_refs_defined(b.external_references(), available)
        self._check_duplicates(b.aliases(), available)

    def _check_terminator(self, available):
        q = self.scope.terminator
        if q is not None:
            self._check_refs_defined(q.external_references(), available)
            self._check_duplicates(q.aliases(), available)

    def _check_refs_defined(self, refs, available):
        for ref in refs:
            if not ref in available:
                raise HplSanityError(
                    "reference to undefined event: " + repr(ref))

    def _check_duplicates(self, aliases, available):
        for alias in aliases:
            if alias in available:
                raise HplSanityError("duplicate alias: " + repr(alias))

    def __eq__(self, other):
        if not isinstance(other, HplProperty):
            return False
        return self.pattern == other.pattern and self.scope == other.scope

    def __hash__(self):
        return 31 * hash(self.scope) + hash(self.pattern)

    def __str__(self):
        return "{}: {}".format(self.scope, self.pattern)

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__,
            repr(self.scope), repr(self.pattern))


class HplScope(HplAstObject):
    __slots__ = ("scope_type", "activator", "terminator")

    GLOBAL = 1
    AFTER_UNTIL = 2
    AFTER = 3
    UNTIL = 4

    def __init__(self, scope, activator=None, terminator=None):
        if scope == self.GLOBAL:
            if activator is not None:
                raise ValueError(activator)
            if terminator is not None:
                raise ValueError(terminator)
        elif scope == self.AFTER:
            if terminator is not None:
                raise ValueError(terminator)
        elif scope == self.UNTIL:
            if activator is not None:
                raise ValueError(activator)
        elif scope != self.AFTER_UNTIL:
            raise ValueError(scope)
        self.scope_type = scope
        self.activator = activator # HplEvent | None
        self.terminator = terminator # HplEvent | None

    @property
    def is_scope(self):
        return True

    @classmethod
    def globally(cls):
        return cls(cls.GLOBAL)

    @classmethod
    def after(cls, activator):
        return cls(cls.AFTER, activator=activator)

    @classmethod
    def until(cls, terminator):
        return cls(cls.UNTIL, terminator=terminator)

    @classmethod
    def after_until(cls, activator, terminator):
        return cls(cls.AFTER_UNTIL, activator=activator, terminator=terminator)

    @property
    def is_global(self):
        return self.scope_type == self.GLOBAL

    @property
    def is_after(self):
        return self.scope_type == self.AFTER

    @property
    def is_until(self):
        return self.scope_type == self.UNTIL

    @property
    def is_after_until(self):
        return self.scope_type == self.AFTER_UNTIL

    def children(self):
        if self.activator is None and self.terminator is None:
            return ()
        if self.activator is None:
            return (self.terminator,)
        if self.terminator is None:
            return (self.activator,)
        return (self.activator, self.terminator)

    def __eq__(self, other):
        if not isinstance(other, HplScope):
            return False
        return (self.scope_type == other.scope_type
                and self.activator == other.activator
                and self.terminator == other.terminator)

    def __hash__(self):
        h = 31 * hash(self.scope_type) + hash(self.activator)
        h = 31 * h + hash(self.terminator)
        return h

    def __str__(self):
        if self.scope_type == self.GLOBAL:
            return "globally"
        if self.scope_type == self.AFTER:
            return "after {}".format(self.activator)
        if self.scope_type == self.UNTIL:
            return "until {}".format(self.terminator)
        if self.scope_type == self.AFTER_UNTIL:
            return "after {} until {}".format(self.activator, self.terminator)
        assert False, "unexpected scope type"

    def __repr__(self):
        return "{}({}, activator={}, terminator={})".format(
            type(self).__name__, repr(self.scope_type),
            repr(self.activator), repr(self.terminator))


class HplPattern(HplAstObject):
    __slots__ = ("pattern_type", "behaviour", "trigger", "min_time", "max_time")

    EXISTENCE = 1
    ABSENCE = 2
    RESPONSE = 3
    REQUIREMENT = 4
    PREVENTION = 5

    def __init__(self, pattern, behaviour, trigger, min_time=0.0, max_time=INF):
        if pattern == self.EXISTENCE or pattern == self.ABSENCE:
            if trigger is not None:
                raise ValueError(trigger)
        elif (pattern != self.RESPONSE and pattern != self.REQUIREMENT
                and pattern != self.PREVENTION):
            raise ValueError(pattern)
        self.pattern_type = pattern
        self.behaviour = behaviour # HplEvent
        self.trigger = trigger # HplEvent | None
        self.min_time = min_time
        self.max_time = max_time

    @property
    def is_pattern(self):
        return True

    @classmethod
    def existence(cls, behaviour, min_time=0.0, max_time=INF):
        return cls(cls.EXISTENCE, behaviour, None,
            min_time=min_time, max_time=max_time)

    @classmethod
    def absence(cls, behaviour, min_time=0.0, max_time=INF):
        return cls(cls.ABSENCE, behaviour, None,
            min_time=min_time, max_time=max_time)

    @classmethod
    def response(cls, event, response, min_time=0.0, max_time=INF):
        return cls(cls.RESPONSE, response, event,
            min_time=min_time, max_time=max_time)

    @classmethod
    def requirement(cls, event, requirement, min_time=0.0, max_time=INF):
        return cls(cls.REQUIREMENT, event, requirement,
            min_time=min_time, max_time=max_time)

    @classmethod
    def prevention(cls, event, forbidden, min_time=0.0, max_time=INF):
        return cls(cls.PREVENTION, event, forbidden,
            min_time=min_time, max_time=max_time)

    @property
    def is_safety(self):
        return (self.pattern_type == self.ABSENCE
                or self.pattern_type == self.REQUIREMENT
                or self.pattern_type == self.PREVENTION)

    @property
    def is_liveness(self):
        return (self.pattern_type == self.EXISTENCE
                or self.pattern_type == self.RESPONSE)

    @property
    def is_absence(self):
        return self.pattern_type == self.ABSENCE

    @property
    def is_existence(self):
        return self.pattern_type == self.EXISTENCE

    @property
    def is_requirement(self):
        return self.pattern_type == self.REQUIREMENT

    @property
    def is_response(self):
        return self.pattern_type == self.RESPONSE

    @property
    def is_prevention(self):
        return self.pattern_type == self.PREVENTION

    def children(self):
        if self.trigger is None:
            return (self.behaviour,)
        return (self.trigger, self.behaviour)

    def __eq__(self, other):
        if not isinstance(other, HplPattern):
            return False
        return (self.pattern_type == other.pattern_type
                and self.behaviour == other.behaviour
                and self.trigger == other.trigger
                and self.min_time == other.min_time
                and self.max_time == other.max_time)

    def __hash__(self):
        h = 31 * hash(self.pattern_type) + hash(self.behaviour)
        h = 31 * h + hash(self.trigger)
        h = 31 * h + hash(self.min_time)
        h = 31 * h + hash(self.max_time)
        return h

    def __str__(self):
        t = ""
        if self.max_time < INF:
            t = " within {}s".format(self.max_time)
        if self.pattern_type == self.EXISTENCE:
            return "some {}{}".format(self.behaviour, t)
        if self.pattern_type == self.ABSENCE:
            return "no {}{}".format(self.behaviour, t)
        if self.pattern_type == self.RESPONSE:
            return "{} causes {}{}".format(self.trigger, self.behaviour, t)
        if self.pattern_type == self.REQUIREMENT:
            return "{} requires {}{}".format(self.behaviour, self.trigger, t)
        if self.pattern_type == self.PREVENTION:
            return "{} forbids {}{}".format(self.trigger, self.behaviour, t)
        assert False, "unexpected observable pattern"

    def __repr__(self):
        return "{}({}, {}, {}, min_time={}, max_time={})".format(
            type(self).__name__, repr(self.pattern_type), repr(self.behaviour),
            repr(self.trigger), repr(self.min_time), repr(self.max_time))


###############################################################################
# Events and Event Operators
###############################################################################

class HplEvent(HplAstObject):
    __slots__ = ("event_type", "predicate", "topic", "alias", "msg_type")

    PUBLISH = 1

    def __init__(self, event_type, predicate, topic, alias=None):
        if event_type != self.PUBLISH:
            raise ValueError(event_type)
        if not isinstance(predicate, HplExpression):
            raise TypeError("predicate is not an expression: " + str(predicate))
        if not predicate.is_bool:
            raise TypeError("predicate must be a boolean expression: "
                            + str(predicate))
        self.event_type = event_type
        self.predicate = predicate # HplExpression
        self.topic = topic # string
        self.alias = alias # string
        self.msg_type = None # .ros_types.TypeToken

    @property
    def is_event(self):
        return True

    @classmethod
    def publish(cls, topic, predicate=None, alias=None):
        if predicate is None:
            predicate = HplVacuousTruth()
        return cls(cls.PUBLISH, predicate, topic, alias=alias)

    @property
    def is_publish(self):
        return self.event_type == self.PUBLISH

    @property
    def phi(self):
        return self.predicate

    def children(self):
        return (self.predicate,)

    def aliases(self):
        if self.alias is None:
            return ()
        return (self.alias,)

    def external_references(self):
        refs = set()
        for obj in self.iterate():
            if isinstance(obj, HplFieldReference):
                if obj.message is not None:
                    refs.add(obj.message)
        if self.alias is not None:
            refs.discard(self.alias)
        return refs

    def __eq__(self, other):
        if not isinstance(other, HplEvent):
            return False
        return (self.event_type == other.event_type
                and self.predicate == other.predicate
                and self.topic == other.topic)

    def __hash__(self):
        h = 31 * hash(self.event_type) + hash(self.predicate)
        return 31 * h + hash(self.topic)

    def __str__(self):
        phi = ""
        if not self.predicate.is_trivial:
            phi = " {{ {} }}".format(self.predicate)
        alias = (" as " + self.alias) if self.alias is not None else ""
        if self.event_type == self.PUBLISH:
            return "{}{}{}".format(self.topic, alias, phi)
        else:
            assert False, "unexpected event type"

    def __repr__(self):
        return "{}({}, {}, {}, alias={})".format(
            type(self).__name__, repr(self.event_type), repr(self.predicate),
            repr(self.topic), repr(self.alias))


###############################################################################
# Type System
###############################################################################

# Bit Flags

T_BOOL = 0x1
T_NUM = 0x2
T_STR = 0x4
T_ARR = 0x8

T_RAN = 0x10
T_SET = 0x20

T_ANY = T_BOOL | T_NUM | T_STR | T_ARR | T_RAN | T_SET
T_COMP = T_ARR | T_RAN | T_SET
T_PRIM = T_BOOL | T_NUM | T_STR
T_ROS = T_BOOL | T_NUM | T_STR | T_ARR

_TYPE_NAMES = {
    T_BOOL: "boolean",
    T_NUM: "number",
    T_STR: "string",
    T_ARR: "array",
    T_RAN: "range",
    T_SET: "set",
}

def type_name(t):
    if t in _TYPE_NAMES:
        return _TYPE_NAMES[t]
    ns = []
    for key, name in _TYPE_NAMES.items():
        if (t & key) != 0:
            ns.append(name)
    return " or ".join(ns)


###############################################################################
# Expressions
###############################################################################

class HplExpression(HplAstObject):
    __slots__ = ("types",)

    def __init__(self, types=T_ANY):
        self.types = types

    @property
    def is_expression(self):
        return True

    @property
    def is_value(self):
        return False

    @property
    def is_operator(self):
        return False

    @property
    def is_quantifier(self):
        return False

    @property
    def is_implicit(self):
        return False

    @property
    def is_vacuous(self):
        return False

    @property
    def can_be_bool(self):
        return bool(self.types & T_BOOL)

    @property
    def can_be_number(self):
        return bool(self.types & T_NUM)

    @property
    def can_be_string(self):
        return bool(self.types & T_STR)

    @property
    def can_be_array(self):
        return bool(self.types & T_ARR)

    @property
    def can_be_set(self):
        return bool(self.types & T_SET)

    @property
    def can_be_range(self):
        return bool(self.types & T_RAN)

    def can_be(self, t):
        return bool(self.types & t)

    def cast(self, t):
        r = self.types & t
        if not r:
            raise TypeError("expected ({}) but found ({}): {}".format(
                type_name(t), type_name(self.types), self))
        self.types = r

    def _type_check(self, x, t):
        try:
            x.cast(t)
        except TypeError as e:
            msg = "Type error in expression '{}'".format(self)
            raise HplTypeError(msg, e)

    def add_type(self, t):
        self.types = self.types | t

    def rem_type(self, t):
        self.types = self.types & ~t
        if not self.types:
            raise TypeError("no types left: " + str(self))


class HplValue(HplExpression):
    __slots__ = HplExpression.__slots__

    @property
    def is_value(self):
        return True

    @property
    def is_literal(self):
        return False

    @property
    def is_set(self):
        return False

    @property
    def is_range(self):
        return False

    @property
    def is_reference(self):
        return False

    @property
    def is_variable(self):
        return False

    @property
    def is_function_call(self):
        return False


###############################################################################
# Quantifiers and Conditions
###############################################################################

class HplVacuousTruth(HplExpression):
    __slots__ = HplExpression.__slots__

    def __init__(self):
        HplExpression.__init__(self, types=T_BOOL)

    @property
    def is_bool(self):
        return True

    @property
    def is_implicit(self):
        return True

    @property
    def is_vacuous(self):
        return True


class HplQuantifier(HplExpression):
    __slots__ = HplExpression.__slots__ + (
        "quantifier", "variable", "domain", "condition")

    _SET_REF = "cannot reference quantified variable '{}' in the domain of:\n{}"
    _MULTI_DEF = "multiple definitions of variable '{}' in:\n{}"
    _UNUSED = "quantified variable '{}' is never used in:\n{}"

    def __init__(self, qt, var, dom, p):
        HplExpression.__init__(self, types=T_BOOL)
        self.quantifier = qt # string
        self.variable = var # string
        self.domain = dom # HplExpression
        self.condition = p # HplExpression
        self._type_check(dom, T_COMP)
        self._type_check(p, T_BOOL)
        self._check_variables()

    @property
    def is_quantifier(self):
        return True

    @property
    def is_universal(self):
        return self.quantifier == "forall"

    @property
    def op(self):
        return self.quantifier

    @property
    def x(self):
        return self.variable

    @property
    def d(self):
        return self.domain

    @property
    def p(self):
        return self.condition

    @property
    def phi(self):
        return self.condition

    def children(self):
        return (self.domain, self.condition)

    def _check_variables(self):
        types = self._check_domain_vars()
        self._check_expression_vars(types)

    def _check_domain_vars(self):
        dom = self.domain
        for obj in dom.iterate():
            assert obj.is_expression
            if obj.is_value and obj.is_variable:
                assert not obj.is_defined
                v = obj.name
                if self.variable == v:
                    raise HplSanityError(self._SET_REF.format(v, self))
        if dom.is_value:
            if dom.is_set or dom.is_range:
                return dom.subtypes
        return T_PRIM

    def _check_expression_vars(self, t):
        uid = id(self)
        used = 0
        for obj in self.condition.iterate():
            assert obj.is_expression
            if obj.is_value and obj.is_variable:
                v = obj.name
                if self.variable == v:
                    if obj.is_defined:
                        assert obj.defined_at != uid
                        raise HplSanityError(self._MULTI_DEF.format(v, self))
                    obj.defined_at = uid
                    self._type_check(obj, t)
                    used += 1
        if not used:
            raise HplSanityError(self._UNUSED.format(self.variable, self))

    def __eq__(self, other):
        if not isinstance(other, HplQuantifier):
            return False
        return (self.quantifier == other.quantifier
                and self.variable == other.variable
                and self.domain == other.domain
                and self.condition == other.condition)

    def __hash__(self):
        h = 31 * hash(self.quantifier) + hash(self.variable)
        h = 31 * h + hash(self.domain)
        h = 31 * h + hash(self.condition)
        return h

    def __str__(self):
        return "({} {} in {}: {})".format(self.quantifier, self.variable,
            self.domain, self.condition)

    def __repr__(self):
        return "{}({}, {}, {}, {})".format(
            type(self).__name__, repr(self.quantifier), repr(self.variable),
            repr(self.domain), repr(self.condition))


###############################################################################
# Operators
###############################################################################

class HplUnaryOperator(HplExpression):
    __slots__ = HplExpression.__slots__ + ("operator", "operand")

    _OPS = {
        "-": (T_NUM, T_NUM),
        "not": (T_BOOL, T_BOOL)
    }

    def __init__(self, op, arg):
        tin, tout = self._OPS[op]
        HplExpression.__init__(self, types=tout)
        self.operator = op # string
        self.operand = arg # HplExpression
        self._type_check(arg, tin)

    @property
    def is_operator(self):
        return True

    @property
    def arity(self):
        return 1

    @property
    def op(self):
        return self.operator

    @property
    def a(self):
        return self.operand

    def children(self):
        return (self.operand,)

    def __eq__(self, other):
        if not isinstance(other, HplUnaryOperator):
            return False
        return (self.operator == other.operator
                and self.operand == other.operand)

    def __hash__(self):
        return 31 * hash(self.operator) + hash(self.operand)

    def __str__(self):
        op = self.operator
        if op and op[-1].isalpha():
            op = op + " "
        return "({}{})".format(op, self.operand)

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.operator), repr(self.operand))


class HplBinaryOperator(HplExpression):
    __slots__ = HplExpression.__slots__ + (
        "operator", "operand1", "operand2", "infix", "commutative")

    # operator: (Input -> Input -> Output), infix, commutative
    _OPS = {
        "+": (T_NUM, T_NUM, T_NUM, True, True),
        "-": (T_NUM, T_NUM, T_NUM, True, False),
        "*": (T_NUM, T_NUM, T_NUM, True, True),
        "/": (T_NUM, T_NUM, T_NUM, True, False),
        "**": (T_NUM, T_NUM, T_NUM, True, False),
        "implies": (T_BOOL, T_BOOL, T_BOOL, True, False),
        "iff": (T_BOOL, T_BOOL, T_BOOL, True, True),
        "or": (T_BOOL, T_BOOL, T_BOOL, True, True),
        "and": (T_BOOL, T_BOOL, T_BOOL, True, True),
        "=": (T_PRIM, T_PRIM, T_BOOL, True, True),
        "!=": (T_PRIM, T_PRIM, T_BOOL, True, True),
        "<": (T_NUM, T_NUM, T_BOOL, True, False),
        "<=": (T_NUM, T_NUM, T_BOOL, True, False),
        ">": (T_NUM, T_NUM, T_BOOL, True, False),
        ">=": (T_NUM, T_NUM, T_BOOL, True, False),
        "in": (T_PRIM, T_SET | T_RAN, T_BOOL, True, False),
    }

    def __init__(self, op, arg1, arg2):
        tin1, tin2, tout, infix, comm = self._OPS[op]
        HplExpression.__init__(self, types=tout)
        self.operator = op # string
        self.operand1 = arg1 # HplExpression
        self.operand2 = arg2 # HplExpression
        self.infix = infix # bool
        self.commutative = comm # bool
        self._type_check(arg1, tin1)
        self._type_check(arg2, tin2)

    @property
    def is_operator(self):
        return True

    @property
    def arity(self):
        return 2

    @property
    def op(self):
        return self.operator

    @property
    def a(self):
        return self.operand1

    @property
    def b(self):
        return self.operand2

    def children(self):
        return (self.operand1, self.operand2)

    def __eq__(self, other):
        if not isinstance(other, HplBinaryOperator):
            return False
        if self.operator != other.operator:
            return False
        a = self.operand1
        b = self.operand2
        x = other.operand1
        y = other.operand2
        if self.commutative:
            return (a == x and b == y) or (a == y and b == x)
        return a == x and b == y

    def __hash__(self):
        h = 31 * hash(self.operator) + hash(self.operand1)
        h = 31 * h + hash(self.operand2)
        return h

    def __str__(self):
        a = str(self.operand1)
        b = str(self.operand2)
        if self.infix:
            return "({} {} {})".format(a, self.operator, b)
        else:
            return "{}({}, {})".format(self.operator, a, b)

    def __repr__(self):
        return "{}({}, {}, {})".format(
            type(self).__name__, repr(self.operator),
            repr(self.operand1), repr(self.operand2))


###############################################################################
# Compound Values
###############################################################################

class HplSet(HplValue):
    __slots__ = HplValue.__slots__ + ("values", "ros_types")

    def __init__(self, values):
        HplValue.__init__(self, types=T_SET)
        self.values = values # [HplValue]
        for value in values:
            self._type_check(value, T_PRIM)
        self.ros_types = tuple(ROS_PRIMITIVE_TYPES)

    @property
    def is_set(self):
        return True

    @property
    def subtypes(self):
        t = 0
        for value in self.values:
            t = t | value.types
        return t

    def to_set(self):
        return set(self.values)

    def children(self):
        return self.values

    def __eq__(self, other):
        if not isinstance(other, HplSet):
            return False
        return self.values == other.values

    def __hash__(self):
        h = 11
        for value in self.values:
            h = 31 * h + hash(value)
        return h

    def __str__(self):
        return "{{{}}}".format(", ".join(str(v) for v in self.values))

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.values))


class HplRange(HplValue):
    __slots__ = HplValue.__slots__ + (
        "min_value", "max_value", "exclude_min", "exclude_max", "ros_types")

    def __init__(self, lb, ub, exc_min=False, exc_max=False):
        HplValue.__init__(self, types=T_RAN)
        self.min_value = lb # HplValue
        self.max_value = ub # HplValue
        self.exclude_min = exc_min # bool
        self.exclude_max = exc_max # bool
        self._type_check(lb, T_NUM)
        self._type_check(ub, T_NUM)
        #if not any(t in ROS_NUMBER_TYPES for t in lb.ros_types):
        #    raise TypeError("not a number: '{}'".format(lb))
        #if not any(t in ROS_NUMBER_TYPES for t in ub.ros_types):
        #    raise TypeError("not a number: '{}'".format(ub))
        #self.ros_types = tuple(ros_type for ros_type in lb.ros_types
        #                       if ros_type in ub.ros_types)

    @property
    def is_range(self):
        return True

    @property
    def subtypes(self):
        return T_NUM

    def children(self):
        return (self.min_value, self.max_value)

    def __eq__(self, other):
        if not isinstance(other, HplRange):
            return False
        return (self.min_value == other.min_value
                and self.max_value == other.max_value
                and self.exclude_min == other.exclude_min
                and self.exclude_max == other.exclude_max)

    def __hash__(self):
        h = 31 * hash(self.min_value) + hash(self.max_value)
        h = 31 * h + hash(self.exclude_min)
        h = 31 * h + hash(self.exclude_max)
        return h

    def __str__(self):
        lp = "![" if self.exclude_min else "["
        rp = "]!" if self.exclude_max else "]"
        lb = str(self.min_value)
        ub = str(self.max_value)
        return "{}{} to {}{}".format(lp, lb, ub, rp)

    def __repr__(self):
        return "{}({}, {}, exc_min={}, exc_max={})".format(
            type(self).__name__, repr(self.min_value), repr(self.max_value),
            repr(self.exclude_min), repr(self.exclude_max))


###############################################################################
# Atomic Values
###############################################################################

class HplLiteral(HplValue):
    __slots__ = HplValue.__slots__ + ("token", "value", "ros_types")

    def __init__(self, token, value):
        t = T_NUM
        if value is True or value is False:
            t = T_BOOL
        elif isinstance(value, basestring):
            t = T_STR
        elif not isinstance(value, (int, long, float)):
            return TypeError("not a literal: " + repr(value))
        HplValue.__init__(self, types=t)
        self.token = token # string
        self.value = value # int | long | float | bool | string
        self.ros_types = possible_types(value) # [TypeToken]

    @property
    def is_literal(self):
        return True

    def __eq__(self, other):
        if not isinstance(other, HplLiteral):
            return False
        return self.token == other.token

    def __hash__(self):
        return hash(self.token)

    def __str__(self):
        return self.token

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__,
            repr(self.token), repr(self.value))


class HplVarReference(HplValue):
    __slots__ = HplValue.__slots__ + ("token", "defined_at", "ros_types")

    def __init__(self, token):
        HplValue.__init__(self, types=T_PRIM)
        self.token = token # string
        self.defined_at = None
        self.ros_types = tuple(ROS_PRIMITIVE_TYPES)

    @property
    def is_variable(self):
        return True

    @property
    def name(self):
        return self.token[1:] # remove lead "@"

    @property
    def is_defined(self):
        return self.defined_at is not None

    def __eq__(self, other):
        if not isinstance(other, HplVarReference):
            return False
        return self.token == other.token

    def __hash__(self):
        return hash(self.token)

    def __str__(self):
        return self.token

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.token))


class HplFieldReference(HplValue):
    __slots__ = HplValue.__slots__ + ("token", "message", "ros_types", "_parts")

    def __init__(self, token, message=None):
        HplValue.__init__(self, types=T_ROS)
        self.token = token # string
        self.message = message # string (HplEvent.alias)
        self.ros_types = ROS_PRIMITIVE_TYPES # [TypeToken]
        parts = []
        for name in token.split("."):
            i = name.find("[")
            if i >= 0:
                assert name.endswith("]")
                index = name[i+1:-1]
                name = name[:i]
                parts.append(HplFieldAccessor(name, False, False))
                if index.startswith("@"):
                    parts.append(HplFieldAccessor(index[1:], True, True))
                else:
                    parts.append(HplFieldAccessor(index, True, False))
            else:
                parts.append(HplFieldAccessor(name, False, False))
        self._parts = tuple(parts)

    @property
    def parts(self):
        return self._parts

    @property
    def ros_type(self):
        if len(self.ros_types) != 1:
            msg = "undetermined ROS type for field '{}'; possible types: {}"
            raise TypeError(msg.format(self.token, self.ros_types))
        return self.ros_types[0]

    @property
    def is_reference(self):
        return True

    def __eq__(self, other):
        if not isinstance(other, HplFieldReference):
            return False
        return (self.token == other.token
                and self.message == other.message)

    def __hash__(self):
        return 31 * hash(self.token) + hash(self.message)

    def __str__(self):
        return "{}{}".format(
            "" if self.message is None else "@{}.".format(self.message),
            self.token)

    def __repr__(self):
        return "{}({}, message={})".format(
            type(self).__name__, repr(self.token), repr(self.message))


HplFieldAccessor = namedtuple("HplFieldAccessor",
    ("key", "is_indexed", "is_variable"))


class HplFunctionCall(HplValue):
    __slots__ = HplValue.__slots__ + ("function", "arguments", "ros_types")

    # name: Input -> Output
    _BUILTINS = {
        "abs": (T_NUM, T_NUM),
        "bool": (T_PRIM, T_BOOL),
        "int": (T_PRIM, T_NUM),
        "float": (T_PRIM, T_NUM),
        "str": (T_PRIM, T_STR),
        "len": (T_ARR, T_NUM),
        "max": (T_ARR, T_NUM),
        "min": (T_ARR, T_NUM),
        "sum": (T_ARR, T_NUM),
        "prod": (T_ARR, T_NUM),
    }

    def __init__(self, fun, args):
        tin, tout = self._BUILTINS[fun]
        HplValue.__init__(self, types=tout)
        self.function = fun # string
        self.arguments = args # [HplValue]
        for arg in args:
            self._type_check(arg, tin)
        self.ros_types = tuple(ROS_NUMBER_TYPES)

    @property
    def is_function_call(self):
        return True

    def children(self):
        return self.arguments

    def __eq__(self, other):
        if not isinstance(other, HplFunctionCall):
            return False
        return (self.function == other.function
                and self.arguments == other.arguments)

    def __hash__(self):
        return 31 * hash(self.function) + hash(self.arguments)

    def __str__(self):
        return "{}({})".format(self.function,
            ", ".join(str(arg) for arg in self.arguments))

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.function), repr(self.arguments))
