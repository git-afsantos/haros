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

from builtins import range # Python 2 and 3: forward-compatible
from collections import deque, namedtuple
from itertools import chain as iterchain

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


###############################################################################
# Top-level Classes
###############################################################################

class HplAstObject(object):
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
    __slots__ = ("topic", "msg_filter")

    def __init__(self, topic, msg_filter):
        self.topic = topic # string
        self.msg_filter = msg_filter # HplMessageFilter

    def children(self):
        return ()

    def __eq__(self, other):
        if not isinstance(other, HplAssumption):
            return False
        return (self.topic == other.topic
                and self.msg_filter == other.msg_filter)

    def __hash__(self):
        return 31 * hash(self.topic) + hash(self.msg_filter)

    def __str__(self):
        return "{} {}".format(self.topic, self.msg_filter)

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__,
            repr(self.topic), repr(self.msg_filter))


class HplProperty(HplAstObject):
    __slots__ = ("scope", "pattern")

    def __init__(self, scope, pattern):
        self.scope = scope # HplScope
        self.pattern = pattern # HplPattern

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
        self.event_type = event_type
        self.predicate = predicate # HplCondition
        self.topic = topic # string
        self.alias = alias # string
        self.msg_type = None # .ros_types.TypeToken

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
# Predicates and Conditions
###############################################################################

class HplCondition(HplAstObject):
    __slots__ = ()

    @property
    def is_atomic(self):
        return False

    @property
    def is_trivial(self):
        return False


class HplVacuousTruth(HplCondition):
    __slots__ = ()

    @property
    def is_atomic(self):
        return True

    @property
    def is_trivial(self):
        return True


class HplQuantifier(HplCondition):
    __slots__ = ("quantifier", "variable", "domain", "condition")

    def __init__(self, qt, var, ran, phi):
        self.quantifier = qt # string
        self.variable = var # string
        self.domain = ran # HplValue
        self.condition = phi # HplCondition

    @property
    def u(self):
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
        return "{} {} in {}: ({})".format(self.quantifier, self.variable,
            self.domain, self.condition)

    def __repr__(self):
        return "{}({}, {}, {}, {})".format(
            type(self).__name__, repr(self.quantifier), repr(self.variable),
            repr(self.domain), repr(self.condition))


class HplConnective(HplCondition):
    __slots__ = ()

    @property
    def arity(self):
        return 0

    @property
    def is_unary(self):
        return False

    @property
    def is_binary(self):
        return False


class HplUnaryConnective(HplConnective):
    __slots__ = ("connective", "condition")

    def __init__(self, con, p):
        self.connective = con # string
        self.condition = p # HplCondition

    @property
    def is_unary(self):
        return True

    @property
    def arity(self):
        return 1

    @property
    def u(self):
        return self.connective

    @property
    def p(self):
        return self.condition

    @property
    def phi(self):
        return self.condition

    def children(self):
        return (self.condition,)

    def __eq__(self, other):
        if not isinstance(other, HplUnaryConnective):
            return False
        return (self.connective == other.connective
                and self.condition == other.condition)

    def __hash__(self):
        return 31 * hash(self.connective) + hash(self.condition)

    def __str__(self):
        return "{}({})".format(self.connective, self.condition)

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.connective), repr(self.condition))


class HplBinaryConnective(HplConnective):
    __slots__ = ("connective", "condition1", "condition2", "commutative")

    def __init__(self, con, p, q, commutative=False):
        self.connective = con # string
        self.condition1 = p # HplCondition
        self.condition2 = q # HplCondition
        self.commutative = commutative # bool

    @property
    def is_binary(self):
        return True

    @property
    def arity(self):
        return 2

    @property
    def b(self):
        return self.connective

    @property
    def p(self):
        return self.condition1

    @property
    def phi(self):
        return self.condition1

    @property
    def q(self):
        return self.condition2

    @property
    def psi(self):
        return self.condition2

    def children(self):
        return (self.condition1, self.condition2)

    def __eq__(self, other):
        if not isinstance(other, HplBinaryConnective):
            return False
        if self.connective != other.connective:
            return False
        if self.commutative != other.commutative:
            return False
        a = self.condition1
        b = self.condition2
        x = other.condition1
        y = other.condition2
        if self.commutative:
            return (a == x and b == y) or (a == y and b == x)
        return a == x and b == y

    def __hash__(self):
        h = 31 * hash(self.connective) + hash(self.condition1)
        h = 31 * h + hash(self.condition2)
        h = 31 * h + hash(self.commutative)
        return h

    def __str__(self):
        a = "({})".format(self.condition1)
        b = "({})".format(self.condition2)
        return "{} {} {}".format(a, self.operator, b)

    def __repr__(self):
        return "{}({}, {}, {}, commutative={})".format(
            type(self).__name__, repr(self.connective), repr(self.condition1),
            repr(self.condition2), repr(self.commutative))


class HplRelationalOperator(HplCondition):
    __slots__ = ("operator", "value1", "value2", "commutative")

    OP_EQ = "="
    OP_NEQ = "!="
    OP_LT = "<"
    OP_LTE = "<="
    OP_GT = ">"
    OP_GTE = ">="
    OP_IN = "in"

    def __init__(self, op, value1, value2, commutative=False):
        self.operator = op # string
        self.value1 = value1 # HplValue
        self.value2 = value2 # HplValue
        self.commutative = commutative # bool

    @property
    def is_atomic(self):
        return True

    @property
    def infix(self):
        return True

    @property
    def is_eq(self):
        return self.operator == self.OP_EQ

    @property
    def is_neq(self):
        return self.operator == self.OP_NEQ

    @property
    def is_lt(self):
        return self.operator == self.OP_LT

    @property
    def is_lte(self):
        return self.operator == self.OP_LTE

    @property
    def is_gt(self):
        return self.operator == self.OP_GT

    @property
    def is_gte(self):
        return self.operator == self.OP_GTE

    @property
    def is_in(self):
        return self.operator == self.OP_IN

    def children(self):
        return (self.value1, self.value2)

    def __eq__(self, other):
        if not isinstance(other, HplRelationalOperator):
            return False
        if self.operator != other.operator:
            return False
        if self.commutative != other.commutative:
            return False
        a = self.value1
        b = self.value2
        x = other.value1
        y = other.value2
        if self.commutative:
            return (a == x and b == y) or (a == y and b == x)
        return a == x and b == y

    def __hash__(self):
        h = 31 * hash(self.operator) + hash(self.value1)
        h = 31 * h + hash(self.value2)
        h = 31 * h + hash(self.commutative)
        return h

    def __str__(self):
        a = "({})".format(self.value1)
        b = "({})".format(self.value2)
        return "{} {} {}".format(a, self.operator, b)

    def __repr__(self):
        return "{}({}, {}, {}, commutative={})".format(
            type(self).__name__, repr(self.operator), repr(self.value1),
            repr(self.value2), repr(self.commutative))


###############################################################################
# Values and Field References
###############################################################################

class HplValue(HplAstObject):
    __slots__ = ()

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
    def is_operator(self):
        return False

    @property
    def is_function_call(self):
        return False

    def external_references(self):
        return set()


class HplLiteral(HplValue):
    __slots__ = ("token", "value", "ros_types")

    def __init__(self, token, value):
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


class HplSet(HplValue):
    __slots__ = ("values", "ros_types")

    def __init__(self, values):
        self.values = values # [HplValue]
        ros_types = set(ROS_PRIMITIVE_TYPES)
        #for value in values:
        #    ros_types = ros_types & set(value.ros_types)
        #if not ros_types:
        #    raise TypeError("mixed incompatible types: " + repr(values))
        self.ros_types = tuple(ros_types)

    @property
    def is_set(self):
        return True

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
        return "[{}]".format(", ".join(str(v) for v in self.values))

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.values))


class HplRange(HplValue):
    __slots__ = ("lower_bound", "upper_bound", "exclude_lower",
                 "exclude_upper", "ros_types")

    def __init__(self, lower, upper, exc_lower=False, exc_upper=False):
        self.lower_bound = lower # HplValue
        self.upper_bound = upper # HplValue
        self.exclude_lower = exc_lower # bool
        self.exclude_upper = exc_upper # bool
        if not any(t in ROS_NUMBER_TYPES for t in lower.ros_types):
            raise TypeError("not a number: '{}'".format(lower))
        if not any(t in ROS_NUMBER_TYPES for t in upper.ros_types):
            raise TypeError("not a number: '{}'".format(upper))
        self.ros_types = tuple(ros_type for ros_type in lower.ros_types
                               if ros_type in upper.ros_types)

    @property
    def is_range(self):
        return True

    def children(self):
        return (self.lower_bound, self.upper_bound)

    def __eq__(self, other):
        if not isinstance(other, HplRange):
            return False
        return (self.lower_bound == other.lower_bound
                and self.upper_bound == other.upper_bound
                and self.exclude_lower == other.exclude_lower
                and self.exclude_upper == other.exclude_upper)

    def __hash__(self):
        h = 31 * hash(self.lower_bound) + hash(self.upper_bound)
        h = 31 * h + hash(self.exclude_lower)
        h = 31 * h + hash(self.exclude_upper)
        return h

    def __str__(self):
        lb = str(self.lower_bound)
        if not lb.endswith(")") and self.exclude_lower:
            lb = "({})!".format(lb)
        ub = str(self.upper_bound)
        if not ub.endswith(")") and self.exclude_upper:
            ub = "({})!".format(ub)
        return "{} to {}".format(lb, ub)

    def __repr__(self):
        return "{}({}, {}, exc_lower={}, exc_upper={})".format(
            type(self).__name__, repr(self.lower_bound), repr(self.upper_bound),
            repr(self.exclude_lower), repr(self.exclude_upper))


class HplVarReference(HplValue):
    __slots__ = ("name", "is_message", "ros_types")

    def __init__(self, name, msg=False):
        self.name = name # string
        self.is_message = msg
        self.ros_types = tuple(ROS_PRIMITIVE_TYPES)

    @property
    def is_variable(self):
        return True

    def __eq__(self, other):
        if not isinstance(other, HplVarReference):
            return False
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

    def __str__(self):
        return "@{}".format(self.name)

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.name))


class HplOperator(HplValue):
    __slots__ = ()

    @property
    def is_operator(self):
        return True

    @property
    def arity(self):
        return 0

    @property
    def is_unary(self):
        return False

    @property
    def is_binary(self):
        return False


class HplUnaryOperator(HplValue):
    __slots__ = ("operator", "value", "ros_types")

    def __init__(self, op, value):
        self.operator = op # string
        self.value = value # HplValue
        self.ros_types = tuple(ROS_NUMBER_TYPES)

    @property
    def is_unary(self):
        return True

    @property
    def arity(self):
        return 1

    def children(self):
        return (self.value,)

    def __eq__(self, other):
        if not isinstance(other, HplUnaryOperator):
            return False
        return (self.operator == other.operator
                and self.value == other.value)

    def __hash__(self):
        return 31 * hash(self.operator) + hash(self.value)

    def __str__(self):
        return "{}({})".format(self.operator, self.value)

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.operator), repr(self.value))


class HplBinaryOperator(HplValue):
    __slots__ = ("operator", "value1", "value2", "infix",
                 "commutative", "ros_types")

    def __init__(self, op, value1, value2, infix=True, commutative=False):
        self.operator = op # string
        self.value1 = value1 # HplValue
        self.value2 = value2 # HplValue
        self.infix = infix # bool
        self.commutative = commutative # bool
        self.ros_types = tuple(ROS_NUMBER_TYPES)

    @property
    def is_binary(self):
        return True

    @property
    def arity(self):
        return 2

    def children(self):
        return (self.value1, self.value2)

    def __eq__(self, other):
        if not isinstance(other, HplBinaryOperator):
            return False
        if self.operator != other.operator:
            return False
        if self.infix != other.infix or self.commutative != other.commutative:
            return False
        a = self.value1
        b = self.value2
        x = other.value1
        y = other.value2
        if self.commutative:
            return (a == x and b == y) or (a == y and b == x)
        return a == x and b == y

    def __hash__(self):
        h = 31 * hash(self.operator) + hash(self.value1)
        h = 31 * h + hash(self.value2)
        h = 31 * h + hash(self.infix)
        h = 31 * h + hash(self.commutative)
        return h

    def __str__(self):
        a = "({})".format(self.value1)
        b = "({})".format(self.value2)
        if self.infix:
            return "{} {} {}".format(a, self.operator, b)
        else:
            return "{}({}, {})".format(self.operator, a, b)

    def __repr__(self):
        return "{}({}, {}, {}, infix={}, commutative={})".format(
            type(self).__name__, repr(self.operator), repr(self.value1),
            repr(self.value2), repr(self.infix), repr(self.commutative))


class HplFunctionCall(HplValue):
    __slots__ = ("function", "arguments", "ros_types")

    def __init__(self, fun, args):
        self.function = fun # string
        self.arguments = args # [HplValue]
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


class HplFieldReference(HplValue):
    __slots__ = ("token", "message", "ros_types", "_parts")

    def __init__(self, token, message=None):
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
                is_var = index.startswith("@")
                parts.append(HplFieldAccessor(index, True, is_var))
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
