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
# Constants
###############################################################################

INF = float("inf")


###############################################################################
# Exceptions
###############################################################################

class HplSanityError(Exception):
    pass


class HplTypeError(Exception):
    pass

    @classmethod
    def ros_field(cls, rostype, field, expr):
        return cls("ROS '{}' has no field '{}': {}".format(
            rostype, field, expr))

    @classmethod
    def ros_array(cls, rostype, expr):
        return cls("ROS '{}' is not an array: {}".format(
            rostype, expr))

    @classmethod
    def ros_index(cls, rostype, idx, expr):
        return cls("ROS '{}' index {} out of range: {}".format(
            rostype, idx, expr))


###############################################################################
# Helper Functions
###############################################################################

def isclose(a, b, rel_tol=1e-06, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


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
    def is_predicate(self):
        return False

    @property
    def is_expression(self):
        return False

    def children(self):
        return ()

    def iterate(self):
        stack = [self]
        while stack:
            obj = stack.pop()
            stack.extend(reversed(obj.children()))
            yield obj


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

    def sanity_check(self):
        for obj in self.predicate.iterate():
            if obj.is_expression and obj.is_accessor:
                if obj.is_field and obj.message.is_value:
                    if obj.message.is_variable:
                        raise HplSanityError(
                            "assumptions cannot reference events: " + str(obj))

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

    def is_fully_typed(self):
        for event in self.events():
            if not event.predicate.is_fully_typed():
                return False
        return True

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
        return cls(cls.PREVENTION, forbidden, event,
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
                and isclose(self.min_time, other.min_time)
                and ((self.max_time == INF and other.max_time == INF)
                    or isclose(self.max_time, other.max_time)))

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
        if not predicate.is_predicate:
            raise TypeError("not a predicate: " + str(predicate))
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
        for obj in self.predicate.iterate():
            if obj.is_expression and obj.is_accessor:
                if obj.is_field and obj.message.is_value:
                    if obj.message.is_variable:
                        refs.add(obj.message.name)
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
        alias = (" as " + self.alias) if self.alias is not None else ""
        if self.event_type == self.PUBLISH:
            return "{}{} {}".format(self.topic, alias, self.predicate)
        else:
            assert False, "unexpected event type"

    def __repr__(self):
        return "{}({}, {}, {}, alias={})".format(
            type(self).__name__, repr(self.event_type), repr(self.predicate),
            repr(self.topic), repr(self.alias))


###############################################################################
# Top-level Predicate
###############################################################################

class HplPredicate(HplAstObject):
    __slots__ = ("condition",)

    _DIFF_TYPES = ("multiple occurrences of '{}' with incompatible types: "
                   "found ({}) and ({})")
    _NO_REFS = "there are no references to any fields of this message"

    def __init__(self, expr):
        if not expr.is_expression:
            raise TypeError("not an expression: " + str(expr))
        if not expr.can_be_bool:
            raise HplTypeError("not a boolean expression: " + str(expr))
        self.condition = expr
        self._static_checks()

    @property
    def is_predicate(self):
        return True

    @property
    def is_vacuous(self):
        return False

    @property
    def phi(self):
        return self.condition

    def is_fully_typed(self):
        return self.condition.is_fully_typed()

    def children(self):
        return (self.condition,)

    def negate(self):
        if self.condition.is_operator and self.condition.operator == "not":
            return HplPredicate(self.condition.operand)
        return HplPredicate(HplUnaryOperator("not", self.condition))

    def join(self, other):
        if other.is_vacuous:
            return self if other.is_true else other
        expr = HplBinaryOperator("and", self.condition, other.condition)
        return HplPredicate(expr)

    def refine_types(self, rostype, **kwargs):
        # rostype: ROS Type Token
        # kwargs: string (alias) -> ROS Type Token
        stack = [self.condition]
        while stack:
            obj = stack.pop()
            if obj.is_accessor:
                self._refine_type(obj, rostype, **kwargs)
            else:
                stack.extend(reversed(obj.children()))

    def _refine_type(self, accessor, rostype, **kwargs):
        stack = [accessor]
        expr = accessor.message
        while expr.is_accessor:
            stack.append(expr)
            expr = expr.message
        assert expr.is_value and (expr.is_this_msg or expr.is_variable)
        if expr.is_this_msg:
            t = rostype
        else:
            if expr.name not in kwargs:
                raise HplSanityError(
                    "undefined message alias: '{}'".format(expr.name))
            t = kwargs[expr.name]
        assert t.is_message
        expr.ros_type = t
        while stack:
            expr = stack.pop()
            if expr.is_field:
                if not (t.is_message or expr.field in t.fields
                        or expr.field in t.constants):
                    raise HplTypeError.ros_field(t, expr.field, expr)
                if expr.field in t.fields:
                    t = t.fields[expr.field]
                else:
                    assert expr.field in t.constants, \
                        "'{}' not in {} or {}".format(
                            expr.field, t.fields, t.constants)
                    t = t.constants[expr.field].ros_type
            else:
                assert expr.is_indexed
                if not t.is_array:
                    raise HplTypeError.ros_array(t, expr)
                i = expr.index
                if (i.is_value and i.is_literal
                        and not t.contains_index(i.value)):
                    raise HplTypeError.ros_index(t, expr.index, expr)
                t = t.type_token
            if t.is_message:
                accessor._type_check(expr, T_MSG)
            elif t.is_array:
                accessor._type_check(expr, T_ARR)
            elif t.is_number:
                accessor._type_check(expr, T_NUM)
                # TODO check that values fit within types
            elif t.is_bool:
                accessor._type_check(expr, T_BOOL)
            elif t.is_string:
                accessor._type_check(expr, T_STR)
            expr.ros_type = t

    def _static_checks(self):
        ref_table = {}
        for obj in self.condition.iterate():
            if obj.is_accessor or (obj.is_value and obj.is_variable):
                key = str(obj)
                refs = ref_table.get(key)
                if refs is None:
                    refs = []
                    ref_table[key] = refs
                refs.append(obj)
        self._all_refs_same_type(ref_table)
        self._some_field_refs(ref_table)

    def _all_refs_same_type(self, table):
        # All references to the same field/variable have the same type.
        for key, refs in table.iteritems():
            # must traverse twice, in case we start with the most generic
            # and go down to the most specific
            final_type = T_ANY
            for ref in refs:
                ref.cast(final_type)
                final_type = ref.types
            for ref in reversed(refs):
                ref.cast(final_type)
                final_type = ref.types

    def _some_field_refs(self, table):
        # There is, at least, one reference to a field (own).
        #   [NYI] Stricter: one reference per atomic condition.
        for refs in table.itervalues():
            for ref in refs:
                if not ref.is_accessor:
                    break
                if ref.is_indexed:
                    break
                if not ref.message.is_value:
                    break
                assert ref.message.is_reference
                if not ref.message.is_this_msg:
                    break
                return # OK
        raise HplSanityError(self._NO_REFS)

    def __eq__(self, other):
        if not isinstance(other, HplPredicate):
            return False
        return self.condition == other.condition

    def __hash__(self):
        return hash(self.condition)

    def __str__(self):
        return "{{ {} }}".format(self.condition)

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.condition))


class HplVacuousTruth(HplAstObject):
    __slots__ = ()

    @property
    def is_predicate(self):
        return True

    @property
    def is_vacuous(self):
        return True

    @property
    def is_true(self):
        return True

    def is_fully_typed(self):
        return True

    def negate(self):
        return HplContradiction()

    def join(self, other):
        return other

    def refine_types(self, rostype, **kwargs):
        pass

    def __eq__(self, other):
        return isinstance(other, HplVacuousTruth)

    def __hash__(self):
        return 27644437

    def __str__(self):
        return "{ True }"

    def __repr__(self):
        return "{}()".format(type(self).__name__)


class HplContradiction(HplAstObject):
    __slots__ = ()

    @property
    def is_predicate(self):
        return True

    @property
    def is_vacuous(self):
        return True

    @property
    def is_true(self):
        return False

    def is_fully_typed(self):
        return True

    def negate(self):
        return HplVacuousTruth()

    def join(self, other):
        return self

    def refine_types(self, rostype, **kwargs):
        pass

    def __eq__(self, other):
        return isinstance(other, HplContradiction)

    def __hash__(self):
        return 65537

    def __str__(self):
        return "{ False }"

    def __repr__(self):
        return "{}()".format(type(self).__name__)


###############################################################################
# Type System
###############################################################################

# Bit Flags

# These work as possible types for the expression.
# An expression of unknown type would have "any" type (i.e., all flags on).
# E.g., (T_NUM | T_BOOL) means the expression can be either a number or a bool.
# Things like variables start with many possible types, and are refined as the
# tree is built.

T_BOOL = 0x1
T_NUM = 0x2
T_STR = 0x4
T_ARR = 0x8

T_RAN = 0x10
T_SET = 0x20
T_MSG = 0x40

T_ANY = T_BOOL | T_NUM | T_STR | T_ARR | T_RAN | T_SET | T_MSG
T_COMP = T_ARR | T_RAN | T_SET
T_PRIM = T_BOOL | T_NUM | T_STR
T_ROS = T_BOOL | T_NUM | T_STR | T_ARR | T_MSG
T_ITEM = T_BOOL | T_NUM | T_STR | T_MSG

_TYPE_NAMES = {
    T_BOOL: "boolean",
    T_NUM: "number",
    T_STR: "string",
    T_ARR: "array",
    T_RAN: "range",
    T_SET: "set",
    T_MSG: "ROS msg",
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
    def is_function_call(self):
        return False

    @property
    def is_quantifier(self):
        return False

    @property
    def is_accessor(self):
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

    @property
    def can_be_msg(self):
        return bool(self.types & T_MSG)

    def can_be(self, t):
        return bool(self.types & t)

    def is_fully_typed(self):
        for obj in self.iterate():
            t = obj.types
            if (not t) or bool(t & (t - 1)): # not a power of 2
                return False
        return True

    def cast(self, t):
        r = self.types & t
        if not r:
            raise HplTypeError("expected ({}) but found ({}): {}".format(
                type_name(t), type_name(self.types), self))
        self.types = r

    def _type_check(self, x, t):
        try:
            x.cast(t)
        except HplTypeError as e:
            msg = "Type error in expression '{}':\n{}".format(self, e)
            raise HplTypeError(msg)

    def add_type(self, t):
        self.types = self.types | t

    def rem_type(self, t):
        self.types = self.types & ~t
        if not self.types:
            raise HplTypeError("no types left: " + str(self))


###############################################################################
# Quantifiers
###############################################################################

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
# Operators and Functions
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


class HplFunctionCall(HplExpression):
    __slots__ = HplExpression.__slots__ + ("function", "arguments",)

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
        try:
            tin, tout = self._BUILTINS[fun]
        except KeyError:
            raise HplTypeError("undefined function '{}'".format(fun))
        HplExpression.__init__(self, types=tout)
        self.function = fun # string
        self.arguments = args # [HplValue]
        for arg in args:
            self._type_check(arg, tin)

    @property
    def is_function_call(self):
        return True

    @property
    def arity(self):
        return len(self.arguments)

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


###############################################################################
# Message Field Access
###############################################################################

class HplFieldAccess(HplExpression):
    __slots__ = HplExpression.__slots__ + ("message", "field", "ros_type")

    def __init__(self, msg, field):
        HplExpression.__init__(self, types=T_ROS)
        self.message = msg # HplExpression
        self.field = field # string
        self.ros_type = None
        self._type_check(msg, T_MSG)

    @property
    def is_accessor(self):
        return True

    @property
    def is_field(self):
        return True

    @property
    def is_indexed(self):
        return False

    def base_message(self):
        obj = self
        while obj.is_accessor:
            obj = obj.message
        assert obj.is_value
        return obj

    def children(self):
        return (self.message,)

    def __eq__(self, other):
        if not isinstance(other, HplFieldAccess):
            return False
        return (self.message == other.message
                and self.field == other.field)

    def __hash__(self):
        return 31 * hash(self.message) + hash(self.field)

    def __str__(self):
        msg = str(self.message)
        if msg:
            return "{}.{}".format(msg, self.field)
        return str(self.field)

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.message), repr(self.field))


class HplArrayAccess(HplExpression):
    __slots__ = HplExpression.__slots__ + ("array", "item", "ros_type")

    _MULTI_ARRAY = "multi-dimensional array access: '{}[{}]'"

    def __init__(self, array, index):
        if array.is_accessor and array.is_indexed:
            raise HplTypeError(self._MULTI_ARRAY.format(array, index))
        HplExpression.__init__(self, types=T_ITEM)
        self.array = array # HplExpression
        self.index = index # HplExpression
        self.ros_type = None
        self._type_check(array, T_ARR)
        self._type_check(index, T_NUM)

    @property
    def is_accessor(self):
        return True

    @property
    def is_field(self):
        return False

    @property
    def is_indexed(self):
        return True

    @property
    def message(self):
        return self.array

    def base_message(self):
        obj = self
        while obj.is_accessor:
            obj = obj.message
        assert obj.is_value
        return obj

    def children(self):
        return (self.array, self.index)

    def __eq__(self, other):
        if not isinstance(other, HplArrayAccess):
            return False
        return (self.array == other.array
                and self.index == other.index)

    def __hash__(self):
        return 31 * hash(self.array) + hash(self.index)

    def __str__(self):
        return "{}[{}]".format(self.array, self.index)

    def __repr__(self):
        return "{}({}, {})".format(
            type(self).__name__, repr(self.array), repr(self.index))


###############################################################################
# Values
###############################################################################

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
    def is_this_msg(self):
        return False


###############################################################################
# Compound Values
###############################################################################

class HplSet(HplValue):
    __slots__ = HplValue.__slots__ + ("values",)

    def __init__(self, values):
        HplValue.__init__(self, types=T_SET)
        self.values = values # [HplValue]
        for value in values:
            self._type_check(value, T_PRIM)

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
        "min_value", "max_value", "exclude_min", "exclude_max")

    def __init__(self, lb, ub, exc_min=False, exc_max=False):
        HplValue.__init__(self, types=T_RAN)
        self.min_value = lb # HplValue
        self.max_value = ub # HplValue
        self.exclude_min = exc_min # bool
        self.exclude_max = exc_max # bool
        self._type_check(lb, T_NUM)
        self._type_check(ub, T_NUM)

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
    __slots__ = HplValue.__slots__ + ("token", "value",)

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


class HplThisMessage(HplValue):
    __slots__ = HplValue.__slots__ + ("ros_type",)

    def __init__(self):
        HplValue.__init__(self, types=T_MSG)
        self.ros_type = None

    @property
    def is_reference(self):
        return True

    @property
    def is_this_msg(self):
        return True

    def __eq__(self, other):
        return isinstance(other, HplThisMessage)

    def __hash__(self):
        return 433494437

    def __str__(self):
        return ""

    def __repr__(self):
        return "{}()".format(type(self).__name__)


class HplVarReference(HplValue):
    __slots__ = HplValue.__slots__ + ("token", "defined_at", "ros_type")

    def __init__(self, token):
        HplValue.__init__(self, types=T_ITEM)
        self.token = token # string
        self.defined_at = None
        self.ros_type = None

    @property
    def is_reference(self):
        return True

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

