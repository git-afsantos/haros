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
from collections import namedtuple
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
    pass


class HplAssumption(HplAstObject):
    __slots__ = ("topic", "msg_filter")

    def __init__(self, topic, msg_filter):
        self.topic = topic # string
        self.msg_filter = msg_filter # HplMessageFilter

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
    __slots__ = ("scope", "observable")

    def __init__(self, scope, observable):
        self.scope = scope # HplScope
        self.observable = observable # HplObservable

    @property
    def is_safety(self):
        return self.observable.is_safety

    @property
    def is_liveness(self):
        return self.observable.is_liveness

    def events(self):
        if self.scope.activator is not None:
            for event in self.scope.activator.events():
                yield event
        for event in self.observable.behaviour.events():
            yield event
        if self.observable.trigger is not None:
            for event in self.observable.trigger.events():
                yield event
        if self.scope.terminator is not None:
            for event in self.scope.terminator.events():
                yield event

    def sanity_check(self):
        initial = self._check_activator()
        aliases = self._check_trigger(initial)
        aliases = self._check_behaviour(aliases)
        self._check_terminator(initial)
        self._check_duplicate_aliases()

    def _check_activator(self):
        if self.scope.activator is not None:
            self.scope.activator.sanity_check()
            refs = self.scope.activator.external_references()
            if refs:
                raise HplSanityError(
                    "references to undefined events: " + repr(refs))
            return self.scope.activator.aliases()
        return set()

    def _check_trigger(self, available):
        if not (self.observable.is_response or self.observable.is_requirement):
            return available
        assert self.observable.trigger is not None
        self.observable.trigger.sanity_check()
        if self.observable.is_response:
            for ref in self.observable.trigger.external_references():
                if ref not in available:
                    raise HplSanityError(
                        "reference to undefined event: " + repr(ref))
            aliases = self.observable.trigger.aliases()
            aliases.update(available)
            return aliases
        else:
            future_alias = set()
            for event in self.observable.behaviour.leaves():
                if event.alias is not None:
                    future_alias.add(event.alias)
            for ref in self.observable.trigger.external_references():
                if ref not in available and ref not in future_alias:
                    raise HplSanityError(
                        "reference to undefined event: " + repr(ref))
            for event in self.observable.trigger.events():
                for condition in event.msg_filter.conditions:
                    v = condition.value
                    if v.is_reference and v.message in future_alias:
                        if not condition.is_invertible:
                            raise HplSanityError(
                                ("reference to future event in non-invertible "
                                 "condition: ") + repr(v.message))
            return available

    def _check_behaviour(self, available):
        self.observable.behaviour.sanity_check()
        if self.observable.is_requirement:
            for prefix in self.observable.behaviour.prefixes():
                for event in prefix:
                    for ref in event.external_references():
                        if ref not in available:
                            raise HplSanityError(
                                "reference to undefined event: " + repr(ref))
            trigger = self.observable.trigger.aliases()
            for event in self.observable.behaviour.leaves():
                for ref in event.external_references():
                    if ref not in available and ref not in trigger:
                        raise HplSanityError(
                            "reference to undefined event: " + repr(ref))
        else:
            for ref in self.observable.behaviour.external_references():
                if ref not in available:
                    raise HplSanityError(
                        "reference to undefined event: " + repr(ref))
            if self.observable.is_response:
                limit = self.observable.max_time
                if limit < INF:
                    for chain in self.observable.behaviour.chains:
                        delay = 0.0
                        for event in chain.events:
                            delay += event.delay
                        if delay >= limit:
                            raise HplSanityError(
                                ("delay between behaviour events "
                                 "is greater than ") + str(limit))
        aliases = self.observable.behaviour.aliases()
        aliases.update(available)
        if self.observable.is_requirement:
            aliases.update(self.observable.trigger.aliases())
        return aliases

    def _check_terminator(self, available):
        if self.scope.terminator is not None:
            self.scope.terminator.sanity_check()
            for ref in self.scope.terminator.external_references():
                if not ref in available:
                    raise HplSanityError(
                        "reference to undefined event: " + repr(ref))

    def _check_duplicate_aliases(self):
        events = []
        aliases = set()
        if self.scope.activator is not None:
            events.extend(self.scope.activator.events())
        if self.observable.trigger is not None:
            events.extend(self.observable.trigger.events())
        events.extend(self.observable.behaviour.events())
        if self.scope.terminator is not None:
            events.extend(self.scope.terminator.events())
        for event in events:
            if not event.alias:
                continue
            if event.alias in aliases:
                raise HplSanityError("duplicate alias: " + event.alias)
            aliases.add(event.alias)

    def __eq__(self, other):
        if not isinstance(other, HplProperty):
            return False
        return (self.observable == other.observable
                and self.scope == other.scope)

    def __hash__(self):
        return 31 * hash(self.scope) + hash(self.observable)

    def __str__(self):
        return "{}: {}".format(self.scope, self.observable)

    def __repr__(self):
        return "{}({}, {})".format(type(self).__name__,
            repr(self.scope), repr(self.observable))


class HplScope(HplAstObject):
    __slots__ = ("scope_type", "activator", "terminator", "timeout")

    GLOBAL = 1
    AFTER_UNTIL = 2
    AFTER_WITHIN = 3

    def __init__(self, scope, activator=None, terminator=None, timeout=INF):
        if scope == self.GLOBAL:
            if activator is not None:
                raise ValueError(activator)
            if terminator is not None:
                raise ValueError(terminator)
            if timeout < INF:
                raise ValueError(timeout)
        elif scope == self.AFTER_UNTIL:
            if timeout < INF:
                raise ValueError(timeout)
        elif scope == self.AFTER_WITHIN:
            if terminator is not None:
                raise ValueError(terminator)
        else:
            raise ValueError(scope)
        self.scope_type = scope
        self.activator = activator # HplTopLevelEvent | None
        self.terminator = terminator # HplTopLevelEvent | None
        self.timeout = timeout # float

    @classmethod
    def globally(cls):
        return cls(cls.GLOBAL)

    @classmethod
    def after(cls, activator, terminator=None, timeout=INF):
        if timeout < INF and terminator is not None:
            raise ValueError("cannot specify both terminator and timeout")
        if timeout < INF:
            return cls(cls.AFTER_WITHIN, activator=activator, timeout=timeout)
        return cls(cls.AFTER_UNTIL, activator=activator, terminator=terminator)

    @property
    def is_global(self):
        return self.scope_type == self.GLOBAL

    @property
    def is_after(self):
        return (self.scope_type == self.AFTER_UNTIL
                or self.scope_type == self.AFTER_WITHIN)

    @property
    def is_after_until(self):
        return self.scope_type == self.AFTER_UNTIL

    @property
    def is_after_within(self):
        return self.scope_type == self.AFTER_WITHIN

    def __eq__(self, other):
        if not isinstance(other, HplScope):
            return False
        return (self.scope_type == other.scope_type
                and self.activator == other.activator
                and self.terminator == other.terminator
                and self.timeout == other.timeout)

    def __hash__(self):
        h = 31 * hash(self.scope_type) + hash(self.activator)
        h = 31 * h + hash(self.terminator)
        return 31 * h + hash(self.timeout)

    def __str__(self):
        activator = "launch" if self.activator is None else str(self.activator)
        terminator = ("shutdown" if self.terminator is None
                                 else str(self.terminator))
        if self.scope_type == self.GLOBAL:
            return "globally"
        if self.scope_type == self.AFTER_UNTIL:
            return "after {} until {}".format(activator, terminator)
        if self.scope_type == self.AFTER_WITHIN:
            return "within {} after {}".format(self.timeout, activator)
        assert False, "unexpected scope type"

    def __repr__(self):
        return "{}({}, activator={}, terminator={}, timeout={})".format(
            type(self).__name__, self.scope_type, repr(self.activator),
            repr(self.terminator), repr(self.timeout))


class HplObservable(HplAstObject):
    __slots__ = ("pattern", "behaviour", "trigger", "min_time", "max_time")

    EXISTENCE = 1
    ABSENCE = 2
    RESPONSE = 3
    REQUIREMENT = 4

    def __init__(self, pattern, behaviour, trigger, min_time=0.0, max_time=INF):
        if pattern == self.EXISTENCE or pattern == self.ABSENCE:
            if trigger is not None:
                raise ValueError(trigger)
        elif pattern != self.RESPONSE and pattern != self.REQUIREMENT:
            raise ValueError(pattern)
        self.pattern = pattern
        self.behaviour = behaviour # HplTopLevelEvent
        self.trigger = trigger # HplTopLevelEvent | None
        self.min_time = min_time
        self.max_time = max_time

    @classmethod
    def existence(cls, behaviour):
        return cls(cls.EXISTENCE, behaviour, None)

    @classmethod
    def absence(cls, behaviour):
        return cls(cls.ABSENCE, behaviour, None)

    @classmethod
    def response(cls, event, response, min_time=0.0, max_time=INF):
        return cls(cls.RESPONSE, response, event,
            min_time=min_time, max_time=max_time)

    @classmethod
    def requirement(cls, event, requirement, min_time=0.0, max_time=INF):
        return cls(cls.REQUIREMENT, event, requirement,
            min_time=min_time, max_time=max_time)

    @property
    def is_safety(self):
        return (self.pattern == self.ABSENCE
                or self.pattern == self.REQUIREMENT)

    @property
    def is_liveness(self):
        return (self.pattern == self.EXISTENCE
                or self.pattern == self.RESPONSE)

    @property
    def is_absence(self):
        return self.pattern == self.ABSENCE

    @property
    def is_existence(self):
        return self.pattern == self.EXISTENCE

    @property
    def is_requirement(self):
        return self.pattern == self.REQUIREMENT

    @property
    def is_response(self):
        return self.pattern == self.RESPONSE

    def __eq__(self, other):
        if not isinstance(other, HplObservable):
            return False
        return (self.pattern == other.pattern
                and self.behaviour == other.behaviour
                and self.trigger == other.trigger)

    def __hash__(self):
        h = 31 * hash(self.pattern) + hash(self.behaviour)
        return 31 * h + hash(self.trigger)

    def __str__(self):
        if self.pattern == self.EXISTENCE:
            return "some {}".format(self.behaviour)
        if self.pattern == self.ABSENCE:
            return "no {}".format(self.behaviour)
        if self.pattern == self.RESPONSE:
            t = ""
            if self.max_time < INF:
                t = "within {}s ".format(self.max_time)
            return "{} causes {}{}".format(self.trigger, t, self.behaviour)
        if self.pattern == self.REQUIREMENT:
            t = ""
            if self.max_time < INF:
                t = "within {}s ".format(self.max_time)
            return "{} requires {}{}".format(self.behaviour, t, self.trigger)
        assert False, "unexpected observable pattern"

    def __repr__(self):
        return "{}({}, {}, {})".format(type(self).__name__,
            self.pattern, repr(self.behaviour), repr(self.trigger))


###############################################################################
# Events and Event Operators
###############################################################################

class HplEvent(HplAstObject):
    __slots__ = ("event_type", "msg_filter", "topic", "delay", "duration",
                 "alias", "msg_type")

    PUBLISH = 1

    def __init__(self, event_type, msg_filter, topic, delay=0.0, duration=INF,
                 alias=None):
        if event_type != self.PUBLISH:
            raise ValueError(event_type)
        self.event_type = event_type
        self.msg_filter = msg_filter # HplMessageFilter
        self.topic = topic # string
        self.delay = delay # float >= 0
        self.duration = duration # float >= 0
        self.alias = alias # string
        self.msg_type = None # .ros_types.TypeToken

    @classmethod
    def publish(cls, topic, msg_filter=None, delay=0.0,
                duration=INF, alias=None):
        if msg_filter is None:
            msg_filter = HplMessageFilter([])
        return cls(cls.PUBLISH, msg_filter, topic, delay=delay,
                   duration=duration, alias=alias)

    @property
    def is_publish(self):
        return self.event_type == self.PUBLISH

    def external_references(self):
        refs = set()
        for condition in self.msg_filter.conditions:
            if condition.value.is_set:
                for value in condition.value.values:
                    if value.is_reference and value.message is not None:
                        refs.add(value.message)
            elif condition.value.is_range:
                value = condition.value.lower_bound
                if value.is_reference and value.message is not None:
                    refs.add(value.message)
                value = condition.value.upper_bound
                if value.is_reference and value.message is not None:
                    refs.add(value.message)
            elif condition.value.is_reference:
                if condition.value.message is not None:
                    refs.add(condition.value.message)
        return refs

    def __eq__(self, other):
        if not isinstance(other, HplEvent):
            return False
        return (self.event_type == other.event_type
                and self.msg_filter == other.msg_filter
                and self.topic == other.topic
                and self.delay == other.delay
                and self.duration == other.duration)

    def __hash__(self):
        h = 31 * hash(self.event_type) + hash(self.msg_filter)
        h = 31 * h + hash(self.topic)
        h = 31 * h + hash(self.delay)
        return 31 * h + hash(self.duration)

    def __str__(self):
        timer = ""
        if self.delay > 0.0 or self.duration < INF:
            timer = "[{}s to {}s] ".format(self.delay, self.duration)
        msg = ""
        if not self.msg_filter.is_empty:
            msg = " " + str(self.msg_filter)
        alias = (" as " + self.alias) if self.alias is not None else ""
        if self.event_type == self.PUBLISH:
            return "{}{}{}{}".format(timer, self.topic, msg, alias)
        else:
            assert False, "unexpected event type"

    def __repr__(self):
        return "{}({}, {}, {}, delay={}, duration={}, alias={})".format(
            type(self).__name__, self.event_type, repr(self.msg_filter),
            self.topic, self.delay, self.duration, self.alias)


class HplEventChain(HplAstObject):
    __slots__ = ("events", "duration")

    def __init__(self, events, duration=INF):
        if len(events) < 1:
            raise ValueError(events)
        self.events = events # [HplEvent]
        self.duration = duration

    @property
    def root(self):
        return self.events[0]

    @property
    def leaf(self):
        return self.events[-1]

    def prefix(self):
        return self.events[:-1]

    def suffix(self):
        return self.events[1:]

    def aliases(self):
        return set(e.alias for e in self.events if e.alias is not None)

    def external_references(self):
        # assume sanity_check
        aliases = set(e.alias for e in self.events if e.alias is not None)
        refs = set()
        for event in self.events:
            for ref in event.external_references():
                if ref not in aliases:
                    refs.add(ref)
        return refs

    def sanity_check(self):
        aliases = []
        for ev in self.events:
            if ev.alias is not None:
                if ev.alias in aliases:
                    raise HplSanityError("duplicate alias: " + repr(ev.alias))
            aliases.append(ev.alias)
        for i in range(len(self.events)):
            for ref in self.events[i].external_references():
                for j in range(i + 1, len(aliases)):
                    if ref == aliases[j]:
                        raise HplSanityError(
                            "reference to a future event: " + repr(ref))

    def __eq__(self, other):
        if not isinstance(other, HplEventChain):
            return False
        return (self.events == other.events
                and self.duration == other.duration)

    def __hash__(self):
        h = hash(self.duration)
        for event in self.events:
            h = 31 * h + hash(event)
        return h

    def __str__(self):
        timer = ""
        if self.duration < INF:
            timer = " within {}s".format(self.duration)
        sequence = "; ".join(str(event) for event in self.events)
        return "{}{}".format(sequence, timer)

    def __repr__(self):
        return "{}({}, duration={})".format(type(self).__name__,
            repr(self.events), self.duration)


class HplTopLevelEvent(HplAstObject):
    def roots(self):
        raise NotImplementedError()

    def leaves(self):
        raise NotImplementedError()

    def prefixes(self):
        raise NotImplementedError()

    def suffixes(self):
        raise NotImplementedError()

    def events(self):
        raise NotImplementedError()

    def aliases(self):
        raise NotImplementedError()

    def external_references(self):
        raise NotImplementedError()

    def sanity_check(self):
        raise NotImplementedError()


class HplChainDisjunction(HplTopLevelEvent):
    __slots__ = ("chains",)

    def __init__(self, chains):
        # chains :: [HplEventChain]
        if len(chains) < 1:
            raise ValueError(chains)
        self.chains = chains

    def roots(self):
        return tuple(chain.root for chain in self.chains)

    def leaves(self):
        return tuple(chain.leaf for chain in self.chains)

    def prefixes(self):
        return [chain.prefix() for chain in self.chains]

    def suffixes(self):
        return [chain.suffix() for chain in self.chains]

    def events(self):
        for chain in self.chains:
            for event in chain.events:
                yield event

    def aliases(self):
        # cannot reference an event for sure when there is a disjunction
        if len(self.chains) > 1:
            return set()
        return self.chains[0].aliases()

    def external_references(self):
        # cannot reference events across disjunctions
        refs = set()
        for chain in self.chains:
            refs.update(chain.external_references())
        return refs

    def sanity_check(self):
        aliases = set()
        for chain in self.chains:
            chain.sanity_check()
            for alias in chain.aliases():
                if alias in aliases:
                    raise HplSanityError("duplicate alias: " + repr(alias))
                aliases.add(alias)
        for chain in self.chains:
            for ref in chain.external_references():
                if ref in aliases:
                    raise HplSanityError(
                        "reference across parallel chains" + repr(ref))

    def __eq__(self, other):
        if not isinstance(other, HplChainDisjunction):
            return False
        return (len(self.chains) == len(other.chains)
                and all(chain in other.chains for chain in self.chains))

    def __hash__(self):
        h = 17
        for chain in self.chains:
            h = 31 * h + hash(chain)
        return h

    def __str__(self):
        return " || ".join(str(chain) for chain in self.chains)

    def __repr__(self):
        return "{}({})".format(type(self).__name__, repr(self.chains))


###############################################################################
# Message Filters and Field Conditions
###############################################################################

class HplMessageFilter(HplAstObject):
    __slots__ = ("conditions", "length_conditions")

    def __init__(self, conditions, len_conditions=None):
        # conditions :: [HplFieldCondition]
        self.conditions = conditions
        if len_conditions is None:
            self.length_conditions = []
        else:
            self.length_conditions = len_conditions

    @property
    def is_empty(self):
        return not self.conditions and not self.length_conditions

    def __eq__(self, other):
        if not isinstance(other, HplMessageFilter):
            return False
        return (len(self.conditions) == len(other.conditions)
                and all(c in other.conditions for c in self.conditions)
                and len(self.length_conditions) == len(other.length_conditions)
                and all(c in other.length_conditions
                        for c in self.length_conditions))

    def __hash__(self):
        h = 1
        for condition in self.conditions:
            h = 31 * h + hash(condition)
        for condition in self.length_conditions:
            h = 31 * h + hash(condition)
        return h

    def __str__(self):
        conditions = ", ".join(str(c)
            for c in iterchain(self.conditions, self.length_conditions))
        return "{{{}}}".format(conditions)

    def __repr__(self):
        return "{}({}, len_conditions={})".format(type(self).__name__,
            repr(self.conditions), repr(self.length_conditions))


class HplFieldCondition(HplAstObject):
    __slots__ = ("field", "operator", "value")

    OP_EQ = "="
    OP_NEQ = "!="
    OP_LT = "<"
    OP_LTE = "<="
    OP_GT = ">"
    OP_GTE = ">="
    OP_IN = "in"
    OP_NIN = "not in"

    _NOT = {
        OP_EQ: OP_NEQ,
        OP_NEQ: OP_EQ,
        OP_LT: OP_GTE,
        OP_GT: OP_LTE,
        OP_LTE: OP_GT,
        OP_GTE: OP_LT,
        OP_IN: OP_NIN,
        OP_NIN: OP_IN
    }

    _INV = {
        OP_EQ: OP_EQ,
        OP_NEQ: OP_NEQ,
        OP_LT: OP_GT,
        OP_LTE: OP_GTE,
        OP_GT: OP_LT,
        OP_GTE: OP_LTE
    }

    def __init__(self, field_ref, op_token, hpl_value):
        if not op_token in self._NOT:
            raise ValueError("invalid operator token: " + str(op_token))
        self.field = field_ref # HplFieldReference
        self.operator = op_token # string
        self.value = hpl_value # HplValue

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

    @property
    def is_not_in(self):
        return self.operator == self.OP_NIN

    @property
    def is_equality_test(self):
        return self.operator == self.OP_EQ or self.operator == self.OP_NEQ

    @property
    def is_comparison_test(self):
        return (self.operator == self.OP_LT or self.operator == self.OP_LTE
            or self.operator == self.OP_GT or self.operator == self.OP_GTE)

    @property
    def is_inclusion_test(self):
        return self.operator == self.OP_IN or self.operator == self.OP_NIN

    @property
    def requires_number(self):
        return (self.operator == self.OP_LT or self.operator == self.OP_LTE
                or self.operator == self.OP_GT or self.operator == self.OP_GTE
                or (self.operator == self.OP_IN
                    and isinstance(self.value, HplRange)))

    @property
    def is_invertible(self):
        return self.operator in self._INV and self.value.is_reference

    def negation(self):
        return HplFieldCondition(
            self.field, self._NOT[self.operator], self.value)

    def inverted(self, alias):
        if self.operator not in self._INV or not self.value.is_reference:
            raise HplLogicError("impossible to invert: " + str(self))
        new_value = HplFieldReference(self.field.token, message=alias)
        new_field = HplFieldReference(self.value.token, message=None)
        new_op = self._INV[self.operator]
        return HplFieldCondition(new_field, new_op, new_value)

    # NOTE old, unused code that might be needed later
    def __normalise_quantifiers(self):
        #negate = False
        #expr = self.field.clone()
        #for field in expr.fields:
        #    if field.index == NO_INDEX:
        #        field.index = SOME_INDEX if negate else ALL_INDICES
        #        negate = not negate
        #    elif field.index == ALL_INDICES and negate:
        #        field.index = SOME_INDEX
        #    elif field.index == SOME_INDEX and negate:
        #        field.index = ALL_INDICES
        #op = self._NOT[self.operator] if negate else self.operator
        #return HplMsgFieldCondition(expr, op, self.value)
        pass

    def __eq__(self, other):
        if not isinstance(other, HplMsgFieldCondition):
            return False
        return (self.field == other.field
                and self.operator == other.operator
                and self.value == other.value)

    def __hash__(self):
        h = 31 * hash(self.field) + hash(self.operator)
        return 31 * h + hash(self.value)

    def __str__(self):
        return "{} {} {}".format(self.field, self.operator, self.value)

    def __repr__(self):
        return "{}({}, {}, {})".format(type(self).__name__,
            repr(self.field), repr(self.operator), repr(self.value))


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
        self.values = values # [HplLiteral | HplFieldReference]
        ros_types = set(ROS_PRIMITIVE_TYPES)
        for value in values:
            ros_types = ros_types & set(value.ros_types)
        if not ros_types:
            raise TypeError("mixed incompatible types: " + repr(values))
        self.ros_types = tuple(ros_types)

    @property
    def is_set(self):
        return True

    def to_set(self):
        return set(self.values)

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
        self.lower_bound = lower # HplLiteral | HplFieldReference
        self.upper_bound = upper # HplLiteral | HplFieldReference
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
        return "{}{} to {}{}".format(
            self.lower_bound, " (exc)" if self.exclude_lower else "",
            self.upper_bound, " (exc)" if self.exclude_upper else "")

    def __repr__(self):
        return "{}({}, {}, exc_lower={}, exc_upper={})".format(
            type(self).__name__, repr(self.lower_bound), repr(self.upper_bound),
            repr(self.exclude_lower), repr(self.exclude_upper))


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
                is_loop = index.startswith("*") or ":" in index
                parts.append(HplFieldAccessor(index, True, is_loop))
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

    @property
    def is_multi_field(self):
        return any(accessor.is_loop for accessor in self._parts)

    def loops(self):
        # ex: "a[*].b.c[*].d.e[0].f"
        # >> [(["a"], "*"), (["b", "c"], "*")], ["d.e[0].f"]
        arrays = [i for i in range(len(self._parts)) if self._parts[i].is_loop]
        previous = 0
        for i in range(len(arrays)):
            j = arrays[i]
            arrays[i] = (self._parts[previous:j], self._parts[j])
            previous = j + 1
        return arrays, self._parts[previous:]

    def __eq__(self, other):
        if not isinstance(other, HplFieldReference):
            return False
        return (self.token == other.token
                and self.message == other.message)

    def __hash__(self):
        return 31 * hash(self.token) + hash(self.message)

    def __str__(self):
        return "{}{}".format(
            "" if self.message is None else "${}.".format(self.message),
            self.token)

    def __repr__(self):
        return "{}({}, message={})".format(
            type(self).__name__, repr(self.token), repr(self.message))


HplFieldAccessor = namedtuple("HplFieldAccessor",
    ("key", "is_indexed", "is_loop"))
