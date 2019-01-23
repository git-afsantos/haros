
#Copyright (c) 2018 Andre Santos
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


################################################################################
# Imports
################################################################################

from collections import namedtuple

from .hpl_ast import (
    ALL_INDICES,
    OPERATOR_EQ, OPERATOR_NEQ, OPERATOR_LT, OPERATOR_LTE,
    OPERATOR_GT, OPERATOR_GTE, OPERATOR_IN, OPERATOR_NIN,
    HplLiteral, HplFieldExpression, HplSet, HplRange
)
from .hypothesis_strategies import (
    StrategyMap, ArrayGenerator, Selector, ros_type_to_name
)


################################################################################
# HPL Condition to Python Condition Compiler
################################################################################

class ConditionTransformer(object):
    def __init__(self, var_prefix=""):
        self.loops = []
        self.left_operand = None
        self.right_operands = []
        self._var = 0
        self._var_prefix = var_prefix

    def gen(self, condition):
        self._reset()
        self._process(condition)
        expr = "{}"
        for loop in self.loops:
            assert isinstance(loop, self.Loop)
            expr = expr.format("{}({{}} for {} in {})".format(
                loop.operator, loop.alias, loop.name))
    # --------------------------------------------------------------------------
        assert isinstance(self.left_operand, self.Reference)
        right_types = (HplLiteral, self.Reference, tuple)
        for op, value in self.right_operands:
            assert isinstance(value, right_types), "found: " + repr(value)
    # --------------------------------------------------------------------------
        left = self._python(self.left_operand)
        inner = " and ".join(
            "{} {} {}".format(left, op, self._python(value))
            for op, value in self.right_operands)
        expr = expr.format(inner)
        return expr

    def _reset(self):
        self.loops = []
        self.left_operand = None
        self.right_operands = []
        self._var = 0

    def _process(self, condition):
        self.left_operand = self._field_expression(condition.field)
        if condition.is_equality_test:
            self._equality_test(condition)
        elif condition.is_comparison_test:
            self._comparison_test(condition)
        else:
            assert condition.is_inclusion_test
            self._inclusion_test(condition)

    def _equality_test(self, condition):
        op = "==" if condition.operator == OPERATOR_EQ else condition.operator
        if isinstance(condition.value, HplLiteral):
            value = condition.value
        else:
            assert isinstance(condition.value, HplFieldExpression)
            value = self._field_expression(condition.value)
        self.right_operands.append((op, value))

    def _comparison_test(self, condition):
        if isinstance(condition.value, HplLiteral):
            value = condition.value
        else:
            assert isinstance(condition.value, HplFieldExpression)
            value = self._field_expression(condition.value)
        self.right_operands.append((condition.operator, value))

    def _inclusion_test(self, condition):
        if isinstance(condition.value, HplSet):
            self.right_operands.append(
                (condition.operator, self._set_literal(condition.value)))
        else:
            assert isinstance(condition.value, HplRange)
            lo, hi = self._range_literal(condition.value)
            self.right_operands.append(
                (">" if condition.value.exclude_lower else ">=", lo))
            self.right_operands.append(
                ("<" if condition.value.exclude_upper else "<=", hi))

    def _field_expression(self, expr):
        root = self._var_prefix + expr.variable
        for fields in expr.get_loops():
            alias = self._new_var()
            loop = self.Loop(fields, root, alias)
            root = alias
            self.loops.append(loop)
        return self.Reference(expr, root)

    def _set_literal(self, hpl_enum):
        # hpl_enum.values :: [HplLiteral | HplFieldExpression]
        return tuple(value for value in hpl_enum.values)

    def _range_literal(self, hpl_range):
        # hpl_range.lower_bound :: HplLiteral | HplFieldExpression
        # hpl_range.upper_bound :: HplLiteral | HplFieldExpression
        # hpl_range.exclude_lower :: bool
        # hpl_range.exclude_upper :: bool
        return (hpl_range.lower_bound, hpl_range.upper_bound)

    def _new_var(self):
        var_name = "x" + str(self._var)
        self._var += 1
        return var_name

    def _python(self, value):
        if isinstance(value, self.Reference):
            # this must come before check for tuple
            name = value.expr.last_name
            return value.root + "." + name if name else value.root
        if isinstance(value, HplLiteral):
            return repr(value.value)
        if isinstance(value, tuple):
            return "({})".format(", ".join(self._python(v) for v in value))
        assert isinstance(value, HplFieldExpression), "found: " + repr(value)
        return self._var_prefix + value.full_name

    Reference = namedtuple("Reference", ["expr", "root"])

    class Loop(object):
        def __init__(self, fields, root, alias):
            self.fields = fields
            self.variable = root
            self.alias = alias

        @property
        def name(self):
            return self.variable + "." + ".".join(
                f.name if f.is_loop else f.token for f in self.fields)

        @property
        def operator(self):
            op = self.fields[-1].index
            assert not (op is None or isinstance(op, int))
            return op


################################################################################
# Condition to Strategy Compiler
################################################################################

class ReceiveToStrategyTransformer(object):
    def __init__(self, strategy_map):
        self.strategy_map = strategy_map
        self._msg_strategy = None
        self._variable = None

    def gen(self, hpl_receive):
        assert not hpl_receive.variable is None
        assert not hpl_receive.msg_type is None
        self._variable = hpl_receive.variable
        self._msg_strategy = None
        if hpl_receive.msg_filter is None:
            return self.strategy_map.defaults[hpl_receive.msg_type]
        msg_filter = hpl_receive.msg_filter.normalise()
        self._msg_strategy = self.strategy_map.make_custom(
            hpl_receive.variable, hpl_receive.msg_type)
        self._filter_to_strategies(msg_filter)
        return self._msg_strategy

    def _filter_to_strategies(self, msg_filter):
        for condition in msg_filter.field_conditions:
            field_gen = self._select(condition.field)
            value = condition.value
            operator = condition.operator
            if operator == OPERATOR_EQ:
                field_gen.eq(self._value(value))
            elif operator == OPERATOR_NEQ:
                field_gen.neq(self._value(value))
            elif operator == OPERATOR_LT:
                field_gen.lt(self._value(value))
            elif operator == OPERATOR_LTE:
                field_gen.lte(self._value(value))
            elif operator == OPERATOR_GT:
                field_gen.gt(self._value(value))
            elif operator == OPERATOR_GTE:
                field_gen.gte(self._value(value))
            elif operator == OPERATOR_IN:
                if isinstance(value, HplRange):
                    if value.exclude_lower:
                        field_gen.gt(self._value(value.lower_bound))
                    else:
                        field_gen.gte(self._value(value.lower_bound))
                    if value.exclude_upper:
                        field_gen.lt(self._value(value.upper_bound))
                    else:
                        field_gen.lte(self._value(value.upper_bound))
                else:
                    assert isinstance(value, HplSet)
                    values = map(self._value, value.values)
                    field_gen.in_set(values)
            elif operator == OPERATOR_NIN:
                assert isinstance(value, HplSet)
                values = map(self._value, value.values)
                field_gen.not_in(values)
            else:
                raise ValueError("unknown operator: " + str(operator))

    def _value(self, value):
        if isinstance(value, HplFieldExpression):
            if not value.value is None:
                return value.value
            return self._reference(value)
        return value

    def _select(self, field_expr):
        fields = []
        for field in field_expr.fields:
            if field.index is None:
                fields.append(field.name)
            else:
                i = Selector.ALL if field.index == ALL_INDICES else field.index
                fields.append((field.name, i))
        return self._msg_strategy.select(fields)

    def _reference(self, field_expr):
        assert field_expr.variable == self._variable
        root = self._msg_strategy.root
        ros_type = field_expr.field_type.ros_type
        fields = []
        for field in field_expr.fields:
            fields.append(field.name)
            if not field.index is None:
                if field.index == ALL_INDICES:
                    fields.append(Selector.ALL)
                else:
                    fields.append(field.index)
        return Selector(root, fields, ros_type)


################################################################################
# ROS Interface Generator
################################################################################

class RosInterfaceGenerator(object):
    TMP = """
class RosInterface(object):
    {slots}

    instance = None
{init}
{reset}
{generator}
{pub_methods}
{callbacks}

    def get_msg_counts(self):
        with self.lock:
            elapsed = rospy.get_rostime() - self._start
            timed_out = elapsed > self.timeout
            if not timed_out:
                timed_out = self.inbox_flag.wait(self.timeout - elapsed)
            return (self._accepts, self._rejects, timed_out)

    def shutdown(self):
        for ps in self.all_pubs_subs():
            ps.unregister()
        with self.lock:
            self.inbox_flag.clear()

    def check_status(self):
        for ps in self.all_pubs_subs():
            assert ps.get_num_connections() >= 1, ("Failed to find "
                                                   + ps.resolved_name)

    def wait_for_interfaces(self, timeout=10.0):
        now = rospy.get_rostime()
        to = rospy.Duration.from_sec(timeout)
        end = now + to
        rate = rospy.Rate(5)
        while now < end:
            failed = None
            for ps in self.all_pubs_subs():
                if ps.get_num_connections() == 0:
                    failed = ps.resolved_name
                    rate.sleep()
                    break
            else:
                break
            now = rospy.get_rostime()
        else:
            raise LookupError("Failed to connect to topic: " + str(failed))
"""

    SLOTS = ('__slots__ = ("lock", "inbox_flag", "timeout", "tolerance", '
             '"_accepts", "_rejects", "_start", {slots})')

    PUB = "self._p{esc_topic}"
    SUB = "self._s{esc_topic}"
    MSG = "self._msg_{var}"

    INIT = """
    def __init__(self):
        self.lock = Lock()
        self.inbox_flag = Event()
        self.timeout = {timeout}
        self.tolerance = {tolerance}
        self._accepts = 0
        self._rejects = 0
        self._start = rospy.get_rostime()
        {inits}"""

    INIT_PUB = ('{pub} = rospy.Publisher("{topic}", '
                "{msg_class}, queue_size=10)")

    INIT_SUB = ('{sub} = rospy.Subscriber("{topic}", '
                "{msg_class}, self._on{esc_topic})")

    INIT_VAR = "{var} = None"

    RESET = """
    def reset(self):
        with self.lock:
            self._start = rospy.get_rostime()
            self._accepts = 0
            self._rejects = 0
            {resets}"""

    YIELDS = """
    def all_pubs_subs(self):
        {yields}"""

    YIELD_PUB_SUB = "yield {pub_sub}"

    PUBLISH_M = """
    def pub_{var}(self, msg):
        with self.lock:
            self._msg_{var} = msg
        self._p{esc_topic}.publish(msg)"""

    PUBLISH = """
    def random_pub{esc_topic}(self, msg):
        self._p{esc_topic}.publish(msg)"""

    CALLBACK = """
    def _on{esc_topic}(self, msg):
        with self.lock:
            self._msg_{var} = msg
            if {deps}:
                return
            elapsed = rospy.get_rostime() - self._start
            if elapsed > self.timeout:
                self._rejects += 1
            elif {condition}:
                self._accepts += 1
            elif elapsed >= self.tolerance:
                self._rejects += 1
            self.inbox_flag.set()"""

    CB_DEP = "self._msg_{var} is None"

    DEFAULT_TIMEOUT = 60.0
    DEFAULT_TOLERANCE = 0.0

    __slots__ = ("timeout", "tolerance", "_slots", "_inits", "_resets",
                 "_yields", "_pub_methods", "_callbacks")

    def __init__(self):
        self.timeout = self.DEFAULT_TIMEOUT
        self.tolerance = self.DEFAULT_TOLERANCE
        self._slots = []
        self._inits = []
        self._resets = []
        self._yields = []
        self._pub_methods = []
        self._callbacks = []

    def add_anon_publisher(self, topic, msg_type):
        esc_topic = topic.replace("/", "_")
        msg_class = msg_type.replace("/", ".")
        pub = self.PUB.format(esc_topic=esc_topic)
        self._slots.append(pub[5:])
        self._inits.append(self.INIT_PUB.format(
            pub=pub, topic=topic, msg_class=msg_class))
        self._yields.append(self.YIELD_PUB_SUB.format(pub_sub=pub))
        self._pub_methods.append(self.PUBLISH.format(esc_topic=esc_topic))

    def add_publisher(self, topic, msg_type, var_name):
        esc_topic = topic.replace("/", "_")
        msg_class = msg_type.replace("/", ".")
        pub = self.PUB.format(esc_topic=esc_topic)
        msg = self.MSG.format(var=var_name)
        self._slots.append(pub[5:])
        self._slots.append(msg[5:])
        self._inits.append(self.INIT_PUB.format(
            pub=pub, topic=topic, msg_class=msg_class))
        init = self.INIT_VAR.format(var=msg)
        self._inits.append(init)
        self._resets.append(init)
        self._pub_methods.append(self.PUBLISH_M.format(
            var=var_name, esc_topic=esc_topic))
        self._yields.append(self.YIELD_PUB_SUB.format(pub_sub=pub))

    def add_subscriber(self, topic, msg_type, var_name, condition, deps):
        esc_topic = topic.replace("/", "_")
        msg_class = msg_type.replace("/", ".")
        sub = self.SUB.format(esc_topic=esc_topic)
        msg = self.MSG.format(var=var_name)
        self._slots.append(sub[5:])
        self._slots.append(msg[5:])
        self._inits.append(self.INIT_SUB.format(
            sub=sub, topic=topic, msg_class=msg_class, esc_topic=esc_topic))
        init = self.INIT_VAR.format(var=msg)
        self._inits.append(init)
        self._resets.append(init)
        self._yields.append(self.YIELD_PUB_SUB.format(pub_sub=sub))
        if not deps:
            cb_deps = "False"
        else:
            cb_deps = " or ".join(self.CB_DEP.format(var=v) for v in deps)
        self._callbacks.append(self.CALLBACK.format(esc_topic=esc_topic,
            var=var_name, condition=condition, deps=cb_deps))

    def gen(self):
        return self.TMP.format(
            slots=self._gen_slots(),
            init=self._gen_init(),
            reset=self._gen_reset(),
            generator=self._gen_generator(),
            pub_methods=self._gen_pub_methods(),
            callbacks=self._gen_callbacks())

    def _gen_slots(self):
        slots = ", ".join('"' + s + '"' for s in self._slots)
        return self.SLOTS.format(slots=slots)

    def _gen_init(self):
        inits = "\n        ".join(self._inits)
        return self.INIT.format(
            timeout=self.timeout, tolerance=self.tolerance, inits=inits)

    def _gen_reset(self):
        resets = "\n            ".join(self._resets)
        return self.RESET.format(resets=resets)

    def _gen_generator(self):
        yields = "\n        ".join(self._yields)
        return self.YIELDS.format(yields=yields)

    def _gen_pub_methods(self):
        return "\n".join(self._pub_methods)

    def _gen_callbacks(self):
        return "\n".join(self._callbacks)


################################################################################
# Infrastructure Manager Generator
################################################################################

class InfrastructureGenerator(object):
    TMP = """
class SystemUnderTest(object):
    NODES = ({nodes})
    LAUNCH = ({launches})

    __slots__ = ("launches",)

    def __init__(self):
        self.launches = []
        for launch_path in self.LAUNCH:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path],
                        is_core=False, verbose=False)
            launch.start(auto_terminate=False)
            self.launches.append(launch)

    def shutdown(self):
        for launch in self.launches:
            launch.shutdown()
        self.wait_for_nodes(online=False)

    def check_status(self):
        for node_name in self.NODES:
            assert rosnode_ping(node_name, max_count=1), ("Failed to find "
                                                          + node_name)

    def wait_for_nodes(self, timeout=60.0, online=True):
        now = rospy.get_rostime()
        to = rospy.Duration.from_sec(timeout)
        end = now + to
        rate = rospy.Rate(5)
        pending = list(self.NODES)
        while now < end and pending:
            node_name = pending.pop()
            if not rosnode_ping(node_name, max_count=1) is online:
                pending.append(node_name)
                rate.sleep()
            now = rospy.get_rostime()
        if pending and online:
            raise LookupError("Failed to find nodes " + str(pending))
"""

    __slots__ = ("_nodes", "_launch")

    def __init__(self):
        self._nodes = []
        self._launch = []

    def add_launch(self, launch_path):
        self._launch.append(launch_path)

    def add_node(self, node_name):
        self._nodes.append(node_name)

    def gen(self):
        if not self._nodes:
            raise RuntimeError("must register nodes under test")
        if not self._launch:
            raise RuntimeError("must register launch files")
        nodes = ", ".join('"' + name + '"' for name in self._nodes)
        if len(self._nodes) == 1:
            nodes += ","
        launches = ", ".join('"' + path + '"' for path in self._launch)
        if len(self._launch) == 1:
            launches += ","
        return self.TMP.format(nodes=nodes, launches=launches)


################################################################################
# Hypothesis State Machine Generator
################################################################################

class StateMachineGenerator(object):
    TMP = """
class HarosPropertyTester(RuleBasedStateMachine):
    _time_spent_on_testing = 0.0
    _time_spent_setting_up = 0.0
{init}
{init_defs}
{rule_defs}
{spin}
{pub_required}

    def teardown(self):
        try:
            self.publish_required_msgs()
            self.sut.check_status()
            self.ros.check_status()
            self.spin()
        finally:
            t0 = rospy.get_rostime()
            t = t0 - self._start_time
            HarosPropertyTester._time_spent_on_testing += t.to_sec()
            self.sut.shutdown()
            self.ros.shutdown()
            t = rospy.get_rostime() - t0
            HarosPropertyTester._time_spent_setting_up += t.to_sec()

    @classmethod
    def print_times(cls):
        print "Time spent on testing (s):", cls._time_spent_on_testing
        print "Time spent setting up (s):", cls._time_spent_setting_up

PropertyTest = HarosPropertyTester.TestCase
"""

    INIT = """
    def __init__(self):
        RuleBasedStateMachine.__init__(self)
        t = rospy.get_rostime()
        {msgs}
        self.ros = RosInterface()
        self.sut = SystemUnderTest()
        self.sut.wait_for_nodes()
        self.ros.wait_for_interfaces()
        self._start_time = rospy.get_rostime()
        t = self._start_time - t
        HarosPropertyTester._time_spent_setting_up += t.to_sec()"""

    INIT_MSG = "self._msg_{var} = None"

    INITIALIZE = """
    @initialize({kwargs})
    def gen_msgs(self, {args}):
        {assigns}"""

    KWARG = "{arg}={strategy}()"

    MSG_ASSIGN = "self._msg_{var} = {arg}"

    PUB_RANDOM = """
    @rule(msg={strategy}())
    def pub_{esc_topic}(self, msg):
        self.ros.random_pub{esc_topic}(msg)"""

    SPIN = """
    def spin(self):
        reporting.report(u"state.spin()")
        valid_msgs = 0
        invalid_msgs = 0
        timed_out = False
        while not timed_out:
            accepts, rejects, timed_out = self.ros.get_msg_counts()
            if timed_out and accepts == 0 and rejects == 0:
                {timeout_action}
            for i in xrange(accepts - valid_msgs):
                {accept_msg_action}
            for i in xrange(rejects - invalid_msgs):
                {reject_msg_action}
            invalid_msgs = rejects
        assert valid_msgs == {expected_messages}"""

    PUB_REQUIRED = """
    def publish_required_msgs(self):
{required_msgs}"""

    PUBS = """        self.ros.pub_{var}(self._msg_{var})
        reporting.report(u"state.v_pub_{var}({{}})".format(self._msg_{var}))"""

    __slots__ = ("_msgs", "_init_kwargs", "_init_args", "_assigns", "_pubs",
                 "_required", "_spin")

    def __init__(self):
        self._msgs = []
        self._init_kwargs = []
        self._init_args = []
        self._assigns = []
        self._pubs = []
        self._required = []
        self._spin = self.SPIN.format(timeout_action="assert False",
            accept_msg_action="return", reject_msg_action="pass",
            expected_messages=1)

    def add_anon_publisher(self, topic, msg_type):
        esc_topic = topic.replace("/", "_")
        strategy = ros_type_to_name(msg_type)
        self._pubs.append(self.PUB_RANDOM.format(
            strategy=strategy, esc_topic=esc_topic))

    def add_publisher(self, var_name, strategy):
        self._msgs.append(self.INIT_MSG.format(var=var_name))
        msg = "msg" + str(len(self._msgs))
        self._init_kwargs.append(self.KWARG.format(arg=msg, strategy=strategy))
        self._init_args.append(msg)
        self._assigns.append(self.MSG_ASSIGN.format(var=var_name, arg=msg))
        self._required.append(self.PUBS.format(var=var_name))

    def set_expected(self, n, exact, exclusive):
        timeout_action = "break" if exact or exclusive else "assert False"
        if exact:
            accept = "assert False" if n == 0 else "valid_msgs += 1"
        else:
            accept = "pass" if exclusive else "return"
        reject = "assert False" if exclusive else "pass"
        self._spin = self.SPIN.format(timeout_action=timeout_action,
            accept_msg_action=accept, reject_msg_action=reject,
            expected_messages=n)

    def gen(self):
        return self.TMP.format(
            init=self._gen_init(),
            init_defs=self._gen_initialize(),
            rule_defs=self._gen_rules(),
            spin=self._spin,
            pub_required=self._gen_pub_required())

    def _gen_init(self):
        msgs = "\n        ".join(self._msgs)
        return self.INIT.format(msgs=msgs)

    def _gen_initialize(self):
        if not self._msgs:
            assert not self._init_kwargs
            assert not self._init_args
            assert not self._assigns
            return ""
        kwargs = ", ".join(self._init_kwargs)
        args = ", ".join(self._init_args)
        assigns = "\n        ".join(self._assigns)
        return self.INITIALIZE.format(kwargs=kwargs, args=args, assigns=assigns)

    def _gen_rules(self):
        return "\n\n".join(self._pubs)

    def _gen_pub_required(self):
        required_msgs = "\n".join(self._required)
        return self.PUB_REQUIRED.format(required_msgs=required_msgs)


################################################################################
# HPL to Python Script Compiler
################################################################################

class HplTestGenerator(object):
    TMP = """#!/usr/bin/env python

################################################################################
# Imports
################################################################################

import os
import sys
from threading import Event, Lock
import unittest

import hypothesis
import hypothesis.reporting as reporting
from hypothesis.stateful import (
    RuleBasedStateMachine, rule, initialize
)
import hypothesis.strategies as strategies
import rospy
import roslaunch
from rosnode import rosnode_ping

{imports}


################################################################################
# Data Strategies
################################################################################

{strategies}


################################################################################
# Utility
################################################################################

class HiddenPrints(object):
    def __enter__(self):
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        self._devnull = open(os.devnull, "w")
        sys.stdout = self._devnull
        sys.stderr = self._devnull

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._original_stdout
        sys.stderr = self._original_stderr
        self._devnull.close()


class OutputCollector(object):
    def __init__(self):
        self.output = []

    def report(self, value):
        self.output.append(value)

    def print_output(self):
        if not self.output:
            return
        print "{hpl_property}"
        print ("==================================="
               "===================================")
        print "Falsifying example"
        print ("-----------------------------------"
               "-----------------------------------")
        for line in self.output:
            print line
        print ("-----------------------------------"
               "-----------------------------------")


################################################################################
# ROS Interface
################################################################################
{ros}

################################################################################
# System Under Test
################################################################################
{sut}

################################################################################
# Hypothesis State Machine
################################################################################
{state_machine}

################################################################################
# Entry Point
################################################################################

def main():
    rospy.init_node("property_tester")
    HarosPropertyTester.settings = hypothesis.settings(
        max_examples=1000, stateful_step_count=100, buffer_size=16384,
        timeout=hypothesis.unlimited)
    collector = OutputCollector()
    reporting.reporter.value = collector.report
    with HiddenPrints():
        unittest.main(module=__name__, argv=[__name__], exit=False)
    collector.print_output()
    HarosPropertyTester.print_times()

if __name__ == "__main__":
    main()
"""

    def __init__(self, launch_files, nodes, publishers, subscribers, msg_data):
        # launch_files :: [str(path)]
        # nodes :: [str(name)]
        # publishers :: {str(topic): str(msg_type)}
        # subscribers :: {str(topic): str(msg_type)}
        # msg_data :: {str(msg_type): {str(field_name): TypeToken}}
        self.sut = InfrastructureGenerator()
        for launch_path in launch_files:
            self.sut.add_launch(launch_path)
        for node_name in nodes:
            self.sut.add_node(node_name)
        self.pubs = publishers
        self.subs = subscribers
        self.msg_data = msg_data

    def gen(self, hpl_property):
        strategies = StrategyMap(self.msg_data)
        ros = RosInterfaceGenerator()
        state_machine = StateMachineGenerator()
        self._process_receive(hpl_property.receive,
                              ros, state_machine, strategies)
        self._process_publish(hpl_property.publish, ros, state_machine)
        return self.TMP.format(
            imports=strategies.get_imports(),
            strategies=self._gen_strategies(strategies),
            hpl_property=str(hpl_property),
            ros=ros.gen(),
            sut=self.sut.gen(),
            state_machine=state_machine.gen())

    def _process_receive(self, hpl_receive, ros, state_machine, strategies):
        if hpl_receive is None:
            return
        topic = hpl_receive.ros_name
        if not topic in self.subs:
            raise ValueError("unknown topic: " + topic)
        transformer = ReceiveToStrategyTransformer(strategies)
        strategy = transformer.gen(hpl_receive)
        ros.add_publisher(topic, hpl_receive.msg_type, hpl_receive.variable)
        state_machine.add_publisher(hpl_receive.variable, strategy.name)
        for ros_name, msg_type in self.subs.iteritems():
            if ros_name != topic:
                ros.add_anon_publisher(ros_name, msg_type)
                state_machine.add_anon_publisher(ros_name, msg_type)

    def _process_publish(self, hpl_publish, ros, state_machine):
        mult = hpl_publish.multiplicity
        if not mult is None:
            state_machine.set_expected(mult.value, mult.exact, mult.exclusive)
        time_bound = hpl_publish.time_bound
        if not time_bound is None:
            # ros.tolerance = 0.0
            s = time_bound.seconds
            ros.timeout = 2 * s if time_bound.is_frequency else s
        topic = hpl_publish.ros_name
        if not topic in self.pubs:
            raise ValueError("unknown topic: " + topic)
        msg_filter = hpl_publish.msg_filter
        if msg_filter is None:
            condition = "True"
        else:
            transformer = ConditionTransformer(var_prefix="self._msg_")
            condition = " and ".join(
                transformer.gen(c) for c in msg_filter.field_conditions)
        ros.add_subscriber(topic, hpl_publish.msg_type, hpl_publish.variable,
                           condition, hpl_publish.references())

    def _gen_strategies(self, strategies):
        code = [s.to_python() for s in strategies.defaults.itervalues()]
        code.extend(s.to_python() for s in strategies.custom.itervalues())
        return "\n\n".join(code)


################################################################################
# Test Code
################################################################################

if __name__ == "__main__":
    from .hpl_ast import (HplReceiveStatement, HplMsgFilter, HplProperty,
        HplMsgFieldCondition, HplFieldReference, HplPublishStatement,
        HplTimeBound, HplLiteral)
    from .ros_types import (TypeToken, ArrayTypeToken)

    TEST_DATA = {
        "geometry_msgs/Twist": {
            "linear": TypeToken("geometry_msgs/Vector3"),
            "angular": TypeToken("geometry_msgs/Vector3")
        },
        "geometry_msgs/Vector3": {
            "x": TypeToken("float64"),
            "y": TypeToken("float64"),
            "z": TypeToken("float64")
        },
        "kobuki_msgs/BumperEvent": {
            "bumper": TypeToken("uint8"),
            "state": TypeToken("uint8")
        },
        "pkg/Msg": {
            "int": TypeToken("int32"),
            "float": TypeToken("float64"),
            "string": TypeToken("string"),
            "twist": TypeToken("geometry_msgs/Twist"),
            "int_list": ArrayTypeToken("int32"),
            "int_array": ArrayTypeToken("int32", length=3),
            "float_list": ArrayTypeToken("float64"),
            "float_array": ArrayTypeToken("float64", length=3),
            "string_list": ArrayTypeToken("string"),
            "string_array": ArrayTypeToken("string", length=3),
            "twist_list": ArrayTypeToken("geometry_msgs/Twist"),
            "twist_array": ArrayTypeToken("geometry_msgs/Twist", length=3),
            "nested_array": ArrayTypeToken("pkg/Nested", length=3)
        },
        "pkg/Nested": {
            "int": TypeToken("int32"),
            "int_array": ArrayTypeToken("int32", length=3),
            "nested_array": ArrayTypeToken("pkg/Nested2", length=3)
        },
        "pkg/Nested2": {
            "int": TypeToken("int32"),
            "int_array": ArrayTypeToken("int32", length=3)
        }
    }

    fields = (
        HplFieldReference("nested_array[all]",
            TEST_DATA["pkg/Nested"]["nested_array"], "pkg/Nested"),
        HplFieldReference("int", TEST_DATA["pkg/Nested2"]["int"], "pkg/Nested2")
    )
    left = HplFieldExpression("nested_array[all].int", fields, "m", "pkg/Nested")

    fields = (
        HplFieldReference("int", TEST_DATA["pkg/Nested"]["int"], "pkg/Nested"),
    )
    right = HplFieldExpression("int", fields, "m", "pkg/Nested")

    cond1 = HplMsgFieldCondition(left, OPERATOR_EQ, right)
    msg_filter = HplMsgFilter((cond1,))

    hpl_receive = HplReceiveStatement("m", "/topic",
        msg_filter=msg_filter, msg_type="pkg/Nested")

    hpl_publish = HplPublishStatement("out", "/topic2",
        time_bound=HplTimeBound("<", HplLiteral("100", int, ()), "ms"),
        msg_filter=None, mult=None, msg_type="pkg/Nested")

    hpl_property = HplProperty(hpl_publish, receive_stmt=hpl_receive)

    publishers = {"/topic2": "pkg/Nested"}
    subscribers = {"/topic": "pkg/Nested"}
    test_gen = HplTestGenerator(
        ["pkg/launch/minimal.launch"],
        ["/node1", "/node2"],
        publishers, subscribers, TEST_DATA)
    print test_gen.gen(hpl_property)
