
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
        assert isinstance(value, HplLiteral)
        return value.value

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
{init}
{generator}
{pub_methods}
{callbacks}
{spin}

    def shutdown(self):
        for ps in self.all_pubs_subs():
            ps.unregister()

    def check_status(self):
        for ps in self.all_pubs_subs():
            assert ps.get_num_connections() >= 1, ("Failed to find "
                                                   + ps.resolved_name)

    def wait_for_interfaces(self, timeout=10.0):
        rospy.logdebug("Waiting for publishers and subscribers")
        now = rospy.get_time()
        rate = rospy.Rate(5)
        while now == 0.0:
            rate.sleep()
            now = rospy.get_time()
        end = now + timeout
        while now < end:
            failed = None
            for ps in self.all_pubs_subs():
                if ps.get_num_connections() == 0:
                    failed = ps.resolved_name
                    rate.sleep()
                    break
            else:
                break
            now = rospy.get_time()
        else:
            raise LookupError("Failed to connect to topic: " + str(failed))
"""

    SLOTS = ('__slots__ = ("msg_feed", "timeout", "tolerance", '
             '"_req_remaining", "_accepts", "_rejects", "_start", {slots})')

    PUB = "self._p{esc_topic}"
    SUB = "self._s{esc_topic}"
    MSG = "self._msg_{var}"

    INIT = """
    def __init__(self):
        self.msg_feed = Condition()
        self.timeout = {timeout}
        self.tolerance = {tolerance}
        self._accepts = 0
        self._rejects = 0
        self._start = 0.0
        self._req_remaining = {req_count}
        {inits}"""

    INIT_PUB = ('{pub} = rospy.Publisher("{topic}", '
                "{msg_class}, queue_size=10)")

    INIT_SUB = ('{sub} = rospy.Subscriber("{topic}", '
                "{msg_class}, self._on{esc_topic})")

    INIT_VAR = "{var} = None"

    YIELDS = """
    def all_pubs_subs(self):
        {yields}"""

    YIELD_PUB_SUB = "yield {pub_sub}"

    PUBLISH_M = """
    def pub_{var}(self, msg):
        rospy.logdebug('Publish user-defined variable "{var}"')
        with self.msg_feed:
            assert self._msg_{var} is None
            self._msg_{var} = msg
            self._p{esc_topic}.publish(msg)
            self._req_remaining -= 1
            if self._req_remaining == 0:
                self._start = rospy.get_time()"""

    PUBLISH = """
    def random_pub{esc_topic}(self, msg):
        rospy.logdebug('Publish random message on "{topic}"')
        self._p{esc_topic}.publish(msg)"""

    CALLBACK = """
    def _on{esc_topic}(self, msg):
        with self.msg_feed:
            if not self._req_remaining == 0:
                rospy.logdebug("Received a message: Discarded (too early)")
                return # ignore msgs while not ready
            self._msg_{var} = msg
            if {deps}:
                rospy.logdebug("Received a message: Discarded (too early)")
                return
            elapsed = rospy.get_time() - self._start
            if elapsed > self.timeout:
                self._rejects += 1
                rospy.logdebug("Received a message: Rejected (too late)")
                self.msg_feed.notify()
            elif {condition}:
                self._accepts += 1
                rospy.logdebug("Received a message: Accepted")
                self.msg_feed.notify()
            elif elapsed >= self.tolerance:
                self._rejects += 1
                rospy.logdebug("Received a message: Rejected (invalid)")
                self.msg_feed.notify()"""

    CB_DEP = "self._msg_{var} is None"

    SPIN_SOME = """
    def spin(self):
        rospy.logdebug("Spinning")
        with self.msg_feed:
            if self._accepts > 0:
                return True
            elapsed = 0.0
            while elapsed < self.timeout:
                self.msg_feed.wait(self.timeout - elapsed)
                if self._accepts > 0:
                    return True
                elapsed = rospy.get_time() - self._start
            r = self._rejects
        assert r == 0, "the given property is False"
        raise TestTimeoutError("timed out waiting for expected messages")"""

    SPIN_NONE = """
    def spin(self):
        rospy.logdebug("Spinning")
        with self.msg_feed:
            if self._accepts > 0:
                raise InvalidMessageError("received an unexpected message")
            elapsed = 0.0
            while elapsed < self.timeout:
                self.msg_feed.wait(self.timeout - elapsed)
                if self._accepts > 0:
                    raise InvalidMessageError("received an unexpected message")
                elapsed = rospy.get_time() - self._start
        return True"""

    SPIN_EXACTLY_N = """
    def spin(self):
        rospy.logdebug("Spinning")
        with self.msg_feed:
            if self._accepts > {n}:
                raise InvalidMessageError("received too many messages")
            elapsed = 0.0
            while elapsed < self.timeout:
                self.msg_feed.wait(self.timeout - elapsed)
                if self._accepts > {n}:
                    raise InvalidMessageError("received too many messages")
                elapsed = rospy.get_time() - self._start
            if self._accepts < {n}:
                raise TestTimeoutError("timed out waiting for expected messages")
        return True"""

    SPIN_JUST = """
    def spin(self):
        rospy.logdebug("Spinning")
        with self.msg_feed:
            if self._rejects > 0:
                raise InvalidMessageError("received an unexpected message")
            elapsed = 0.0
            while elapsed < self.timeout:
                self.msg_feed.wait(self.timeout - elapsed)
                if self._rejects > 0:
                    raise InvalidMessageError("received an unexpected message")
                elapsed = rospy.get_time() - self._start
        return True"""

    # TODO
    SPIN_JUST_SOME = """
    def spin(self):
        rospy.logdebug("Spinning")
        with self.msg_feed:
            if self._rejects > 0:
                raise InvalidMessageError("received an unexpected message")
            elapsed = 0.0
            while elapsed < self.timeout:
                self.msg_feed.wait(self.timeout - elapsed)
                if self._rejects > 0:
                    raise InvalidMessageError("received an unexpected message")
                elapsed = rospy.get_time() - self._start
            if self._accepts == 0:
                raise TestTimeoutError("timed out waiting for expected messages")
        return True"""

    SPIN_JUST_N = """
    def spin(self):
        rospy.logdebug("Spinning")
        with self.msg_feed:
            if self._accepts > {n}:
                raise InvalidMessageError("received too many messages")
            if self._rejects > 0:
                raise InvalidMessageError("received an unexpected message")
            elapsed = 0.0
            while elapsed < self.timeout:
                self.msg_feed.wait(self.timeout - elapsed)
                if self._accepts > {n}:
                    raise InvalidMessageError("received too many messages")
                if self._rejects > 0:
                    raise InvalidMessageError("received an unexpected message")
                elapsed = rospy.get_time() - self._start
            if self._accepts < {n}:
                raise TestTimeoutError("timed out waiting for expected messages")
        return True"""

    DEFAULT_TIMEOUT = 60.0
    DEFAULT_TOLERANCE = 0.0

    __slots__ = ("timeout", "tolerance", "_slots", "_inits", "_spin",
                 "_yields", "_pub_methods", "_callbacks", "_req_pubs")

    def __init__(self):
        self.timeout = self.DEFAULT_TIMEOUT
        self.tolerance = self.DEFAULT_TOLERANCE
        self._slots = []
        self._inits = []
        self._yields = []
        self._pub_methods = []
        self._callbacks = []
        self._spin = self.SPIN_SOME
        self._req_pubs = 0

    def add_anon_publisher(self, topic, msg_type):
        esc_topic = topic.replace("/", "_")
        msg_class = msg_type.replace("/", ".")
        pub = self.PUB.format(esc_topic=esc_topic)
        self._slots.append(pub[5:])
        self._inits.append(self.INIT_PUB.format(
            pub=pub, topic=topic, msg_class=msg_class))
        self._yields.append(self.YIELD_PUB_SUB.format(pub_sub=pub))
        self._pub_methods.append(self.PUBLISH.format(
            esc_topic=esc_topic, topic=topic))

    def add_publisher(self, topic, msg_type, var_name):
        esc_topic = topic.replace("/", "_")
        msg_class = msg_type.replace("/", ".")
        pub = self.PUB.format(esc_topic=esc_topic)
        msg = self.MSG.format(var=var_name)
        self._slots.append(pub[5:])
        self._slots.append(msg[5:])
        self._inits.append(self.INIT_PUB.format(
            pub=pub, topic=topic, msg_class=msg_class))
        self._inits.append(self.INIT_VAR.format(var=msg))
        self._pub_methods.append(self.PUBLISH_M.format(
            var=var_name, esc_topic=esc_topic))
        self._yields.append(self.YIELD_PUB_SUB.format(pub_sub=pub))
        self._req_pubs += 1

    def add_subscriber(self, topic, msg_type, var_name, condition, deps):
        esc_topic = topic.replace("/", "_")
        msg_class = msg_type.replace("/", ".")
        sub = self.SUB.format(esc_topic=esc_topic)
        msg = self.MSG.format(var=var_name)
        self._slots.append(sub[5:])
        self._slots.append(msg[5:])
        self._inits.append(self.INIT_SUB.format(
            sub=sub, topic=topic, msg_class=msg_class, esc_topic=esc_topic))
        self._inits.append(self.INIT_VAR.format(var=msg))
        self._yields.append(self.YIELD_PUB_SUB.format(pub_sub=sub))
        if not deps:
            cb_deps = "False"
        else:
            cb_deps = " or ".join(self.CB_DEP.format(var=v) for v in deps)
        self._callbacks.append(self.CALLBACK.format(esc_topic=esc_topic,
            var=var_name, condition=condition, deps=cb_deps))

    def set_expected(self, n, exact, exclusive):
        if exclusive:
            if exact:
                self._spin = self.SPIN_JUST_N.format(n=n)
            else:
                # self._spin = self.SPIN_JUST_SOME
                self._spin = self.SPIN_JUST
        elif exact:
            if n == 0:
                self._spin = self.SPIN_NONE
            else:
                self._spin = self.SPIN_EXACTLY_N.format(n=n)
        else:
            self._spin = self.SPIN_SOME

    def gen(self):
        return self.TMP.format(
            slots=self._gen_slots(),
            init=self._gen_init(),
            generator=self._gen_generator(),
            pub_methods=self._gen_pub_methods(),
            callbacks=self._gen_callbacks(),
            spin=self._spin)

    def _gen_slots(self):
        slots = ", ".join('"' + s + '"' for s in self._slots)
        return self.SLOTS.format(slots=slots)

    def _gen_init(self):
        inits = "\n        ".join(self._inits)
        return self.INIT.format(timeout=self.timeout, tolerance=self.tolerance,
                                inits=inits, req_count=self._req_pubs)

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
        rospy.logdebug("Waiting for the SUT to start")
        now = rospy.get_time()
        end = now + timeout
        rate = rospy.Rate(5)
        pending = list(self.NODES)
        while now < end and pending:
            node_name = pending.pop()
            if not rosnode_ping(node_name, max_count=1) is online:
                pending.append(node_name)
                rate.sleep()
            now = rospy.get_time()
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
    _test_number = 1
{init}
{init_defs}
{rule_defs}
{pub_required}

    @rule(ms=random_wait_time())
    def sleep(self, ms):
        rospy.sleep(rospy.Duration(0, ms * 1000000))

    @invariant()
    def not_shutdown(self):
        assert not rospy.is_shutdown(), "rospy is shut down"

    def teardown(self):
        try:
            if self._has_required_msgs and self._initialized:
                self.publish_required_msgs()
                self.sut.check_status()
                self.ros.check_status()
                self.ros.spin()
            else:
                rospy.logdebug("Invalid trial")
        finally:
            t0 = rospy.get_time()
            t = t0 - self._start_time
            HarosPropertyTester._time_spent_on_testing += t
            rospy.logdebug("Shutting down the SUT")
            with HiddenPrints():
                self.sut.shutdown()
                self.ros.shutdown()
            t = rospy.get_time() - t0
            HarosPropertyTester._time_spent_setting_up += t
{print_end}

    def print_step(self, step):
        rule, data = step
        if rule.function.__name__ != "gen_msgs":
            RuleBasedStateMachine.print_step(self, step)

    @classmethod
    def print_times(cls):
        print "Time spent on testing (s):", cls._time_spent_on_testing
        print "Time spent setting up (s):", cls._time_spent_setting_up

PropertyTest = HarosPropertyTester.TestCase
"""

    INIT = """
    def __init__(self):
        RuleBasedStateMachine.__init__(self)
        rospy.loginfo("Running trial #" + str(self._test_number))
        HarosPropertyTester._test_number += 1
        t = rospy.get_time()
        self._initialized = False
        self._has_required_msgs = {has_required}
        {msgs}
        self.ros = RosInterface()
        with HiddenPrints():
            self.sut = SystemUnderTest()
            self.sut.wait_for_nodes()
            self.ros.wait_for_interfaces()
        self._start_time = rospy.get_time()
        t = self._start_time - t
        HarosPropertyTester._time_spent_setting_up += t"""

    INIT_MSG = "self._msg_{var} = None"

    INITIALIZE = """
    @initialize({kwargs})
    def gen_msgs(self, {args}):
        self._initialized = True
        {assigns}"""

    KWARG = "{arg}={strategy}()"

    MSG_ASSIGN = "self._msg_{var} = {arg}"

    PUB_RANDOM = """
    @rule(msg={strategy}())
    def pub_{esc_topic}(self, msg):
        self.ros.random_pub{esc_topic}(msg)"""

    PUB_REQUIRED = """
    def publish_required_msgs(self):
        {required_msgs}"""

    PUB_VAR = "self.ros.pub_{var}(self._msg_{var})"

    PRINTS = """
    def print_end(self):
        {req_pubs}
        reporting.report(u"state.spin()")
        RuleBasedStateMachine.print_end(self)"""

    PRINT_PUB = ('reporting.report(u"state.v_pub_{var}({{}})"'
                 '.format(self._msg_{var}))')

    __slots__ = ("_msgs", "_init_kwargs", "_init_args", "_assigns", "_pubs",
                 "_required", "_prints")

    def __init__(self):
        self._msgs = []
        self._init_kwargs = []
        self._init_args = []
        self._assigns = []
        self._pubs = []
        self._required = []
        self._prints = []

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
        self._required.append(self.PUB_VAR.format(var=var_name))
        self._prints.append(self.PRINT_PUB.format(var=var_name))

    def gen(self):
        return self.TMP.format(
            init=self._gen_init(),
            init_defs=self._gen_initialize(),
            rule_defs=self._gen_rules(),
            pub_required=self._gen_pub_required(),
            print_end=self._gen_prints())

    def _gen_init(self):
        msgs = "\n        ".join(self._msgs)
        has_required = len(self._required) > 0
        return self.INIT.format(msgs=msgs, has_required=has_required)

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
        if self._required:
            required_msgs = "\n        ".join(self._required)
        else:
            required_msgs = "pass"
        return self.PUB_REQUIRED.format(required_msgs=required_msgs)

    def _gen_prints(self):
        prints = "\n        ".join(self._prints)
        return self.PRINTS.format(req_pubs=prints)


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
from threading import Condition
import unittest

import hypothesis
from hypothesis import assume
import hypothesis.reporting as reporting
from hypothesis.stateful import (
    RuleBasedStateMachine, rule, initialize, invariant
)
import hypothesis.strategies as strategies
import rospy
import roslaunch
from rosnode import rosnode_ping

{imports}


################################################################################
# Data Strategies
################################################################################

def random_wait_time():
    # wait between 10ms and 1s
    return strategies.integers(min_value=10, max_value=1000)

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

class TestTimeoutError(Exception):
    pass

class InvalidMessageError(Exception):
    pass


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
    rospy.init_node("property_tester", log_level=rospy.{log_level})
    PropertyTest.settings = hypothesis.settings(
        max_examples=150, stateful_step_count=100, buffer_size=16384,
        deadline=None)
    collector = OutputCollector()
    reporting.reporter.value = collector.report
    unittest.main(module=__name__, argv=[__name__], exit=False)
    collector.print_output()
    HarosPropertyTester.print_times()

if __name__ == "__main__":
    main()
"""

    __slots__ = ("sut", "pubs", "subs", "msg_data", "debug")

    def __init__(self, launch_files, nodes, publishers, subscribers, msg_data,
                 debug=False):
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
        self.debug = debug

    def gen(self, hpl_property):
        strategies = StrategyMap(self.msg_data)
        ros = RosInterfaceGenerator()
        state_machine = StateMachineGenerator()
        self._process_receive(hpl_property.receive,
                              ros, state_machine, strategies)
        self._process_publish(hpl_property.publish, ros, state_machine)
        log_level = "DEBUG" if self.debug else "INFO"
        return self.TMP.format(
            imports=strategies.get_imports(),
            strategies=self._gen_strategies(strategies),
            hpl_property=str(hpl_property),
            ros=ros.gen(),
            sut=self.sut.gen(),
            state_machine=state_machine.gen(),
            log_level=log_level)

    def _process_receive(self, hpl_receive, ros, state_machine, strategies):
        topic = None
        if not hpl_receive is None:
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
            ros.set_expected(mult.value, mult.exact, mult.exclusive)
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
