
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
from .hypothesis_strategies import StrategyMap, ArrayGenerator, Selector


################################################################################
# Templates
################################################################################

SPIN_TEMPLATE = """
    def spin(self, inbox):
        valid_messages = 0
        t0 = rospy.get_rostime()
        time_left = self.timeout
        while time_left >= 0.0:
            # time_left = self.timeout - (rospy.get_rostime() - t0)
            msgs, timed_out = inbox.wait_for_messages(timeout=time_left)
            if timed_out:
                {timeout_action}
            time_left = self.timeout - (rospy.get_rostime() - t0)
            for msg in msgs:
                # assume we are subscribing only expected topics
                topic = msg._connection_header["topic"]
                if self.pub_msg_filters[topic](msg):
                    {accept_msg_action}
                elif time_left < self.time_tolerance_threshold:
                    {reject_msg_action}
        return valid_messages == self.expected_messages
"""

INITIALIZE_TEMPLATE = """
    @initialize(msg={strategy}())
    def gen_{var}(self, msg):
        self._user_{var} = msg
"""

PUBLISH_IRRELEVANT_TEMPLATE = """
    @rule(msg={strategy}())
    def pub_{name}(self, msg):
        self.ros.pub["{topic}"].publish(msg)
"""

PUBLISH_RELEVANT_TEMPLATE = """
    @rule()
    def pub_{var}(self):
        self.ros.pub["{topic}"].publish(self._user_{var})
"""


PUBLISH_ALL_REQUIRED_TEMPLATE = """
    def publish_required_msgs(self):
{required_msgs}
"""

PUBLISH_REQUIRED_MSG_TEMPLATE = (
"""        self.ros.pub["{topic}"].publish(self._user_{var})
        reporting.report(u"state.v_pub_{var}({{}})".format(self._user_{var}))"""
)

TEARDOWN_TEMPLATE = """
    def teardown(self):
        if self.publish_required_msgs():
            assert self.spin(inbox)
"""

PUB_MSG_FILTER_TEMPLATE = """
    def _accepts_{topic}(self, msg):
        self._user_{var} = msg
        return {condition}
"""

STATE_MACHINE_TEMPLATE = """
class HarosPropertyTester(RuleBasedStateMachine):
    def __init__(self):
        RuleBasedStateMachine.__init__(self)
        {self_vars}
{init_defs}{rule_defs}{msg_filters}{spin}{pub_required}
    def teardown(self):
        self.publish_required_msgs()
        assert self.spin(inbox)
"""


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

    def _get_field_type(self, name, msg_type):
        # TODO
        for i in xrange(len(msg_type.__slots__)):
            if name == msg_type.__slots__[i]:
                field_type = msg_type._slot_types[i]
        else:
            constants = {key: type(value) for key, value in msg_type.__dict__.iteritems() if not key.startswith('__') and not callable(key)}
            if name in constants:
                field_type = constants[name]

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
            if ps.get_num_connections() < 1:
                return ps.resolved_name
        return None

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
    def __init__(self, timeout, tolerance):
        self.lock = Lock()
        self.inbox_flag = Event()
        self.timeout = timeout
        self.tolerance = tolerance
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
            elapsed = rospy.get_rostime() - self._start
            if elapsed > self.timeout:
                self._rejects += 1
            elif {condition}:
                self._accepts += 1
            elif elapsed >= self.tolerance:
                self._rejects += 1
            self.inbox_flag.set()"""

    __slots__ = ("_slots", "_inits", "_resets", "_yields",
                 "_pub_methods", "_callbacks")

    def __init__(self):
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

    def add_subscriber(self, topic, msg_type, var_name, condition):
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
        self._callbacks.append(self.CALLBACK.format(
            esc_topic=esc_topic, var=var_name, condition=condition))

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
        return self.INIT.format(inits=inits)

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
# HPL to Python Script Compiler
################################################################################

class HplTestGenerator(object):
    def __init__(self, msg_data):
        self.spin_vars = {}
        self.self_vars = {}
        self.pub_msg_filters = []
        self.init_defs = []
        self.rule_defs = []
        self.required_msgs = {}
        self.receive_transformer = ReceiveToStrategyTransformer(
            StrategyMap(msg_data))

    def gen(self, hpl_property):
        self.self_vars["pub_msg_filters"] = {}
        self._process_receive_stmt(hpl_property.receive)
        self._process_publish_stmt(hpl_property.publish)
        
        return STATE_MACHINE_TEMPLATE.format(
            self_vars=self._self_vars_to_python(),
            init_defs="\n".join(self.init_defs),
            rule_defs="\n".join(self.rule_defs),
            msg_filters="\n".join(self.pub_msg_filters),
            spin=SPIN_TEMPLATE.format(**self.spin_vars),
            pub_required=self._pub_required_msgs())

    def _process_receive_stmt(self, receive):
        if receive is None:
            return
        else:
            self.required_msgs[receive.variable] = receive.ros_name
            msg_strategy = self.receive_transformer.gen(receive)
            self.init_defs.append(INITIALIZE_TEMPLATE.format(
                strategy=msg_strategy.name, var=receive.variable))

    def _process_publish_stmt(self, publish):
        self._process_pub_multiplicity(publish.multiplicity)
        self._process_pub_time_bound(publish.time_bound)
        self._process_pub_msg_filter(publish.variable, publish.ros_name,
                                     publish.msg_filter)

    def _process_pub_multiplicity(self, multiplicity):
        if multiplicity is None:
            self.spin_vars["timeout_action"] = "return False"
            self.spin_vars["accept_msg_action"] = "return True"
            self.spin_vars["reject_msg_action"] = "pass"
            self.self_vars["expected_messages"] = 1
        else:
            self.self_vars["expected_messages"] = multiplicity.value
            if multiplicity.exact or multiplicity.exclusive:
                self.spin_vars["timeout_action"] = "break"
            else:
                self.spin_vars["timeout_action"] = "return False"
            if multiplicity.exact:
                if multiplicity.value == 0:
                    self.spin_vars["accept_msg_action"] = "return False"
                else:
                    self.spin_vars["accept_msg_action"] = "valid_messages += 1"
            else:
                if multiplicity.exclusive:
                    self.spin_vars["accept_msg_action"] = "pass"
                else:
                    self.spin_vars["accept_msg_action"] = "return True"
            if multiplicity.exclusive:
                self.spin_vars["reject_msg_action"] = "return False"
            else:
                self.spin_vars["reject_msg_action"] = "pass"

    def _process_pub_time_bound(self, time_bound):
        if time_bound is None:
            self.self_vars["timeout"] = 60.0
            self.self_vars["time_tolerance_threshold"] = 60.0
        else:
            s = time_bound.seconds
            if time_bound.is_frequency:
                self.self_vars["timeout"] = 2 * s
                self.self_vars["time_tolerance_threshold"] = s
            else:
                self.self_vars["timeout"] = s
                self.self_vars["time_tolerance_threshold"] = s

    def _process_pub_msg_filter(self, var_name, ros_name, msg_filter):
        # TODO edge case where var_name == "self"
        topic = ros_name.replace("/", "_")
        if msg_filter is None:
            condition = "True"
        else:
            transformer = ConditionTransformer(var_prefix="self._user_")
            condition = " and ".join(
                transformer.gen(c) for c in msg_filter.field_conditions)
        self.pub_msg_filters.append(PUB_MSG_FILTER_TEMPLATE.format(
            topic=topic, var=var_name, condition=condition))
        self.self_vars["pub_msg_filters"][ros_name] = ("self._accepts_" + topic)

    def _self_vars_to_python(self):
        tmp = "self.{} = {}"
        return "\n        ".join(tmp.format(key, value)
                                 for key, value in self.self_vars.iteritems())

    def _pub_required_msgs(self):
        body = "\n".join(
            PUBLISH_REQUIRED_MSG_TEMPLATE.format(topic=value, var=key)
            for key, value in self.required_msgs.iteritems())
        return PUBLISH_ALL_REQUIRED_TEMPLATE.format(required_msgs=body)


################################################################################
# Test Code
################################################################################

if __name__ == "__main__":
    from .hpl_ast import (HplReceiveStatement, HplMsgFilter, HplProperty,
        HplMsgFieldCondition, HplFieldReference, HplPublishStatement)
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

    hpl_publish = HplPublishStatement("out", "/topic2", time_bound=None,
        msg_filter=None, mult=None, msg_type="pkg/Nested")

    hpl_property = HplProperty(hpl_publish, receive_stmt=hpl_receive)

    test_gen = HplTestGenerator(TEST_DATA)
    print test_gen.gen(hpl_property)


    rosint = RosInterfaceGenerator()
    rosint.add_publisher("/events/bumper", "kobuki_msgs/BumperEvent", "bumper")
    rosint.add_anon_publisher("/events/wheel_drop", "kobuki_msgs/WheelDropEvent")
    rosint.add_subscriber("/cmd_vel", "geometry_msgs/Twist", "cmd", "True")
    print ""
    print rosint.gen()
