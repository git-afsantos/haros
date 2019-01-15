
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
                if self.pub_msg_filter.accepts(msg):
                    {accept_msg_action}
                elif time_left < self.time_tolerance_threshold:
                    {reject_msg_action}
        return valid_messages == self.expected_messages
"""

INITIALIZE_TEMPLATE = """
    @initialize(msg={strategy})
    def gen_msg_{var}(self, msg):
        self.required_msgs["{var}"] = (msg, "{topic}")
        # self._user_{var} = (msg, "{topic}")
"""

PUBLISH_IRRELEVANT_TEMPLATE = """
    @rule(msg={strategy})
    def pub_{name}(self, msg):
        self.ros.pub["{topic}"].publish(msg)
"""

PUBLISH_RELEVANT_TEMPLATE = """
    @rule()
    def pub_{var}(self):
        self.ros.pub["{topic}"].publish(self._user_{var})
"""


################################################################################
# HPL Condition to Python Condition Compiler
################################################################################

class ConditionTransformer(object):
    def __init__(self):
        self.loops = []
        self.left_operand = None
        self.right_operands = []
        self._var = 0
        self._var_prefix = "self._user_"

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
        return "assert " + expr

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
        op = "==" if condition.operator == "=" else condition.operator
        if isinstance(condition.value, HplLiteral):
            value = condition.value
        else:
            assert isinstance(condition.value, HplFieldExpression2)
            value = self._field_expression(condition.value)
        self.right_operands.append((op, value))

    def _comparison_test(self, condition):
        if isinstance(condition.value, HplLiteral):
            value = condition.value
        else:
            assert isinstance(condition.value, HplFieldExpression2)
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
        assert isinstance(value, HplFieldExpression2), "found: " + repr(value)
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
        if not hpl_receive.msg_filter is None:
            msg_filter = hpl_receive.msg_filter.normalise()
            self._variable = hpl_receive.variable
            self._msg_strategy = self.strategy_map.make_custom(
                hpl_receive.variable, hpl_receive.msg_type)
            self._filter_to_strategies(msg_filter)
        # set msg as required
        # set default/custom strategy
        return self._msg_strategy.to_python()

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
# Test Code
################################################################################

if __name__ == "__main__":
    from .hpl_ast import (HplReceiveStatement, HplMsgFilter,
        HplMsgFieldCondition, HplFieldReference)
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

    sm = StrategyMap(TEST_DATA)

    fields = (
        HplFieldReference("nested_array[all]",
            TEST_DATA["pkg/Nested"]["nested_array"], "pkg/Nested"),
        HplFieldReference("int", TEST_DATA["pkg/Nested2"]["int"], "pkg/Nested2")
    )
    left = HplFieldExpression("nested_array[0].int", fields, "m", "pkg/Nested")

    fields = (
        HplFieldReference("int", TEST_DATA["pkg/Nested"]["int"], "pkg/Nested"),
    )
    right = HplFieldExpression("int", fields, "m", "pkg/Nested")

    cond1 = HplMsgFieldCondition(left, OPERATOR_EQ, right)
    msg_filter = HplMsgFilter((cond1,))

    hpl_receive = HplReceiveStatement("m", "/topic",
        msg_filter=msg_filter, msg_type="pkg/Nested")

    transformer = ReceiveToStrategyTransformer(sm)

    print str(hpl_receive)
    print ""
    print transformer.gen(hpl_receive)
