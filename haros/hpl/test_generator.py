
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
    ALL_INDICES, SOME_INDEX, NO_INDEX, OPERATOR_EQ, OPERATOR_NEQ, OPERATOR_LT,
    OPERATOR_LTE, OPERATOR_GT, OPERATOR_GTE, OPERATOR_IN, OPERATOR_NIN,
    HplLiteral, HplFieldExpression, HplSet, HplRange
)
from .hypothesis_strategies import (
    StrategyMap, FieldStrategy, StrategyReference
)


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
        self._deps = {}
        self._msg_strategy = None

    def gen(self, hpl_receive):
        assert not hpl_receive.variable is None
        assert not hpl_receive.msg_type is None
        if not hpl_receive.msg_filter is None:
            msg_filter = hpl_receive.msg_filter.normalise()
            self._msg_strategy = self.strategy_map.make_custom(
                hpl_receive.variable, hpl_receive.msg_type)
            group = self.strategy_map.custom[hpl_receive.variable]
            self._filter_to_strategies(msg_filter, group)
        # set msg as required
        # set default/custom strategy
        return ""

    def _filter_to_strategies(self, msg_filter, group):
        for condition in msg_filter.field_conditions:
            self._prepare_strategies(condition.field)
        for condition in msg_filter.field_conditions:
            field_expr = condition.field
            last_field = field_expr.fields[-1]
            value = self._process_value(field_expr, condition.value)
            value = condition.value
            # TODO detect dependencies between fields
            field_strategy = self._operator_strategy(last_field,
                condition.operator, value)
            msg_strategy = group[last_field.msg_type]
            msg_strategy.fields[last_field.name] = field_strategy
            # TODO propagate changes to parent custom strategies

    def _prepare_strategies(self, field_expr):
        key = field_expr.variable
        msg_strategy = self._msg_strategy
        for i in xrange(len(field_expr.fields) - 1):
            field = field_expr.fields[i]
            key += "." + field.name
            new_strategy = self.strategy_map.make_custom(key, field.ros_type)
            msg_strategy.fields[field.name] = FieldStrategy(field.name,
                strategy=StrategyReference(new_strategy.name))
            msg_strategy = new_strategy

    def _process_value(self, field_expr, value):
        if isinstance(value, HplLiteral):
            return repr(value.value)
        if isinstance(value, HplFieldExpression):
            assert field_expr.variable == value.variable
            left_key = field_expr.variable
            right_key = value.variable
            for i in xrange(len(field_expr.fields) - 1):
                msg_strategy = self.strategy_map.custom[left_key]
                left_field = field_expr.fields[i]
                right_field = value.fields[i]
                left_key += "." + left_field.name
                right_key += "." + right_field.name
                if left_field.name != right_field.name:
                    # msg_strategy.order[left_field.name] = right_field.name
                    # msg_strategy.inject_dependency(left_field.name, )
                    msg_strategy = self.strategy_map.custom[left_key]
                    # a[all].b.c != a[all].b.d
        if isinstance(value, HplSet):
        if isinstance(value, HplRange):
        raise TypeError("unexpected value type: " + type(value).__name__)

    def _operator_strategy(self, field, operator, value):
        assert field.index != NO_INDEX
        if operator == OPERATOR_EQ:
            return self._operator_eq(field, value)
        if operator == OPERATOR_NEQ:
            return self._operator_neq(field, value)
        if operator == OPERATOR_LT:
            return self._operator_lt(field, value)
        if operator == OPERATOR_LTE:
            return self._operator_lte(field, value)
        if operator == OPERATOR_GT:
            return self._operator_gt(field, value)
        if operator == OPERATOR_GTE:
            return self._operator_gte(field, value)
        if operator == OPERATOR_IN:
            return self._operator_in(field, value)
        if operator == OPERATOR_NIN:
            return self._operator_nin(field, value)
        raise ValueError("unknown operator: " + str(operator))

    def _operator_eq(self, field, value):
        if field.index is None:
            pass
        if field.index == ALL_INDICES:
            pass
        if field.index == SOME_INDEX:
            pass
        return field_strategy

    def _operator_neq(self, field, value):
        return field_strategy

    def _operator_lt(self, field, value):
        return field_strategy

    def _operator_lte(self, field, value):
        return field_strategy

    def _operator_gt(self, field, value):
        return field_strategy

    def _operator_gte(self, field, value):
        return field_strategy

    def _operator_in(self, field, value):
        return field_strategy

    def _operator_nin(self, field, value):
        return field_strategy

    def _reset(self):
        self.target_msg_type = None
        self.target_field = None
        self.target_type = None
        self.strategies = {}
        self.assumptions = []

    def _process(self, msg_filter):
        # "none" has to be converted into a proper negation
        msg_filter = msg_filter.normalise()
        for condition in msg_filter.field_conditions:
            pass
        
        self._process_left_field(condition.field)
        """
        if condition.is_equality_test:
            self._equality_test(condition)
        elif condition.is_comparison_test:
            self._comparison_test(condition)
        else:
            assert condition.is_inclusion_test
            self._inclusion_test(condition)
        """

    def _process_left_field(self, expr):
        # each message type only appears once, messages are not recursive
        # traverse from right to left to propagate custom strategies
        custom_strategies = []
        field = expr.fields[-1]
        msg_strat = self.strategies.make_custom(field.msg_type)
        field_strat = FieldStrategy.make_default("int", TypeToken("int32"))
        if not field.index is None:
            assert isinstance(field.field_type, ArrayFieldTypeToken)
            if field.is_loop:
                pass
            else:
                pass
        else:
            pass
        for i in xrange(len(expr.fields) - 1, -1, -1):
            field = expr.fields[i]
            msg_strat = self.strategies.make_custom(field.msg_type)
            self.strategies[field.msg_type] = self.MsgStrategy(
                field.msg_type, custom=True)
            if not field.index is None:
                assert isinstance(field.field_type, ArrayFieldTypeToken)
                if field.is_loop:
                    pass # self.Array(...)
                else:
                    pass # self.Array(...)
            else:
                pass

    def _equality_test(self, condition):
        if condition.operator == "=":
            op = "=="
            self._eq_operator = True
        else:
            op = condition.operator
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
                (condition.operator, self._set_literal(condition.value))
            )
        else:
            assert isinstance(condition.value, HplRange)
            lo, hi = self._range_literal(condition.value)
            self.right_operands.append(
                (">" if condition.value.exclude_lower else ">=", lo)
            )
            self.right_operands.append(
                ("<" if condition.value.exclude_upper else "<=", hi)
            )


################################################################################
# Test Code
################################################################################
