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
# Constants
###############################################################################

TOK_VAL_TRUE = "True"
TOK_VAL_FALSE = "False"
TOK_VAL_PI = "PI"
TOK_VAL_INF = "INF"
TOK_VAL_NAN = "NAN"

TOK_VAR_REF = "@"

TOK_OP_NOT = "not"
TOK_OP_IMPLIES = "implies"
TOK_OP_IFF = "iff"
TOK_OP_AND = "and"
TOK_OP_OR = "or"
TOK_OP_IN = "in"
TOK_OP_FORALL = "forall"
TOK_OP_EXISTS = "exists"

TOK_UN_SECS = "s"
TOK_UN_MILLIS = "ms"

TOK_KW_ALIAS = "as"
TOK_KW_TIMEOUT = "within"
TOK_KW_NO = "no"
TOK_KW_SOME = "some"
TOK_KW_CAUSES = "causes"
TOK_KW_FORBIDS = "forbids"
TOK_KW_REQUIRES = "requires"
TOK_KW_GLOBAL = "globally"
TOK_KW_AFTER = "after"
TOK_KW_UNTIL = "until"


###############################################################################
# Token Grammar
###############################################################################

TOKEN_GRAMMAR = r"""
ros_name: ROS_NAME

int_literal: INT
string: ESCAPED_STRING
number: NUMBER
signed_number: SIGNED_NUMBER
boolean: TRUE | FALSE

TRUE: "{kw_true}"
FALSE: "{kw_false}"

RELATIONAL_OPERATOR: EQ_OPERATOR | COMP_OPERATOR | IN_OPERATOR
EQ_OPERATOR: "=" | "!="
COMP_OPERATOR: "<" "="?
             | ">" "="?
IN_OPERATOR.2: "{op_in}"

NOT_OPERATOR.3: "{op_not}"
IF_OPERATOR.3: "{op_implies}" | "{op_iff}"
OR_OPERATOR.3: "{op_or}"
AND_OPERATOR.3: "{op_and}"

QUANT_OPERATOR.4: ALL_OPERATOR | SOME_OPERATOR
ALL_OPERATOR: "{op_forall}"
SOME_OPERATOR: "{op_exists}"

CONSTANT.5: "{kw_pi}" | "{kw_inf}" | "{kw_nan}"
ADD_OPERATOR: "+" | "-"
MULT_OPERATOR: "*" | "/"
POWER_OPERATOR: "**"
MINUS_OPERATOR: "-"

L_RANGE_EXC: "!["
L_RANGE_INC: "["
R_RANGE_EXC: "]!"
R_RANGE_INC: "]"

ROS_NAME: /[\/~]?[a-zA-Z][0-9a-zA-Z_]*(\/[a-zA-Z][0-9a-zA-Z_]*)*/

VAR_REF: "{op_var}" CNAME

TIME_UNIT: "{kw_secs}" | "{kw_msecs}"
FREQ_UNIT: "hz"

%import common.CNAME
%import common.INT
%import common.NUMBER
%import common.SIGNED_NUMBER
%import common.ESCAPED_STRING
%import common.WS
%ignore WS
""".format(
    kw_true=TOK_VAL_TRUE,
    kw_false=TOK_VAL_FALSE,
    op_in=TOK_OP_IN,
    op_not=TOK_OP_NOT,
    op_implies=TOK_OP_IMPLIES,
    op_iff=TOK_OP_IFF,
    op_or=TOK_OP_OR,
    op_and=TOK_OP_AND,
    op_forall=TOK_OP_FORALL,
    op_exists=TOK_OP_EXISTS,
    kw_pi=TOK_VAL_PI,
    kw_inf=TOK_VAL_INF,
    kw_nan=TOK_VAL_NAN,
    op_var=TOK_VAR_REF,
    kw_secs=TOK_UN_SECS,
    kw_msecs=TOK_UN_MILLIS
)


###############################################################################
# Predicate Grammar
###############################################################################

PREDICATE_GRAMMAR = (r"""
predicate: "{" condition "}"

top_level_condition: condition

condition: [condition IF_OPERATOR] disjunction

disjunction: [disjunction OR_OPERATOR] conjunction

conjunction: [conjunction AND_OPERATOR] _logic_expr

_logic_expr: negation
           | quantification
           | atomic_condition

negation: NOT_OPERATOR _logic_expr

quantification: QUANT_OPERATOR CNAME "in" _atomic_value ":" _logic_expr

atomic_condition: expr [RELATIONAL_OPERATOR expr]

expr: [expr ADD_OPERATOR] term

term: [term MULT_OPERATOR] factor

factor: [factor POWER_OPERATOR] _exponent

_exponent: _atomic_value
         | negative_number
         | "(" condition ")"

negative_number: MINUS_OPERATOR _exponent

_atomic_value: boolean
             | string
             | number_constant
             | number
             | function_call
             | enum_literal
             | range_literal
             | _reference

number_constant: CONSTANT

enum_literal: "{" _enum_member "}"

_enum_member: [_enum_member ","] expr

range_literal: _start_range expr "to" expr _end_range

_start_range: L_RANGE_EXC | L_RANGE_INC

_end_range: R_RANGE_EXC | R_RANGE_INC

variable: VAR_REF

function_call: CNAME "(" expr ")"

_base_ref: variable
         | own_field

own_field: CNAME

_reference: _base_ref
          | field_access
          | array_access

field_access: _reference "." CNAME

array_access: _reference "[" _index "]"

_index: expr
""" + TOKEN_GRAMMAR)


###############################################################################
# Property Grammar
###############################################################################

PROPERTY_GRAMMAR = (r"""
hpl_property: _scope ":" _pattern

_scope: global_scope
      | after_until
      | until

global_scope: "{kw_global}"

after_until: "{kw_after}" activator ["{kw_until}" terminator]

until: "{kw_until}" terminator

activator: event

terminator: event

_pattern: existence
        | absence
        | response
        | prevention
        | requirement

existence: "{kw_some}" event _time_bound?

absence: "{kw_no}" event _time_bound?

response: event "{kw_causes}" event _time_bound?

prevention: event "{kw_forbids}" event _time_bound?

requirement: event "{kw_requires}" event _time_bound?

_time_bound: "{kw_within}" time_amount

event: message predicate?

message: ros_name _alias?

time_amount: NUMBER TIME_UNIT

frequency: NUMBER FREQ_UNIT

_alias: "{kw_alias}" CNAME
""".format(
    kw_global=TOK_KW_GLOBAL,
    kw_after=TOK_KW_AFTER,
    kw_until=TOK_KW_UNTIL,
    kw_some=TOK_KW_SOME,
    kw_no=TOK_KW_NO,
    kw_causes=TOK_KW_CAUSES,
    kw_forbids=TOK_KW_FORBIDS,
    kw_requires=TOK_KW_REQUIRES,
    kw_within=TOK_KW_TIMEOUT,
    kw_alias=TOK_KW_ALIAS
)
+ PREDICATE_GRAMMAR)


###############################################################################
# Assumption Grammar
###############################################################################

ASSUMPTION_GRAMMAR = (r"""
hpl_assumption: ros_name predicate
""" + PREDICATE_GRAMMAR)
