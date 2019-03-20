
#Copyright (c) 2019 Andre Santos
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

from __future__ import unicode_literals
from builtins import str, bytes

import sys

from pyflwor.parser import Parser
from pyflwor.lexer import Lexer
from ply import lex, yacc


###############################################################################
# Monkey-patched Classes
###############################################################################

class MonkeyPatchLexer(Lexer):
    def __new__(cls, pyflwor_dir, **kwargs):
        self = super(Lexer, cls).__new__(cls, **kwargs)
        self.lexer = lex.lex(object=self, debug=False, optimize=True,
                             outputdir=pyflwor_dir, **kwargs)
        return self.lexer


class MonkeyPatchParser(Parser):
    def __new__(cls, pyflwor_dir, **kwargs):
        self = super(Parser, cls).__new__(cls, **kwargs)
        self.names = dict()
        self.yacc = yacc.yacc(module=self, debug=False,
                              optimize=True, write_tables=False,
                              outputdir=pyflwor_dir, **kwargs)
        return self.yacc


###############################################################################
# Entry Point
###############################################################################

def make_parser(pyflwor_dir):
    if pyflwor_dir not in sys.path:
        sys.path.insert(0, pyflwor_dir)
    def execute(query, namespace):
        lexer = MonkeyPatchLexer(pyflwor_dir)
        parser = MonkeyPatchParser(pyflwor_dir)
        qbytes = bytes(query, "utf-8").decode("unicode_escape")
        qfunction = parser.parse(qbytes, lexer=lexer)
        return qfunction(namespace)
    return execute
