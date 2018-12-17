
#Copyright (c) 2017 Andre Santos
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
# Notes
###############################################################################

# The parsing is based on
# https://github.com/rpavlik/cmake-multitool

# TODO: This module is outdated and needs a major refactoring.
# It should probably replace the parsing with some AST library such as
# https://github.com/polysquare/cmake-ast

###############################################################################
# Imports
###############################################################################

from distutils.version import LooseVersion
import glob
import os
import re


###############################################################################
# CMake Grammar
###############################################################################

class IncompleteStatementError(Exception):
    """Exception raised by parse_line when not given a full valid statement"""
    pass

class CMakeGrammar(object):
    _reParenArgFuncs = (r"(?ix)"
        + "(?P<parenArgFunc>"
        + "|".join(
            """
            if
            else
            elseif
            endif

            while
            endwhile

            math
            """.split())
        + r")")

    _reFuncName = r"(?x) (?P<FuncName> [\w\d]+)"

    _reArgs = r"(?x) (?P<Args> (\S ((\s)*\S)*))"

    _reArg = r"""(?x) (?:
                 (?:\\.|[^"'\s])+|
                 "(?:\\.|[^"\\])*"|
                 '(?:\\.|[^'\\])*')
             """

    _reComment = r"(?x) (?P<Comment> (?<!\\) \# (?: [^\S\n]* \S+)*)"

    _reCommandStart = _reFuncName + r"\s* \("

    _reCommandEnd = r"\)"

    _reFullLine = ( r"^\s*(?P<FullLine>"
                + r"("
                + _reCommandStart
                + r"\s*"
                + r"("
                + _reArgs
                + r")?"
                + r"\s*"
                + _reCommandEnd
                + r")?"
                + r"\s*"
                + r"("
                + _reComment
                + r")?"
                + r")\s*$")

    _reMLChunk = ( r"(?mx)(" + _reComment + "\n|" + _reArg + ")")

    reFullLine = re.compile(_reFullLine, (re.IGNORECASE
                                          | re.VERBOSE
                                          | re.MULTILINE))

    _blockTagsDict = {
        "foreach": ("endforeach",),
        "function": ("endfunction",),
        "if": ("elseif", "else", "endif"),
        "elseif": ("elseif", "else", "endif"),
        "else": ("endif",),
        "macro": ("endmacro",),
        "while": ("endwhile",)
    }

    _reBlockBeginnings = (r"(?ix)"
                          + r"(?P<BlockBeginnings>"
                          + "|".join(_blockTagsDict.keys())
                          + r")")

    reBlockBeginnings = re.compile(_reBlockBeginnings, re.IGNORECASE)

    dReBlockTagsDict = dict([
        (beginning,
         re.compile(r"(?ix)^(?P<BlockEnding>"
                    + r"|".join(ends)
                    + r")$", re.IGNORECASE)
        ) for beginning, ends in _blockTagsDict.iteritems()
    ])

    @staticmethod
    def parse_line(line):
        # Handle EOF sentry: a "None" entry returns an all-None tuple
        if line is None:
            return (None, None, None)
        m = CMakeGrammar.reFullLine.match(line)
        if m is None:
            raise IncompleteStatementError()
        FuncName, Args, Comment = m.group("FuncName", "Args", "Comment")
        if Args is not None and re.search(r"\n", Args) is not None:
            units = re.findall(CMakeGrammar._reMLChunk, Args)
            MLArgs = []
            MLComment = []
            for unit in units:
                if re.match(CMakeGrammar._reComment, unit[0]):
                    MLComment.append(unit[0])
                elif re.match(CMakeGrammar._reArg, unit[0]):
                    MLArgs.append(unit[0])
            Args = " ".join(MLArgs)
            if Comment is not None:
                MLComment.append(Comment)
            if len(MLComment) > 0:
                Comment = "\n".join(MLComment)
            else:
                Comment = None
        if FuncName is None:
            FuncName = ""
        return (FuncName, Args, Comment)

    @staticmethod
    def split_args(args):
        return re.findall(CMakeGrammar._reArg, args)

# sanity check the comprehension above
assert len(CMakeGrammar._blockTagsDict) == len(CMakeGrammar.dReBlockTagsDict)


###############################################################################
# CMake Parser
###############################################################################

class UnclosedChildBlockError(Exception):
    pass

class InputExhaustedError(Exception):
    pass


class ParseInput():
    """Class providing an iterable interface to the parser's input"""

    def __init__(self, strdata):
        self._data = strdata.splitlines()
        # Add a None to the end as a sentry.
        self._data.append(None)
        self._lineindex = 0
        self.alldone = False
        self.gotline = False
        self.alreadyseen = False

    def __iter__(self):
        """Be usable as an iterator."""
        return self

    def next(self):
        """Return the current line each time we are iterated.
        We don't go to the next line unless we've been accepted."""

        # Will always hit this condition if all works well
        if self._lineindex == len(self._data):
            self.alldone = True
            raise StopIteration()

        # If we break, break big
        assert self._lineindex < len(self._data)

        # OK, we can actually return the data now
        self.alreadyseen = self.gotline
        self.gotline = True
        return self._data[self._lineindex]

    def accept(self):
        """Signal that we've processed this line and should go to the next"""

        # We shouldn't accept a line we haven't even seen
        assert self.gotline

        self._lineindex = self._lineindex + 1
        self.gotline = False

    def merge(self):
        # Don't want to merge current line with the next if we don't
        # know what the current line is
        assert self.gotline

        ln = self._lineindex
        if len(self._data) <= ln + 2:
            # we want 2 - one more text line, and the EOF sentinel
            raise InputExhaustedError()
        newdata = self._data[:ln] + [self._data[ln] + "\n" + self._data[ln+1]]
        newdata.extend(self._data[ln+2:])
        # Make sure we didn't change anything - leave off end of file sentinel
        old = "\n".join(self._data[:-1])
        thenew = "\n".join(newdata[:-1])
        assert old == thenew

        self._data = newdata
        return self._data[ln]


class CMakeParser():
    def __init__(self):
        self.input = None
        self.parsetree = None

    def parse(self, filename):
        with open(filename, "r") as cmakefile:
            self.input = ParseInput(cmakefile.read())
        self.parsetree = self.parse_block_children(None)
        if self.parsetree is None:
            self.parsetree = []

    def parse_block_children(self, startTag):
        if startTag is None:
            isEnder = lambda x: (x is None)
        elif CMakeGrammar.reBlockBeginnings.match(startTag):
            endblock = CMakeGrammar.dReBlockTagsDict[startTag.lower()]
            isEnder = endblock.match
        else:
            return None

        block = []
        for line in self.input:
            while True:
                try:
                    func, args, comment = CMakeGrammar.parse_line(line)
                except IncompleteStatementError:
                    try:
                        line = self.input.merge()
                    except InputExhaustedError:
                        raise IncompleteStatementError()
                else:
                    break

            if startTag == None and isEnder(func):
                return block
            elif (func is not None and isEnder(func)
                    and not self.input.alreadyseen):
                return block

            # Not an ender, so we accept this child.
            self.input.accept()
            children = self.parse_block_children(func)
            statement = (func, args, comment, children)
            block.append(statement)

        # If we make it this far, we never found our Ender.
        raise UnclosedChildBlockError()

    def read_until_match(self, text, chars = "()", start = 0):
        left = chars[0]
        right = chars[1]
        assert text[start] == left
        i = start + 1
        matches = 1
        while matches > 0:
            if text[i] == left:
                matches += 1
            elif text[i] == right:
                matches -= 1
            i += 1
        return (text[start+1:i-1], i)

    def split_paren_args(self, text):
        i = text.find("(")
        if i < 0:
            return CMakeGrammar.split_args(text)
        args = CMakeGrammar.split_args(text[:i])
        expr, k = self.read_until_match(text, start = i)
        expr = self.split_paren_args(expr)
        rest = self.split_paren_args(text[k+1:])
        args.append(expr)
        args.extend(rest)
        while len(args) == 1 and isinstance(args[0], list):
            args = args[0]
        return args


###############################################################################
# CMake Extraction
###############################################################################

class BuildTarget(object):
    def __init__(self, name, files, is_executable):
        self.name = name
        self.base_name = name
        self.prefix = "" if is_executable else "lib"
        self.suffix = "" if is_executable else ".so"
        self.files = files
        self.links = []

    @property
    def prefixed_name(self):
        return self.prefix + self.base_name

    @property
    def output_name(self):
        return self.prefix + self.base_name + self.suffix

    @classmethod
    def new_target(cls, name, files, directory, is_executable):
        files = [os.path.join(directory, f) for fs in files \
                                            for f in fs.split(";") if f]
        i = 0
        while i < len(files):
            if not os.path.isfile(files[i]):
                replacement = None
                parent = os.path.dirname(files[i])
                prefix = files[i].rsplit(os.sep, 1)[-1]
                if os.path.isdir(parent):
                    for f in os.listdir(parent):
                        joined = os.path.join(parent, f)
                        if f.startswith(prefix) and os.path.isfile(joined):
                            replacement = joined
                            break
                if replacement:
                    files[i] = replacement
                else:
                    del files[i]
                    i -= 1
            i += 1
        return cls(name, files, is_executable)

    def apply_property(self, prop, value):
        if prop == "PREFIX":
            self.prefix = value
        elif prop == "SUFFIX":
            self.suffix = value
        elif prop == "OUTPUT_NAME":
            self.base_name = value
        # TODO elif prop == "<CONFIG>_OUTPUT_NAME":


class RosCMakeParser(object):
    def __init__(self, srcdir, bindir, pkgs = None, env = None, vars = None):
        self.parser = CMakeParser()
        self.source_dir = srcdir
        self.binary_dir = bindir
        self.packages = pkgs if not pkgs is None else set()
        self.variables = vars if not vars is None else {}
        self.environment = env if not env is None else {}
        self.variables["CMAKE_BINARY_DIR"] = bindir
        self._reset()

    _CONTROL_FLOW = ("if", "else", "elseif", "foreach", "while")

    def parse(self, cmakelists, toplevel = True):
        self.directory = os.path.dirname(cmakelists)
        if toplevel:
            self.directory = os.path.abspath(self.directory)
            self.variables["CMAKE_SOURCE_DIR"] = self.directory
        self.variables["CMAKE_CURRENT_SOURCE_DIR"] = self.directory
        self.variables["CMAKE_CURRENT_LIST_FILE"] = cmakelists
        self.variables["CMAKE_CURRENT_LIST_DIR"] = self.directory
        self.variables["CMAKE_CURRENT_BINARY_DIR"] = (
            self.binary_dir + self.directory[len(self.source_dir):]
        )
        self.parser.parse(cmakelists)
        for stmt in self.parser.parsetree:
            command = stmt[0].lower()
            args = self.parser.split_paren_args(stmt[1]) if stmt[1] else []
            children = stmt[3]
            if command in self._CONTROL_FLOW and children:
                self._analyse_control_flow(command, args, children)
            else:
                self._analyse_command(command, args)
        for subdir in self.subdirectories:
            path = os.path.join(self.directory, subdir, "CMakeLists.txt")
            if os.path.isfile(path):
                parser = RosCMakeParser(self.source_dir, self.binary_dir,
                                          pkgs = self.packages,
                                          env = self.environment,
                                          vars = dict(self.variables))
                parser.parse(path, toplevel = False)
                self._merge(parser)
        if toplevel:
            self._link_targets()

    def _analyse_control_flow(self, command, args, children):
        if command == "else":
            condition = True
        else:
            condition = self._control_arguments(args)
        if condition:
            for stmt in children:
                child_command = stmt[0].lower()
                child_args = self.parser.split_paren_args(stmt[1]) if stmt[1] else []
                child_children = stmt[3]
                if child_command in self._CONTROL_FLOW and child_children:
                    self._analyse_control_flow(child_command, child_args, child_children)
                else:
                    self._analyse_command(child_command, child_args)

    def _analyse_command(self, command, args):
        args = [self._argument(arg) for arg in args]
        if command == "project" and args[0]:
            self.project = args[0]
            self.variables["PROJECT_NAME"] = args[0]
        elif command == "include_directories":
            self._process_include_directories(args)
        elif command == "add_subdirectory" and args[0]:
            self.subdirectories.append(args[0])
        elif command == "add_library":
            self._process_library(args)
        elif command == "add_executable":
            self._process_executable(args)
        elif command == "set":
            self._process_set(args)
        elif command == "unset":
            self._process_unset(args)
        elif command == "find_package":
            self._process_find_package(args)
        elif command == "catkin_package":
            self._process_catkin_package(args)
        elif command == "file":
            self._process_file(args)
        elif command == "set_target_properties":
            self._process_set_target_properties(args)
        elif command == "target_link_libraries":
            self._process_link_libraries(args)

    def _process_include_directories(self, args):
        n = len(args)
        assert n >= 1
        i = 0
        before = self.variables.get("CMAKE_INCLUDE_DIRECTORIES_BEFORE") == "ON"
        before = (before or args[0] == "BEFORE") and args[0] != "AFTER"
        if args[0] == "AFTER" or args[0] == "BEFORE":
            i = 1
        while i < n:
            arg = args[i]
            if arg == "SYSTEM":
                i += 1
            elif arg:
                arg = os.path.join(self.directory, arg)
                self.include_dirs.append(arg)
            i += 1

    def _process_library(self, args):
        n = len(args)
        assert n > 1
        name = args[0]
        i = 1
        flags = ("STATIC", "SHARED", "MODULE", "UNKNOWN", "EXCLUDE_FROM_ALL")
        while args[i] in flags:
            i += 1
            if i >= n:
                return
        if args[i] in ("IMPORTED", "ALIAS", "OBJECT", "INTERFACE"):
            return # TODO
        target = BuildTarget.new_target(name, args[i:], self.directory, False)
        self.libraries[name] = target

    def _process_executable(self, args):
        n = len(args)
        assert n > 1
        name = args[0]
        if args[1] == "IMPORTED" or args[1] == "ALIAS":
            return # TODO
        i = 1
        while args[i] == "WIN32" or args[i] == "MACOSX_BUNDLE" \
                                 or args[i] == "EXCLUDE_FROM_ALL":
            i += 1
        target = BuildTarget.new_target(name, args[i:], self.directory, True)
        self.executables[name] = target

    def _process_set_target_properties(self, args):
        assert len(args) >= 4
        i = args.index("PROPERTIES")
        targets = args[:i]
        k = 0
        while k < len(targets):
            t = targets[k]
            t = self.libraries.get(t, self.executables.get(t))
            if t:
                targets[k] = t
            else:
                del targets[k]
                k -= 1
            k += 1
        i += 1
        while i < len(args):
            prop = args[i]
            value = args[i+1]
            for t in targets:
                t.apply_property(prop, value)
            i += 2

    def _process_link_libraries(self, args):
        assert len(args) >= 1
        t = args[0]
        t = self.libraries.get(t, self.executables.get(t))
        if not t:
            return
        for i in xrange(1, len(args)):
            t.links.append(args[i])

    def _link_targets(self):
        for target in self.libraries.itervalues():
            candidates = target.links
            target.links = []
            for other in candidates:
                d = self.libraries.get(other, self.executables.get(other))
                if d:
                    target.links.append(d)
        for target in self.executables.itervalues():
            candidates = target.links
            target.links = []
            for other in candidates:
                d = self.libraries.get(other, self.executables.get(other))
                if d:
                    target.links.append(d)

    def _process_set(self, args):
        n = len(args)
        assert n > 0
        var = args[0]
        env = var.startswith("ENV{")
        var = var[4:-1] if env else var
        data = self.environment if env else self.variables
        if n == 1:
            if var in data:
                del data[var]
        else:
            i = 1
            while i < n:
                value = args[i]
                if value == "CACHE" or value == "PARENT_SCOPE":
                    break
                i += 1
            value = ";".join(args[1:i])
            data[var] = value

    def _process_unset(self, args):
        assert len(args) > 0
        var = args[0]
        env = var.startswith("ENV{")
        var = var[4:-1] if env else var
        data = self.environment if env else self.variables
        del data[var]

    def _process_find_package(self, args):
        if args[0] != "catkin":
            return
        i = 1
        n = len(args)
        while i < n and args[i] != "COMPONENTS":
            i += 1
        if i < n and args[i] == "COMPONENTS":
            k = i + 1
            while k < n and args[k] != "OPTIONAL_COMPONENTS" \
                        and args[k] != "NO_POLICY_SCOPE":
                k += 1
            args = args[i:k]
            for arg in args:
                if arg in self.packages:
                    self.variables[arg + "_FOUND"] = "TRUE"
                else:
                    self.variables[arg + "_FOUND"] = "FALSE"
                    self.variables[arg + "-NOTFOUND"] = "TRUE"

    def _process_catkin_package(self, args):
        data = None
        for arg in args:
            if arg == "INCLUDE_DIRS":
                data = self.package_includes
            elif arg == "CATKIN_DEPENDS":
                data = self.dependencies
            elif arg == "DEPENDS":
                data = self.system_dependencies
            elif arg == "LIBRARIES" or arg == "CFG_EXTRAS":
                data = [] # ignored
            else:
                data.append(arg)
        data = self.package_includes
        i = 0
        n = len(data)
        while i < n:
            data[i] = os.path.join(self.directory, data[i])
            i += 1

    def _process_file(self, args):
        if args[0] == "GLOB":
            var = args[1]
            if args[2] == "RELATIVE":
                args = args[4:]
            else:
                args = args[2:]
            matches = []
            for expr in args:
                expr = self.directory + os.sep + expr
                matches.extend(glob.glob(expr))
            self.variables[var] = ";".join(matches)

    _TRUTH_CONSTANTS = ("1", "ON", "TRUE", "Y", "YES")
    _FALSE_CONSTANTS = ("0", "OFF", "FALSE", "N", "NO", "IGNORE", "NOTFOUND", "")
    _UNARY_OPERATORS = ("DEFINED", "COMMAND", "POLICY", "TARGET", "EXISTS",
                        "IS_DIRECTORY", "IS_SYMLINK", "IS_ABSOLUTE")
    _BINARY_OPERATORS = ("EQUAL", "LESS", "GREATER", "STREQUAL", "STRLESS",
                         "STRGREATER", "MATCHES", "IS_NEWER_THAN",
                         "VERSION_EQUAL", "VERSION_LESS", "VERSION_GREATER")

    def _control_arguments(self, args):
        # Step 0: evaluate ${variables}
        # Step 1: evaluate parentheses
        args = [self._control_arguments(arg) \
                if isinstance(arg, list) \
                else self._argument(arg)
                for arg in args]
        # Step 2: evaluate unary tests
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg in self._UNARY_OPERATORS:
                i += 1
                arg = self._unary_test(arg, args[i])
            values.append(arg)
            i += 1
        # Step 3: evaluate binary tests
        args = values
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg in self._BINARY_OPERATORS:
                i += 1
                values.pop()
                arg = self._binary_test(arg, args[i-2], args[i])
            values.append(arg)
            i += 1
        # Step 4: evaluate boolean values
        i = 0
        n = len(values)
        while i < n:
            value = values[i]
            uval = value.upper()
            if uval in self._TRUTH_CONSTANTS:
                values[i] = True
            elif uval in self._FALSE_CONSTANTS or uval.endswith("-NOTFOUND"):
                values[i] = False
            elif value != "NOT" and value != "AND" and value != "OR":
                value = self.variables.get(value, "FALSE")
                values[i] = (not value in self._FALSE_CONSTANTS
                             and not value.endswith("-NOTFOUND"))
            i += 1
        # Step 5: evaluate boolean NOT
        args = values
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg == "NOT":
                i += 1
                assert isinstance(args[i], bool)
                arg = not args[i]
            values.append(arg)
            i += 1
        # Step 6: evaluate boolean AND and OR
        args = values
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg == "AND" or arg == "OR":
                i += 1
                values.pop()
                if arg == "AND":
                    arg = args[i-2] and args[i]
                else:
                    arg = args[i-2] or args[i]
            values.append(arg)
            i += 1
        return values[0]

    def _unary_test(self, test, arg):
        if test == "DEFINED":
            return "TRUE" if arg in self.variables else "FALSE"
        if test == "TARGET":
            return ("TRUE" if arg in self.libraries
                    or arg in self.executables
                    else "FALSE")
        if test == "EXISTS":
            return "TRUE" if os.path.exists(arg) else "FALSE"
        if test == "IS_DIRECTORY":
            return "TRUE" if os.path.isdir(arg) else "FALSE"
        if test == "IS_SYMLINK":
            return "TRUE" if os.path.islink(arg) else "FALSE"
        if test == "IS_ABSOLUTE":
            return "TRUE" if os.path.isabs(arg) else "FALSE"
        # TODO COMMAND and POLICY
        return "FALSE"

    def _binary_test(self, test, arg1, arg2):
        if not arg1 or not arg2:
            return "FALSE"
        if test == "IS_NEWER_THAN":
            return "TRUE" if not os.path.exists(arg1) \
                   or not os.path.exists(arg2) \
                   or os.path.getmtime(arg1) - os.path.getmtime(arg2) >= 0 \
                   else "FALSE"
        arg1 = self.variables.get(arg1, arg1)
        if test == "MATCHES":
            return "FALSE"  # TODO
        arg2 = self.variables.get(arg2, arg2)
        if test == "STRLESS":
            return "TRUE" if arg1 < arg2 else "FALSE"
        if test == "STRGREATER":
            return "TRUE" if arg1 > arg2 else "FALSE"
        if test == "STREQUAL":
            return "TRUE" if arg1 == arg2 else "FALSE"
        if test == "VERSION_LESS":
            return "TRUE" if LooseVersion(arg1) < LooseVersion(arg2) else "FALSE"
        if test == "VERSION_GREATER":
            return "TRUE" if LooseVersion(arg1) > LooseVersion(arg2) else "FALSE"
        if test == "VERSION_EQUAL":
            return "TRUE" if LooseVersion(arg1) == LooseVersion(arg2) else "FALSE"
        # only the numeric tests remain
        try:
            if test == "LESS":
                return "TRUE" if int(arg1) < int(arg2) else "FALSE"
            if test == "GREATER":
                return "TRUE" if int(arg1) > int(arg2) else "FALSE"
            return "TRUE" if int(arg1) == int(arg2) else "FALSE"
        except ValueError as e:
            return "FALSE"

    def _argument(self, arg):
        i = 0
        n = len(arg)
        while i < n and arg[i] != "$":
            i += 1
        if i == n:
            return arg.replace('"', "")
        prefix = arg[:i]
        suffix = self._maybe_variable(arg[i:])
        return (prefix + suffix).replace('"', "")

    def _maybe_variable(self, arg):
        assert arg[0] == "$"
        is_env_var = arg.startswith("$ENV{")
        if arg.startswith("${") or is_env_var:
            value, k = self._variable(arg, is_env_var)
            suffix = self._argument(arg[k:])
            return value + suffix
        suffix = self._argument(arg[1:])
        return "$" + suffix

    def _variable(self, arg, env):
        var, k = self.parser.read_until_match(arg, chars = "{}", start = 4 if env else 1)
        var = self._argument(var)
        if env:
            value = self.environment.get(var, "")
        else:
            value = self.variables.get(var, "")
        return (value, k)

    def _merge(self, other):
        self.include_dirs.extend(other.include_dirs)
        self.libraries.update(other.libraries)
        self.executables.update(other.executables)

    def _reset(self):
        self.project = None
        self.package_includes = []
        self.include_dirs = []
        self.dependencies = []
        self.system_dependencies = []
        self.libraries = {}
        self.executables = {}
        self.subdirectories = []
