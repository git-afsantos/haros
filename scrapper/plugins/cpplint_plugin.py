
import codecs
import os
import sys
import re


# These constants define types of headers for use with
# IncludeState.CheckNextIncludeOrder().
_C_SYS_HEADER = 1
_CPP_SYS_HEADER = 2
_LIKELY_MY_HEADER = 3
_POSSIBLE_MY_HEADER = 4
_OTHER_HEADER = 5

# These constants define the current inline assembly state
_NO_ASM = 0       # Outside of inline assembly block
_INSIDE_ASM = 1   # Inside inline assembly block
_END_ASM = 2      # Last line of inline assembly block
_BLOCK_ASM = 3    # The whole block is an inline assembly block

# Match start of assembly blocks
_MATCH_ASM = re.compile(r'^\s*(?:asm|_asm|__asm|__asm__)'
                        r'(?:\s+(volatile|__volatile__))?'
                        r'\s*[{(]')

# Matches standard C++ escape sequences per 2.13.2.3 of the C++ standard.
_RE_PATTERN_CLEANSE_LINE_ESCAPES = re.compile(
    r'\\([abfnrtv?"\\\']|\d+|x[0-9a-fA-F]+)')
# Match a single C style comment on the same line.
_RE_PATTERN_C_COMMENTS = r'/\*(?:[^*]|\*(?!/))*\*/'
# Matches multi-line C style comments.
# This RE is a little bit more complicated than one might expect, because we
# have to take care of space removals tools so we can handle comments inside
# statements better.
# The current rule is: We only clear spaces from both sides when we're at the
# end of the line. Otherwise, we try to remove spaces from the right side,
# if this doesn't work we try on left side but only if there's a non-character
# on the right.
_RE_PATTERN_CLEANSE_LINE_C_COMMENTS = re.compile(
    r'(\s*' + _RE_PATTERN_C_COMMENTS + r'\s*$|' +
    _RE_PATTERN_C_COMMENTS + r'\s+|' +
    r'\s+' + _RE_PATTERN_C_COMMENTS + r'(?=\W)|' +
    _RE_PATTERN_C_COMMENTS + r')')

_RE_PATTERN_INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]*)[>"].*$')
# Matches the first component of a filename delimited by -s and _s. That is:
#  _RE_FIRST_COMPONENT.match('foo').group(0) == 'foo'
#  _RE_FIRST_COMPONENT.match('foo.cc').group(0) == 'foo'
#  _RE_FIRST_COMPONENT.match('foo-bar_baz.cc').group(0) == 'foo'
#  _RE_FIRST_COMPONENT.match('foo_bar-baz.cc').group(0) == 'foo'
_RE_FIRST_COMPONENT = re.compile(r'^[^-_.]+')



# {str, set(int)}: a map from error categories to sets of linenumbers
# on which those errors are expected and should be suppressed.
_error_suppressions = {}


_regexp_compile_cache = {}



def plugin_run(ctx):
    files = get_file_list(ctx)
    for f in files:
        process_file(ctx, f)



def get_file_list(ctx):
    return [(1, "keyop_core.cpp", "kobuki/kobuki_keyop/src", 1),
            (2, "keyop_core.hpp", "kobuki/kobuki_keyop/include/keyop_core", 1)]



def process_file(ctx, file_info):
    fpath = get_file_path(ctx, file_info)
    try:
        fdata = preprocess_file(ctx, file_info, fpath)
    except IOError:
        print "Error reading file", fpath
        return
    process_file_data(ctx, fdata)



def preprocess_file(ctx, file_info, fpath):
    fname       = file_info[1]
    lf_lines    = []
    crlf_lines  = []
    lines = codecs.open(fpath, "r", "utf8", "replace").read().split("\n")
    # Remove trailing "\r".
    # The -1 accounts for the extra trailing blank line we get from split()
    for linenum in range(len(lines) - 1):
        if lines[linenum].endswith("\r"):
            lines[linenum] = lines[linenum].rstrip("\r")
            crlf_lines.append(linenum + 1)
        else:
            lf_lines.append(linenum + 1)
    # Note, if no dot is found, this will give the entire filename as the ext.
    fext = fname[fname.rfind(".") + 1:]
    return {
        "id": file_info[0],
        "name": fname,
        "path": file_info[2],
        "absolute": fpath,
        "package": file_info[3],
        "lines": (['// marker so line numbers and indices both start at 1'] +
                    lines +
                    ['// marker so line numbers end in a known way']),
        "lf": lf_lines,
        "crlf": crlf_lines,
        "ext": fext
    }



def get_file_path(ctx, file_info):
    return os.path.join(os.path.expanduser("~"), "ros", "repos",
            file_info[2], file_info[1])



def process_file_data(ctx, fdata):
    include_state   = IncludeState()
    function_state  = FunctionState()
    nesting_state   = NestingState()

    reset_nolint_suppressions()
    process_line_endings(ctx, fdata)
    check_for_copyright(ctx, fdata)
    remove_multiline_comments(ctx, fdata)
    clean_lines = CleansedLines(lines)



def process_line_endings(ctx, fdata):
    # End-of-line sequences should not be a mix of LF and CR-LF.
    if fdata["lf"] and fdata["crlf"]:
        if len(fdata["lf"]) < len(fdata["crlf"]):
            lines = fdata["lf"]
            msg = "LF line-ending in a file with mostly CRLF line-endings"
        else:
            lines = fdata["crlf"]
            msg = "CRLF line-ending in a file with mostly LF line-endings"
        for linenum in lines:
            ctx.writeNonCompliance(10000, fdata["package"],
                    file_id=fdata["id"], line=linenum,
                    comment=msg)



def reset_nolint_suppressions():
    """Resets the set of NOLINT suppressions to empty."""
    _error_suppressions.clear()


def check_for_copyright(ctx, fdata, filename, lines, error):
    """Logs an error if no Copyright message appears at the top of the file."""
    # We'll say it should occur by line 10. Don't forget there's a
    # dummy line at the front.
    lines = fdata["lines"]
    for line in xrange(1, min(len(lines), 11)):
        if re.search(r'Copyright', lines[line], re.I): break
    else:                       # means no copyright line was found
        ctx.writeNonCompliance(10100, fdata["package"],
                file_id=fdata["id"],
                comment="No copyright message found." +
                    "You should have a line: 'Copyright [year] <owner>'")


def remove_multiline_comments(ctx, fdata):
    """Removes multiline (c-style) comments from lines."""
    lineix = 0
    while lineix < len(lines):
        lineix_begin = find_next_multiline_comment_start(lines, lineix)
        if lineix_begin >= len(lines):
            return
        lineix_end = find_next_multiline_comment_end(lines, lineix_begin)
        if lineix_end >= len(lines):
            ctx.writeNonCompliance(10101, fdata["package"],
                    file_id=fdata["id"],
                    comment="Could not find end of multi-line comment")
            return
        remove_multiline_comments_from_range(lines, lineix_begin, lineix_end + 1)
        lineix = lineix_end + 1



def remove_multiline_comments_from_range(lines, begin, end):
    """Clears a range of lines for multi-line comments."""
    # Having // dummy comments makes the lines non-empty, so we will not get
    # unnecessary blank line warnings later in the code.
    for i in range(begin, end):
        lines[i] = '/**/'


def find_next_multiline_comment_start(lines, lineix):
    """Find the beginning marker for a multiline comment."""
    while lineix < len(lines):
        if lines[lineix].strip().startswith('/*'):
            # Only return this marker if the comment goes beyond this line
            if lines[lineix].strip().find('*/', 2) < 0:
                return lineix
        lineix += 1
    return len(lines)


def find_next_multiline_comment_end(lines, lineix):
    """We are inside a comment, find the end marker."""
    while lineix < len(lines):
        if lines[lineix].strip().endswith('*/'):
            return lineix
        lineix += 1
    return len(lines)



class IncludeState(object):
  """Tracks line numbers for includes, and the order in which includes appear.

  include_list contains list of lists of (header, line number) pairs.
  It's a lists of lists rather than just one flat list to make it
  easier to update across preprocessor boundaries.

  Call CheckNextIncludeOrder() once for each header in the file, passing
  in the type constants defined above. Calls in an illegal order will
  raise an _IncludeError with an appropriate error message.

  """
  # self._section will move monotonically through this set. If it ever
  # needs to move backwards, CheckNextIncludeOrder will raise an error.
  _INITIAL_SECTION = 0
  _MY_H_SECTION = 1
  _C_SECTION = 2
  _CPP_SECTION = 3
  _OTHER_H_SECTION = 4

  _TYPE_NAMES = {
      _C_SYS_HEADER: 'C system header',
      _CPP_SYS_HEADER: 'C++ system header',
      _LIKELY_MY_HEADER: 'header this file implements',
      _POSSIBLE_MY_HEADER: 'header this file may implement',
      _OTHER_HEADER: 'other header',
      }
  _SECTION_NAMES = {
      _INITIAL_SECTION: "... nothing. (This can't be an error.)",
      _MY_H_SECTION: 'a header this file implements',
      _C_SECTION: 'C system header',
      _CPP_SECTION: 'C++ system header',
      _OTHER_H_SECTION: 'other header',
      }

  def __init__(self):
    self.include_list = [[]]
    self.ResetSection('')

  def FindHeader(self, header):
    """Check if a header has already been included.

    Args:
      header: header to check.
    Returns:
      Line number of previous occurrence, or -1 if the header has not
      been seen before.
    """
    for section_list in self.include_list:
      for f in section_list:
        if f[0] == header:
          return f[1]
    return -1

  def ResetSection(self, directive):
    """Reset section checking for preprocessor directive.

    Args:
      directive: preprocessor directive (e.g. "if", "else").
    """
    # The name of the current section.
    self._section = self._INITIAL_SECTION
    # The path of last found header.
    self._last_header = ''

    # Update list of includes.  Note that we never pop from the
    # include list.
    if directive in ('if', 'ifdef', 'ifndef'):
      self.include_list.append([])
    elif directive in ('else', 'elif'):
      self.include_list[-1] = []

  def SetLastHeader(self, header_path):
    self._last_header = header_path

  def CanonicalizeAlphabeticalOrder(self, header_path):
    """Returns a path canonicalized for alphabetical comparison.

    - replaces "-" with "_" so they both cmp the same.
    - removes '-inl' since we don't require them to be after the main header.
    - lowercase everything, just in case.

    Args:
      header_path: Path to be canonicalized.

    Returns:
      Canonicalized path.
    """
    return header_path.replace('-inl.h', '.h').replace('-', '_').lower()

  def IsInAlphabeticalOrder(self, clean_lines, linenum, header_path):
    """Check if a header is in alphabetical order with the previous header.

    Args:
      clean_lines: A CleansedLines instance containing the file.
      linenum: The number of the line to check.
      header_path: Canonicalized header to be checked.

    Returns:
      Returns true if the header is in alphabetical order.
    """
    # If previous section is different from current section, _last_header will
    # be reset to empty string, so it's always less than current header.
    #
    # If previous line was a blank line, assume that the headers are
    # intentionally sorted the way they are.
    if (self._last_header > header_path and
        Match(r'^\s*#\s*include\b', clean_lines.elided[linenum - 1])):
      return False
    return True

  def CheckNextIncludeOrder(self, header_type):
    """Returns a non-empty error message if the next header is out of order.

    This function also updates the internal state to be ready to check
    the next include.

    Args:
      header_type: One of the _XXX_HEADER constants defined above.

    Returns:
      The empty string if the header is in the right order, or an
      error message describing what's wrong.

    """
    error_message = ('Found %s after %s' %
                     (self._TYPE_NAMES[header_type],
                      self._SECTION_NAMES[self._section]))

    last_section = self._section

    if header_type == _C_SYS_HEADER:
      if self._section <= self._C_SECTION:
        self._section = self._C_SECTION
      else:
        self._last_header = ''
        return error_message
    elif header_type == _CPP_SYS_HEADER:
      if self._section <= self._CPP_SECTION:
        self._section = self._CPP_SECTION
      else:
        self._last_header = ''
        return error_message
    elif header_type == _LIKELY_MY_HEADER:
      if self._section <= self._MY_H_SECTION:
        self._section = self._MY_H_SECTION
      else:
        self._section = self._OTHER_H_SECTION
    elif header_type == _POSSIBLE_MY_HEADER:
      if self._section <= self._MY_H_SECTION:
        self._section = self._MY_H_SECTION
      else:
        # This will always be the fallback because we're not sure
        # enough that the header is associated with this file.
        self._section = self._OTHER_H_SECTION
    else:
      assert header_type == _OTHER_HEADER
      self._section = self._OTHER_H_SECTION

    if last_section != self._section:
      self._last_header = ''

    return ''


class FunctionState(object):
  """Tracks current function name and the number of lines in its body."""

  _NORMAL_TRIGGER = 250  # for --v=0, 500 for --v=1, etc.
  _TEST_TRIGGER = 400    # about 50% more than _NORMAL_TRIGGER.

  def __init__(self):
    self.in_a_function = False
    self.lines_in_function = 0
    self.current_function = ''

  def Begin(self, function_name):
    """Start analyzing function body.

    Args:
      function_name: The name of the function being tracked.
    """
    self.in_a_function = True
    self.lines_in_function = 0
    self.current_function = function_name

  def Count(self):
    """Count line in current function body."""
    if self.in_a_function:
      self.lines_in_function += 1

  def Check(self, error, filename, linenum):
    """Report if too many lines in function body.

    Args:
      error: The function to call with any errors found.
      filename: The name of the current file.
      linenum: The number of the line to check.
    """
    if Match(r'T(EST|est)', self.current_function):
      base_trigger = self._TEST_TRIGGER
    else:
      base_trigger = self._NORMAL_TRIGGER
    trigger = base_trigger * 2**1 # _VerboseLevel()

    if self.lines_in_function > trigger:
      error_level = int(math.log(self.lines_in_function / base_trigger, 2))
      # 50 => 0, 100 => 1, 200 => 2, 400 => 3, 800 => 4, 1600 => 5, ...
      if error_level > 5:
        error_level = 5
      # error(filename, linenum, 'readability/fn_size', error_level,
            # 'Small and focused functions are preferred:'
            # ' %s has %d non-comment lines'
            # ' (error triggered by exceeding %d lines).'  % (
                # self.current_function, self.lines_in_function, trigger))

  def End(self):
    """Stop analyzing function body."""
    self.in_a_function = False



class NestingState(object):
  """Holds states related to parsing braces."""

  def __init__(self):
    # Stack for tracking all braces.  An object is pushed whenever we
    # see a "{", and popped when we see a "}".  Only 3 types of
    # objects are possible:
    # - _ClassInfo: a class or struct.
    # - _NamespaceInfo: a namespace.
    # - _BlockInfo: some other type of block.
    self.stack = []

    # Top of the previous stack before each Update().
    #
    # Because the nesting_stack is updated at the end of each line, we
    # had to do some convoluted checks to find out what is the current
    # scope at the beginning of the line.  This check is simplified by
    # saving the previous top of nesting stack.
    #
    # We could save the full stack, but we only need the top.  Copying
    # the full nesting stack would slow down cpplint by ~10%.
    self.previous_stack_top = []

    # Stack of _PreprocessorInfo objects.
    self.pp_stack = []

  def SeenOpenBrace(self):
    """Check if we have seen the opening brace for the innermost block.

    Returns:
      True if we have seen the opening brace, False if the innermost
      block is still expecting an opening brace.
    """
    return (not self.stack) or self.stack[-1].seen_open_brace

  def InNamespaceBody(self):
    """Check if we are currently one level inside a namespace body.

    Returns:
      True if top of the stack is a namespace block, False otherwise.
    """
    return self.stack and isinstance(self.stack[-1], NamespaceInfo)

  def InExternC(self):
    """Check if we are currently one level inside an 'extern "C"' block.

    Returns:
      True if top of the stack is an extern block, False otherwise.
    """
    return self.stack and isinstance(self.stack[-1], ExternCInfo)

  def InClassDeclaration(self):
    """Check if we are currently one level inside a class or struct declaration.

    Returns:
      True if top of the stack is a class/struct, False otherwise.
    """
    return self.stack and isinstance(self.stack[-1], ClassInfo)

  def InAsmBlock(self):
    """Check if we are currently one level inside an inline ASM block.

    Returns:
      True if the top of the stack is a block containing inline ASM.
    """
    return self.stack and self.stack[-1].inline_asm != _NO_ASM

  def InTemplateArgumentList(self, clean_lines, linenum, pos):
    """Check if current position is inside template argument list.

    Args:
      clean_lines: A CleansedLines instance containing the file.
      linenum: The number of the line to check.
      pos: position just after the suspected template argument.
    Returns:
      True if (linenum, pos) is inside template arguments.
    """
    while linenum < clean_lines.NumLines():
      # Find the earliest character that might indicate a template argument
      line = clean_lines.elided[linenum]
      match = Match(r'^[^{};=\[\]\.<>]*(.)', line[pos:])
      if not match:
        linenum += 1
        pos = 0
        continue
      token = match.group(1)
      pos += len(match.group(0))

      # These things do not look like template argument list:
      #   class Suspect {
      #   class Suspect x; }
      if token in ('{', '}', ';'): return False

      # These things look like template argument list:
      #   template <class Suspect>
      #   template <class Suspect = default_value>
      #   template <class Suspect[]>
      #   template <class Suspect...>
      if token in ('>', '=', '[', ']', '.'): return True

      # Check if token is an unmatched '<'.
      # If not, move on to the next character.
      if token != '<':
        pos += 1
        if pos >= len(line):
          linenum += 1
          pos = 0
        continue

      # We can't be sure if we just find a single '<', and need to
      # find the matching '>'.
      (_, end_line, end_pos) = CloseExpression(clean_lines, linenum, pos - 1)
      if end_pos < 0:
        # Not sure if template argument list or syntax error in file
        return False
      linenum = end_line
      pos = end_pos
    return False

  def UpdatePreprocessor(self, line):
    """Update preprocessor stack.

    We need to handle preprocessors due to classes like this:
      #ifdef SWIG
      struct ResultDetailsPageElementExtensionPoint {
      #else
      struct ResultDetailsPageElementExtensionPoint : public Extension {
      #endif

    We make the following assumptions (good enough for most files):
    - Preprocessor condition evaluates to true from #if up to first
      #else/#elif/#endif.

    - Preprocessor condition evaluates to false from #else/#elif up
      to #endif.  We still perform lint checks on these lines, but
      these do not affect nesting stack.

    Args:
      line: current line to check.
    """
    if Match(r'^\s*#\s*(if|ifdef|ifndef)\b', line):
      # Beginning of #if block, save the nesting stack here.  The saved
      # stack will allow us to restore the parsing state in the #else case.
      self.pp_stack.append(PreprocessorInfo(copy.deepcopy(self.stack)))
    elif Match(r'^\s*#\s*(else|elif)\b', line):
      # Beginning of #else block
      if self.pp_stack:
        if not self.pp_stack[-1].seen_else:
          # This is the first #else or #elif block.  Remember the
          # whole nesting stack up to this point.  This is what we
          # keep after the #endif.
          self.pp_stack[-1].seen_else = True
          self.pp_stack[-1].stack_before_else = copy.deepcopy(self.stack)

        # Restore the stack to how it was before the #if
        self.stack = copy.deepcopy(self.pp_stack[-1].stack_before_if)
      else:
        # TODO(unknown): unexpected #else, issue warning?
        pass
    elif Match(r'^\s*#\s*endif\b', line):
      # End of #if or #else blocks.
      if self.pp_stack:
        # If we saw an #else, we will need to restore the nesting
        # stack to its former state before the #else, otherwise we
        # will just continue from where we left off.
        if self.pp_stack[-1].seen_else:
          # Here we can just use a shallow copy since we are the last
          # reference to it.
          self.stack = self.pp_stack[-1].stack_before_else
        # Drop the corresponding #if
        self.pp_stack.pop()
      else:
        # TODO(unknown): unexpected #endif, issue warning?
        pass

  # TODO(unknown): Update() is too long, but we will refactor later.
  def Update(self, filename, clean_lines, linenum, error):
    """Update nesting state with current line.

    Args:
      filename: The name of the current file.
      clean_lines: A CleansedLines instance containing the file.
      linenum: The number of the line to check.
      error: The function to call with any errors found.
    """
    line = clean_lines.elided[linenum]

    # Remember top of the previous nesting stack.
    #
    # The stack is always pushed/popped and not modified in place, so
    # we can just do a shallow copy instead of copy.deepcopy.  Using
    # deepcopy would slow down cpplint by ~28%.
    if self.stack:
      self.previous_stack_top = self.stack[-1]
    else:
      self.previous_stack_top = None

    # Update pp_stack
    self.UpdatePreprocessor(line)

    # Count parentheses.  This is to avoid adding struct arguments to
    # the nesting stack.
    if self.stack:
      inner_block = self.stack[-1]
      depth_change = line.count('(') - line.count(')')
      inner_block.open_parentheses += depth_change

      # Also check if we are starting or ending an inline assembly block.
      if inner_block.inline_asm in (_NO_ASM, _END_ASM):
        if (depth_change != 0 and
            inner_block.open_parentheses == 1 and
            _MATCH_ASM.match(line)):
          # Enter assembly block
          inner_block.inline_asm = _INSIDE_ASM
        else:
          # Not entering assembly block.  If previous line was _END_ASM,
          # we will now shift to _NO_ASM state.
          inner_block.inline_asm = _NO_ASM
      elif (inner_block.inline_asm == _INSIDE_ASM and
            inner_block.open_parentheses == 0):
        # Exit assembly block
        inner_block.inline_asm = _END_ASM

    # Consume namespace declaration at the beginning of the line.  Do
    # this in a loop so that we catch same line declarations like this:
    #   namespace proto2 { namespace bridge { class MessageSet; } }
    while True:
      # Match start of namespace.  The "\b\s*" below catches namespace
      # declarations even if it weren't followed by a whitespace, this
      # is so that we don't confuse our namespace checker.  The
      # missing spaces will be flagged by CheckSpacing.
      namespace_decl_match = Match(r'^\s*namespace\b\s*([:\w]+)?(.*)$', line)
      if not namespace_decl_match:
        break

      new_namespace = _NamespaceInfo(namespace_decl_match.group(1), linenum)
      self.stack.append(new_namespace)

      line = namespace_decl_match.group(2)
      if line.find('{') != -1:
        new_namespace.seen_open_brace = True
        line = line[line.find('{') + 1:]

    # Look for a class declaration in whatever is left of the line
    # after parsing namespaces.  The regexp accounts for decorated classes
    # such as in:
    #   class LOCKABLE API Object {
    #   };
    class_decl_match = Match(
        r'^(\s*(?:template\s*<[\w\s<>,:]*>\s*)?'
        r'(class|struct)\s+(?:[A-Z_]+\s+)*(\w+(?:::\w+)*))'
        r'(.*)$', line)
    if (class_decl_match and
        (not self.stack or self.stack[-1].open_parentheses == 0)):
      # We do not want to accept classes that are actually template arguments:
      #   template <class Ignore1,
      #             class Ignore2 = Default<Args>,
      #             template <Args> class Ignore3>
      #   void Function() {};
      #
      # To avoid template argument cases, we scan forward and look for
      # an unmatched '>'.  If we see one, assume we are inside a
      # template argument list.
      end_declaration = len(class_decl_match.group(1))
      if not self.InTemplateArgumentList(clean_lines, linenum, end_declaration):
        self.stack.append(_ClassInfo(
            class_decl_match.group(3), class_decl_match.group(2),
            clean_lines, linenum))
        line = class_decl_match.group(4)

    # If we have not yet seen the opening brace for the innermost block,
    # run checks here.
    if not self.SeenOpenBrace():
      self.stack[-1].CheckBegin(filename, clean_lines, linenum, error)

    # Update access control if we are inside a class/struct
    if self.stack and isinstance(self.stack[-1], ClassInfo):
      classinfo = self.stack[-1]
      access_match = Match(
          r'^(.*)\b(public|private|protected|signals)(\s+(?:slots\s*)?)?'
          r':(?:[^:]|$)',
          line)
      if access_match:
        classinfo.access = access_match.group(2)

        # Check that access keywords are indented +1 space.  Skip this
        # check if the keywords are not preceded by whitespaces.
        indent = access_match.group(1)
        if (len(indent) != classinfo.class_indent + 1 and
            Match(r'^\s*$', indent)):
          if classinfo.is_struct:
            parent = 'struct ' + classinfo.name
          else:
            parent = 'class ' + classinfo.name
          slots = ''
          if access_match.group(3):
            slots = access_match.group(3)
          # error(filename, linenum, 'whitespace/indent', 3,
                # '%s%s: should be indented +1 space inside %s' % (
                    # access_match.group(2), slots, parent))

    # Consume braces or semicolons from what's left of the line
    while True:
      # Match first brace, semicolon, or closed parenthesis.
      matched = Match(r'^[^{;)}]*([{;)}])(.*)$', line)
      if not matched:
        break

      token = matched.group(1)
      if token == '{':
        # If namespace or class hasn't seen a opening brace yet, mark
        # namespace/class head as complete.  Push a new block onto the
        # stack otherwise.
        if not self.SeenOpenBrace():
          self.stack[-1].seen_open_brace = True
        elif Match(r'^extern\s*"[^"]*"\s*\{', line):
          self.stack.append(_ExternCInfo())
        else:
          self.stack.append(_BlockInfo(True))
          if _MATCH_ASM.match(line):
            self.stack[-1].inline_asm = _BLOCK_ASM

      elif token == ';' or token == ')':
        # If we haven't seen an opening brace yet, but we already saw
        # a semicolon, this is probably a forward declaration.  Pop
        # the stack for these.
        #
        # Similarly, if we haven't seen an opening brace yet, but we
        # already saw a closing parenthesis, then these are probably
        # function arguments with extra "class" or "struct" keywords.
        # Also pop these stack for these.
        if not self.SeenOpenBrace():
          self.stack.pop()
      else:  # token == '}'
        # Perform end of block checks and pop the stack.
        if self.stack:
          self.stack[-1].CheckEnd(filename, clean_lines, linenum, error)
          self.stack.pop()
      line = matched.group(2)

  def InnermostClass(self):
    """Get class info on the top of the stack.

    Returns:
      A _ClassInfo object if we are inside a class, or None otherwise.
    """
    for i in range(len(self.stack), 0, -1):
      classinfo = self.stack[i - 1]
      if isinstance(classinfo, _ClassInfo):
        return classinfo
    return None

  def CheckCompletedBlocks(self, filename, error):
    """Checks that all classes and namespaces have been completely parsed.

    Call this when all lines in a file have been processed.
    Args:
      filename: The name of the current file.
      error: The function to call with any errors found.
    """
    # Note: This test can result in false positives if #ifdef constructs
    # get in the way of brace matching. See the testBuildClass test in
    # cpplint_unittest.py for an example of this.
    for obj in self.stack:
      if isinstance(obj, ClassInfo):
        # error(filename, obj.starting_linenum, 'build/class', 5,
              # 'Failed to find complete declaration of class %s' %
              # obj.name)
      elif isinstance(obj, NamespaceInfo):
        # error(filename, obj.starting_linenum, 'build/namespaces', 5,
              # 'Failed to find complete declaration of namespace %s' %
              # obj.name)



class CleansedLines(object):
  """Holds 4 copies of all lines with different preprocessing applied to them.

  1) elided member contains lines without strings and comments.
  2) lines member contains lines without comments.
  3) raw_lines member contains all the lines without processing.
  4) lines_without_raw_strings member is same as raw_lines, but with C++11 raw
     strings removed.
  All these members are of <type 'list'>, and of the same length.
  """

  def __init__(self, lines):
    self.elided = []
    self.lines = []
    self.raw_lines = lines
    self.num_lines = len(lines)
    self.lines_without_raw_strings = CleanseRawStrings(lines)
    for linenum in range(len(self.lines_without_raw_strings)):
      self.lines.append(CleanseComments(
          self.lines_without_raw_strings[linenum]))
      elided = self._CollapseStrings(self.lines_without_raw_strings[linenum])
      self.elided.append(CleanseComments(elided))

  def NumLines(self):
    """Returns the number of lines represented."""
    return self.num_lines

  @staticmethod
  def _CollapseStrings(elided):
    """Collapses strings and chars on a line to simple "" or '' blocks.

    We nix strings first so we're not fooled by text like '"http://"'

    Args:
      elided: The line being processed.

    Returns:
      The line with collapsed strings.
    """
    if _RE_PATTERN_INCLUDE.match(elided):
      return elided

    # Remove escaped characters first to make quote/single quote collapsing
    # basic.  Things that look like escaped characters shouldn't occur
    # outside of strings and chars.
    elided = _RE_PATTERN_CLEANSE_LINE_ESCAPES.sub('', elided)

    # Replace quoted strings and digit separators.  Both single quotes
    # and double quotes are processed in the same loop, otherwise
    # nested quotes wouldn't work.
    collapsed = ''
    while True:
      # Find the first quote character
      match = Match(r'^([^\'"]*)([\'"])(.*)$', elided)
      if not match:
        collapsed += elided
        break
      head, quote, tail = match.groups()

      if quote == '"':
        # Collapse double quoted strings
        second_quote = tail.find('"')
        if second_quote >= 0:
          collapsed += head + '""'
          elided = tail[second_quote + 1:]
        else:
          # Unmatched double quote, don't bother processing the rest
          # of the line since this is probably a multiline string.
          collapsed += elided
          break
      else:
        # Found single quote, check nearby text to eliminate digit separators.
        #
        # There is no special handling for floating point here, because
        # the integer/fractional/exponent parts would all be parsed
        # correctly as long as there are digits on both sides of the
        # separator.  So we are fine as long as we don't see something
        # like "0.'3" (gcc 4.9.0 will not allow this literal).
        if Search(r'\b(?:0[bBxX]?|[1-9])[0-9a-fA-F]*$', head):
          match_literal = Match(r'^((?:\'?[0-9a-zA-Z_])*)(.*)$', "'" + tail)
          collapsed += head + match_literal.group(1).replace("'", '')
          elided = match_literal.group(2)
        else:
          second_quote = tail.find('\'')
          if second_quote >= 0:
            collapsed += head + "''"
            elided = tail[second_quote + 1:]
          else:
            # Unmatched single quote
            collapsed += elided
            break

    return collapsed


class BlockInfo(object):
  """Stores information about a generic block of code."""

  def __init__(self, seen_open_brace):
    self.seen_open_brace = seen_open_brace
    self.open_parentheses = 0
    self.inline_asm = _NO_ASM
    self.check_namespace_indentation = False

  def CheckBegin(self, filename, clean_lines, linenum, error):
    """Run checks that applies to text up to the opening brace.

    This is mostly for checking the text after the class identifier
    and the "{", usually where the base class is specified.  For other
    blocks, there isn't much to check, so we always pass.

    Args:
      filename: The name of the current file.
      clean_lines: A CleansedLines instance containing the file.
      linenum: The number of the line to check.
      error: The function to call with any errors found.
    """
    pass

  def CheckEnd(self, filename, clean_lines, linenum, error):
    """Run checks that applies to text after the closing brace.

    This is mostly used for checking end of namespace comments.

    Args:
      filename: The name of the current file.
      clean_lines: A CleansedLines instance containing the file.
      linenum: The number of the line to check.
      error: The function to call with any errors found.
    """
    pass

  def IsBlockInfo(self):
    """Returns true if this block is a _BlockInfo.

    This is convenient for verifying that an object is an instance of
    a _BlockInfo, but not an instance of any of the derived classes.

    Returns:
      True for this class, False for derived classes.
    """
    return self.__class__ == BlockInfo


class ExternCInfo(BlockInfo):
  """Stores information about an 'extern "C"' block."""

  def __init__(self):
    BlockInfo.__init__(self, True)


class ClassInfo(BlockInfo):
  """Stores information about a class."""

  def __init__(self, name, class_or_struct, clean_lines, linenum):
    BlockInfo.__init__(self, False)
    self.name = name
    self.starting_linenum = linenum
    self.is_derived = False
    self.check_namespace_indentation = True
    if class_or_struct == 'struct':
      self.access = 'public'
      self.is_struct = True
    else:
      self.access = 'private'
      self.is_struct = False

    # Remember initial indentation level for this class.  Using raw_lines here
    # instead of elided to account for leading comments.
    self.class_indent = GetIndentLevel(clean_lines.raw_lines[linenum])

    # Try to find the end of the class.  This will be confused by things like:
    #   class A {
    #   } *x = { ...
    #
    # But it's still good enough for CheckSectionSpacing.
    self.last_line = 0
    depth = 0
    for i in range(linenum, clean_lines.NumLines()):
      line = clean_lines.elided[i]
      depth += line.count('{') - line.count('}')
      if not depth:
        self.last_line = i
        break

  def CheckBegin(self, filename, clean_lines, linenum, error):
    # Look for a bare ':'
    if Search('(^|[^:]):($|[^:])', clean_lines.elided[linenum]):
      self.is_derived = True

  def CheckEnd(self, filename, clean_lines, linenum, error):
    # If there is a DISALLOW macro, it should appear near the end of
    # the class.
    seen_last_thing_in_class = False
    for i in xrange(linenum - 1, self.starting_linenum, -1):
      match = Search(
          r'\b(DISALLOW_COPY_AND_ASSIGN|DISALLOW_IMPLICIT_CONSTRUCTORS)\(' +
          self.name + r'\)',
          clean_lines.elided[i])
      if match:
        if seen_last_thing_in_class:
          #error(filename, i, 'readability/constructors', 3,
                #match.group(1) + ' should be the last thing in the class')
        break

      if not Match(r'^\s*$', clean_lines.elided[i]):
        seen_last_thing_in_class = True

    # Check that closing brace is aligned with beginning of the class.
    # Only do this if the closing brace is indented by only whitespaces.
    # This means we will not check single-line class definitions.
    indent = Match(r'^( *)\}', clean_lines.elided[linenum])
    if indent and len(indent.group(1)) != self.class_indent:
      if self.is_struct:
        parent = 'struct ' + self.name
      else:
        parent = 'class ' + self.name
      # error(filename, linenum, 'whitespace/indent', 3,
            # 'Closing brace should be aligned with beginning of %s' % parent)


class NamespaceInfo(BlockInfo):
  """Stores information about a namespace."""

  def __init__(self, name, linenum):
    BlockInfo.__init__(self, False)
    self.name = name or ''
    self.starting_linenum = linenum
    self.check_namespace_indentation = True

  def CheckEnd(self, filename, clean_lines, linenum, error):
    """Check end of namespace comments."""
    line = clean_lines.raw_lines[linenum]

    # Check how many lines is enclosed in this namespace.  Don't issue
    # warning for missing namespace comments if there aren't enough
    # lines.  However, do apply checks if there is already an end of
    # namespace comment and it's incorrect.
    #
    # TODO(unknown): We always want to check end of namespace comments
    # if a namespace is large, but sometimes we also want to apply the
    # check if a short namespace contained nontrivial things (something
    # other than forward declarations).  There is currently no logic on
    # deciding what these nontrivial things are, so this check is
    # triggered by namespace size only, which works most of the time.
    if (linenum - self.starting_linenum < 10
        and not Match(r'};*\s*(//|/\*).*\bnamespace\b', line)):
      return

    # Look for matching comment at end of namespace.
    #
    # Note that we accept C style "/* */" comments for terminating
    # namespaces, so that code that terminate namespaces inside
    # preprocessor macros can be cpplint clean.
    #
    # We also accept stuff like "// end of namespace <name>." with the
    # period at the end.
    #
    # Besides these, we don't accept anything else, otherwise we might
    # get false negatives when existing comment is a substring of the
    # expected namespace.
    if self.name:
      # Named namespace
      if not Match((r'};*\s*(//|/\*).*\bnamespace\s+' + re.escape(self.name) +
                    r'[\*/\.\\\s]*$'),
                   line):
        # error(filename, linenum, 'readability/namespace', 5,
              # 'Namespace should be terminated with "// namespace %s"' %
              # self.name)
    else:
      # Anonymous namespace
      if not Match(r'};*\s*(//|/\*).*\bnamespace[\*/\.\\\s]*$', line):
        # If "// namespace anonymous" or "// anonymous namespace (more text)",
        # mention "// anonymous namespace" as an acceptable form
        if Match(r'}.*\b(namespace anonymous|anonymous namespace)\b', line):
          # error(filename, linenum, 'readability/namespace', 5,
                # 'Anonymous namespace should be terminated with "// namespace"'
                # ' or "// anonymous namespace"')
        else:
          # error(filename, linenum, 'readability/namespace', 5,
                # 'Anonymous namespace should be terminated with "// namespace"')


class PreprocessorInfo(object):
  """Stores checkpoints of nesting stacks when #if/#else is seen."""

  def __init__(self, stack_before_if):
    # The entire nesting stack before #if
    self.stack_before_if = stack_before_if

    # The entire nesting stack up to #else
    self.stack_before_else = []

    # Whether we have already seen #else or #elif
    self.seen_else = False




def Match(pattern, s):
  """Matches the string with the pattern, caching the compiled regexp."""
  # The regexp compilation caching is inlined in both Match and Search for
  # performance reasons; factoring it out into a separate function turns out
  # to be noticeably expensive.
  if pattern not in _regexp_compile_cache:
    _regexp_compile_cache[pattern] = sre_compile.compile(pattern)
  return _regexp_compile_cache[pattern].match(s)


def Search(pattern, s):
  """Searches the string for the pattern, caching the compiled regexp."""
  if pattern not in _regexp_compile_cache:
    _regexp_compile_cache[pattern] = sre_compile.compile(pattern)
  return _regexp_compile_cache[pattern].search(s)



def CloseExpression(clean_lines, linenum, pos):
  """If input points to ( or { or [ or <, finds the position that closes it.

  If lines[linenum][pos] points to a '(' or '{' or '[' or '<', finds the
  linenum/pos that correspond to the closing of the expression.

  TODO(unknown): cpplint spends a fair bit of time matching parentheses.
  Ideally we would want to index all opening and closing parentheses once
  and have CloseExpression be just a simple lookup, but due to preprocessor
  tricks, this is not so easy.

  Args:
    clean_lines: A CleansedLines instance containing the file.
    linenum: The number of the line to check.
    pos: A position on the line.

  Returns:
    A tuple (line, linenum, pos) pointer *past* the closing brace, or
    (line, len(lines), -1) if we never find a close.  Note we ignore
    strings and comments when matching; and the line we return is the
    'cleansed' line at linenum.
  """

  line = clean_lines.elided[linenum]
  if (line[pos] not in '({[<') or Match(r'<[<=]', line[pos:]):
    return (line, clean_lines.NumLines(), -1)

  # Check first line
  (end_pos, stack) = FindEndOfExpressionInLine(line, pos, [])
  if end_pos > -1:
    return (line, linenum, end_pos)

  # Continue scanning forward
  while stack and linenum < clean_lines.NumLines() - 1:
    linenum += 1
    line = clean_lines.elided[linenum]
    (end_pos, stack) = FindEndOfExpressionInLine(line, 0, stack)
    if end_pos > -1:
      return (line, linenum, end_pos)

  # Did not find end of expression before end of file, give up
  return (line, clean_lines.NumLines(), -1)


def FindEndOfExpressionInLine(line, startpos, stack):
  """Find the position just after the end of current parenthesized expression.

  Args:
    line: a CleansedLines line.
    startpos: start searching at this position.
    stack: nesting stack at startpos.

  Returns:
    On finding matching end: (index just after matching end, None)
    On finding an unclosed expression: (-1, None)
    Otherwise: (-1, new stack at end of this line)
  """
  for i in xrange(startpos, len(line)):
    char = line[i]
    if char in '([{':
      # Found start of parenthesized expression, push to expression stack
      stack.append(char)
    elif char == '<':
      # Found potential start of template argument list
      if i > 0 and line[i - 1] == '<':
        # Left shift operator
        if stack and stack[-1] == '<':
          stack.pop()
          if not stack:
            return (-1, None)
      elif i > 0 and Search(r'\boperator\s*$', line[0:i]):
        # operator<, don't add to stack
        continue
      else:
        # Tentative start of template argument list
        stack.append('<')
    elif char in ')]}':
      # Found end of parenthesized expression.
      #
      # If we are currently expecting a matching '>', the pending '<'
      # must have been an operator.  Remove them from expression stack.
      while stack and stack[-1] == '<':
        stack.pop()
      if not stack:
        return (-1, None)
      if ((stack[-1] == '(' and char == ')') or
          (stack[-1] == '[' and char == ']') or
          (stack[-1] == '{' and char == '}')):
        stack.pop()
        if not stack:
          return (i + 1, None)
      else:
        # Mismatched parentheses
        return (-1, None)
    elif char == '>':
      # Found potential end of template argument list.

      # Ignore "->" and operator functions
      if (i > 0 and
          (line[i - 1] == '-' or Search(r'\boperator\s*$', line[0:i - 1]))):
        continue

      # Pop the stack if there is a matching '<'.  Otherwise, ignore
      # this '>' since it must be an operator.
      if stack:
        if stack[-1] == '<':
          stack.pop()
          if not stack:
            return (i + 1, None)
    elif char == ';':
      # Found something that look like end of statements.  If we are currently
      # expecting a '>', the matching '<' must have been an operator, since
      # template argument list should not contain statements.
      while stack and stack[-1] == '<':
        stack.pop()
      if not stack:
        return (-1, None)

  # Did not find end of expression or unbalanced parentheses on this line
  return (-1, stack)


def CleanseRawStrings(raw_lines):
  """Removes C++11 raw strings from lines.

    Before:
      static const char kData[] = R"(
          multi-line string
          )";

    After:
      static const char kData[] = ""
          (replaced by blank line)
          "";

  Args:
    raw_lines: list of raw lines.

  Returns:
    list of lines with C++11 raw strings replaced by empty strings.
  """

  delimiter = None
  lines_without_raw_strings = []
  for line in raw_lines:
    if delimiter:
      # Inside a raw string, look for the end
      end = line.find(delimiter)
      if end >= 0:
        # Found the end of the string, match leading space for this
        # line and resume copying the original lines, and also insert
        # a "" on the last line.
        leading_space = Match(r'^(\s*)\S', line)
        line = leading_space.group(1) + '""' + line[end + len(delimiter):]
        delimiter = None
      else:
        # Haven't found the end yet, append a blank line.
        line = '""'

    # Look for beginning of a raw string, and replace them with
    # empty strings.  This is done in a loop to handle multiple raw
    # strings on the same line.
    while delimiter is None:
      # Look for beginning of a raw string.
      # See 2.14.15 [lex.string] for syntax.
      matched = Match(r'^(.*)\b(?:R|u8R|uR|UR|LR)"([^\s\\()]*)\((.*)$', line)
      if matched:
        delimiter = ')' + matched.group(2) + '"'

        end = matched.group(3).find(delimiter)
        if end >= 0:
          # Raw string ended on same line
          line = (matched.group(1) + '""' +
                  matched.group(3)[end + len(delimiter):])
          delimiter = None
        else:
          # Start of a multi-line raw string
          line = matched.group(1) + '""'
      else:
        break

    lines_without_raw_strings.append(line)

  # TODO(unknown): if delimiter is not None here, we might want to
  # emit a warning for unterminated string.
  return lines_without_raw_strings



def CleanseComments(line):
  """Removes //-comments and single-line C-style /* */ comments.

  Args:
    line: A line of C++ source.

  Returns:
    The line with single-line comments removed.
  """
  commentpos = line.find('//')
  if commentpos != -1 and not IsCppString(line[:commentpos]):
    line = line[:commentpos].rstrip()
  # get rid of /* ... */
  return _RE_PATTERN_CLEANSE_LINE_C_COMMENTS.sub('', line)


def IsCppString(line):
  """Does line terminate so, that the next symbol is in string constant.

  This function does not consider single-line nor multi-line comments.

  Args:
    line: is a partial line of code starting from the 0..n.

  Returns:
    True, if next character appended to 'line' is inside a
    string constant.
  """

  line = line.replace(r'\\', 'XX')  # after this, \\" does not match to \"
  return ((line.count('"') - line.count(r'\"') - line.count("'\"'")) & 1) == 1



if __name__ == "__main__":
    plugin_run("running")
    sys.exit(0)

