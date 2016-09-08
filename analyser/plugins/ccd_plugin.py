#!/usr/bin/python

#Copyright (c) 2013 Dominik Borowiec, Andre Santos
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

#=============================================================================
#                         *** CONFIGURATION ***
#=============================================================================
# THIS CONFIGURATION APLLIES FOR C++, BUT IT CAN BE TUNED FOR EACH 
# PROGRAMMING LANGUAGE

# Reserved words and operators for C++
# Note that pure operators can interleave with reserved words 
RESERVED_WORDS = [
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor",
    "bool", "break", "case", "catch", "char", "char16_t", "char32_t", "class",
    "compl", "const", "constexpr", "const_cast", "continue", "decltype",
    "default", "delete", "do", "double", "dynamic_cast", "else", "enum",
    "explicit", "export", "extern", "false", "float", "for", "friend", "goto",
    "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept",
    "not", "not_eq", "nullptr", "operator", "or", "or_eq", "private",
    "protected", "public", "register", "reinterpret_cast", "return", "short",
    "signed", "sizeof", "static", "static_assert", "static_cast", "struct",
    "switch", "template", "this", "thread_local", "throw", "true", "try",
    "typedef", "typeid", "typename", "union", "unsigned", "using", "virtual",
    "void", "volatile", "wchar_t", "while", "xor", "xor_eq"
    ]
    
# Note that the order of the list PURE_OPERATORS is important, because
# because matching is being done from the begining of the list to the end.
PURE_OPERATORS = [
    "::", "++", "--", "(", ")", "[", "]", ".", "->", "++", "--", "+", "-", 
    "!", "~", "*", "&", ".*", "->*", "*", "/", "%", "+", "-", "<<", ">>", 
    "<", "<=", ">", ">=", "==", "!=", "&", "^", "|", "&&", "||", "?:", "=", 
    "+=", "-=", "*=", "/=", "%=", "<<=", ">>=", "&=", "^=", "|=", ",", "\"", 
    "\'", "\\", ";", "{", "}"
    ]

# Comment tokens
ONELINE_COMMENT_TOKEN = "//"
MULTILINE_COMMENT_TOKEN_BEGIN = "/*"
MULTILINE_COMMENT_TOKEN_END = "*/"

# Detector tresholds
# Expected differences (in %) of the metrics computed for snippet with 
# the content of the comment joined with its context should compared with
# the values of metrics for the context only.
# At least given number of tresholds should be applied succesfully to
# state that the comment may contain code. See the code of the script :)

COMMENTED_CODE_MAX_TRESHOLDS = [
    25, #Operators count
    5,  #Distinct operators
    25, #Operands count
    5,  #Distinct operand
    20, #Program length
    5,  #Program vocabulary
    25, #Volume
    20, #Difficulty
    40, #Effort     
]
COMMENTED_CODE_MAX_TRESHOLDS_EXPECTED_PASSES = 7

COMMENTED_CODE_MIN_TRESHOLDS = [
    5, #Operators count
    0, #Distinct operators
    0, #Operands count
    0, #Distinct operand
    0, #Program length
    0, #Program vocabulary
    0, #Volume
    0, #Difficulty
    0, #Effort     
]
COMMENTED_CODE_MIN_TRESHOLDS_EXPECTED_PASSES = 9

#=============================================================================
#                                *** SCRIPT ***
#=============================================================================

import argparse
import itertools
import math
import os


idGen = 1


class Comment:
    def __init__(self):
        self.content = ""
        self.firstLineNumber = -1
        self.lastLineNumber = -1    
        
        
    def addContent(self, appendedContent, lineNumber):
        self.content = " ".join([self.content, appendedContent])
        self.lastLineNumber = lineNumber
        if self.firstLineNumber == -1:
            self.firstLineNumber = lineNumber
        
        
    def isEmpty(self):
        return self.firstLineNumber == -1
        
        
    def getContent(self):
        return self.content
        
        
    def getFirstLineNumber(self):
        return self.firstLineNumber
        
        
    def getLastLineNumber(self):
        return self.lastLineNumber
        
        
    def getLength(self):
        return self.getLastLineNumber() - self.getFirstLineNumber() + 1
        
        
    def newLine(self):
        self.content += "\n"
        
        
    def __str__(self):
        return "Comment from lines %d-%d:\n%s" %(self.getFirstLineNumber(),
            self.getLastLineNumber(), self.getContent())

            
            
class Halstead:
    def __init__(self, source):
        self.operandsCnt = 0
        self.operatorsCnt = 0
        self.operands = set()
        self.operators = set()
        
        for line in source:
            self._analyzeLine(line)

            
    def _analyzeLine(self, line):
        for lexem in line.split():
            self._analyzeLexem(lexem)

        
    def _analyzeLexem(self, lexem):
        reduct = lexem
        while reduct:
            reduct = self._reduceLexem(reduct)
          
          
    def _reduceLexem(self, lexem):
        nonPureShortestPrefixLen = len(lexem)
        for operator in PURE_OPERATORS:
            if lexem.startswith(operator):
                self.operatorsCnt += 1
                self.operators.add(operator)
                return lexem[len(operator):]
            if operator in lexem:
                nonPureShortestPrefixLen = min(
                    nonPureShortestPrefixLen, 
                    lexem.find(operator)
                )
        nonPureShortestPrefix = lexem[:nonPureShortestPrefixLen]
        for keyword in RESERVED_WORDS:
            if nonPureShortestPrefix == keyword:
                self.operatorsCnt += 1
                self.operators.add(keyword)
                return lexem[nonPureShortestPrefixLen:]
        self.operandsCnt += 1
        self.operands.add(nonPureShortestPrefix)
        return lexem[nonPureShortestPrefixLen:]
    
    
    def getDistinctOperatorsCnt(self):
        return len(self.operators)
    
    
    def getDistinctOperandsCnt(self):
        return len(self.operands)
    
    
    def getTotalOperatorsCnt(self):
        return self.operatorsCnt
    
    
    def getTotalOparandsCnt(self):
        return self.operandsCnt
    
    
    def getLength(self):
        return self.getTotalOperatorsCnt() + self.getTotalOparandsCnt()
    
    
    def getVocabulary(self):
        return self.getDistinctOperatorsCnt() + self.getDistinctOperandsCnt()

        
    def getVolume(self):
        return self.getLength() * math.log(unzero(self.getVocabulary()), 2)
    
    
    def getDifficulty(self):
        return (self.getDistinctOperatorsCnt() / 2 * 
            self.getTotalOparandsCnt() / unzero(
                self.getDistinctOperandsCnt()))
    
    
    def getEffort(self):
        return self.getDifficulty() * self.getVolume()
    
    
    def getValuesVector(self):
        return [
            self.getTotalOperatorsCnt(), 
            self.getDistinctOperatorsCnt(),
            self.getTotalOparandsCnt(),
            self.getDistinctOperandsCnt(),
            self.getLength(),
            self.getVocabulary(),
            self.getVolume(),
            self.getDifficulty(),
            self.getEffort()
        ]
    
    
    @staticmethod
    def printStatistics(valuesVectors, headers=None):
        names = ["Operators count:", "Distinct operators:", "Operands count:",
        "Distinct operands:", "Program length:", "Program vocabulary:",
        "Volume:", "Difficulty:", "Effort"]
        
        if headers:
            output = "".ljust(22)
            for header in headers:
                output += header.ljust(10)
            print output
        
        for i in xrange(len(names)):
            output=names[i].ljust(22)
            for vector in valuesVectors:                
                output += ("%.2f" %(vector[i])).ljust(10)
            print output
        

        
class CommentFilter:
    def filterComments(self, source):
        """ 
            @input: list of lines of file
            @return: tuple containing list of lines of file and list of 
                Comments
        """
        self.regularLines = []
        self.comments = []
        self.inMultilineComment = False
        self.currentComment = Comment()
        self.lineNumber = 0
    
        for line in source:
            self.currentLine = []        
            while line:
                line = self.reduceLine(line)
            self.regularLines.append(" ".join(self.currentLine))
            self.lineNumber += 1
            if not self.currentComment.isEmpty():
                self.currentComment.newLine()
        
        if not self.currentComment.isEmpty():
            self.comments.append(self.currentComment)
            self.currentComment = Comment()
        
        return (self.regularLines, self.comments)
    
    
    def reduceLine(self, line):
        notInLine = 999999
        multiLineBeginPosition = notInLine
        multiLineEndPosition = notInLine
        oneLinePosition = notInLine
        
        if MULTILINE_COMMENT_TOKEN_BEGIN in line:
            multiLineBeginPosition = line.find(MULTILINE_COMMENT_TOKEN_BEGIN)
            
        if MULTILINE_COMMENT_TOKEN_END in line:
            multiLineEndPosition = line.find(MULTILINE_COMMENT_TOKEN_END)
            
        if ONELINE_COMMENT_TOKEN in line:
            oneLinePosition = line.find(ONELINE_COMMENT_TOKEN)
        
        if (not self.inMultilineComment 
                and oneLinePosition < multiLineBeginPosition):
            self.currentLine.append(line[:oneLinePosition])
            self.currentComment.addContent(
                line[oneLinePosition + len(ONELINE_COMMENT_TOKEN):],
                self.lineNumber    
            )            
        elif self.inMultilineComment and multiLineEndPosition != notInLine:
            self.currentComment.addContent(
                line[:multiLineEndPosition],
                self.lineNumber
            )
            self.inMultilineComment = False
            return (line[multiLineEndPosition + 
                len(MULTILINE_COMMENT_TOKEN_END):])
        elif multiLineBeginPosition != notInLine:
            self.currentLine.append(line[:multiLineBeginPosition])
            self.inMultilineComment = True
            return (line[multiLineBeginPosition + 
                len(MULTILINE_COMMENT_TOKEN_BEGIN):])
        elif self.inMultilineComment:
            self.currentComment.addContent(line, self.lineNumber)
        else:
            if not self.currentComment.isEmpty():
                self.comments.append(self.currentComment)
                self.currentComment = Comment()
            self.currentLine.append(line)
        return ""

            
def parseArgs():
    parser = argparse.ArgumentParser(description="Discover commented code "
    "using Halstead code metrics. "
    "Metrics are computed on the given number of context "
    "lines for each comment. "
    "If the value computed on the context lines themselves and for "
    "the context lines merged with the content of the comment are similar, "
    "the comment may "
    "consist of commented code. "
    "The script (after modifying "
    "the dictionary of operators) may be used with every "
    "programming language." )
    parser.add_argument(
        "sourcefile",
        metavar="FILE", 
        nargs=1, 
        help="File to be processed."
    )
    parser.add_argument(
        "-v", 
        "--verbose", 
        dest="verbose",
        metavar="N",
        type=int,
        default=0,
        help="How many additional information should be printed, " +
            "accepted levels 0-3."
    )
    parser.add_argument(
        "-fm", 
        "--show-file-metrics", 
        dest="showFileMetrics", 
        action='store_true',  
        help="Print metrics for the full file (comments removed)."
    )
    parser.add_argument(
        "-c", 
        "--context-multiplier",
        default=5,
        metavar="N",
        dest="contextMultiplier", 
        type=int,
        help="How much bigger a context should be than a comment itself."
    )
    parser.add_argument(
        "-m", 
        "--min-context",
        default=8,
        metavar="N",
        dest="minContext", 
        type=int,
        help="Minimal length of context for a comment."
    )    
    return vars(parser.parse_args())

    
def analyzeComment(comment, regularLines, args):
    contextLength = max(
        args["minContext"], 
        args["contextMultiplier"] * comment.getLength()
    )
    linesBefore = regularLines[
        max(0, comment.getFirstLineNumber() - contextLength):
        comment.getFirstLineNumber()
    ]
    linesAfter = regularLines[
        comment.getLastLineNumber():
        comment.getLastLineNumber() + contextLength
    ]
    linesCnt = len(linesBefore) + len(linesAfter) + comment.getLength()
    
    codeNoCommentMetrics = Halstead(linesBefore + linesAfter)
    codeWithCommentMetrics =( Halstead(linesBefore + linesAfter + 
        [comment.getContent()]))
    noCommentPerLineValues = [(1.00 * v) / linesCnt 
        for v in codeNoCommentMetrics.getValuesVector()]       
    withCommentPerLineValues = [(1.00 * v) / linesCnt 
        for v in codeWithCommentMetrics.getValuesVector()]

    diffValues = [abs(w - n) / unzero(n) * 100 for 
        (n, w) in zip(noCommentPerLineValues, withCommentPerLineValues)
    ]
        
    maxTresholdHits = [d <= t for (d, t) in 
        zip (diffValues, COMMENTED_CODE_MAX_TRESHOLDS)]
    totalMaxTresholdHit = (sum(x > 0 for x in maxTresholdHits) >= 
        COMMENTED_CODE_MAX_TRESHOLDS_EXPECTED_PASSES)
    minTresholdHits = [d >= t for (d, t) in 
        zip (diffValues, COMMENTED_CODE_MIN_TRESHOLDS)]
    totalMinTresholdHit = (sum(x > 0 for x in minTresholdHits) >= 
        COMMENTED_CODE_MIN_TRESHOLDS_EXPECTED_PASSES)       
    hit = totalMaxTresholdHit and totalMinTresholdHit
        
    if (args["verbose"] >= 1 and hit) or (args["verbose"] == 3):
        print comment
        if (args["verbose"] >= 2):
            print "Analyzed context lines count: %s." % contextLength
            Halstead.printStatistics([
                codeNoCommentMetrics.getValuesVector(),
                codeWithCommentMetrics.getValuesVector(),
                noCommentPerLineValues,
                withCommentPerLineValues,
                diffValues,
                maxTresholdHits
            ], ["-Cmt", "+Cmt", "-Cmt/l", "+Cmt/l", "diff%", "<maxT?"])
    if hit:
        print ("Lines %s-%s seems to be commented code." 
            %(comment.getFirstLineNumber(), comment.getLastLineNumber()))
    if (args["verbose"] >= 1 and hit) or (args["verbose"] == 3):
        print ""
        print "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"

            
def unzero(v):
    if v == 0:
        return 0.00000001
    else:
        return v            
    
    
def main():
    args = parseArgs()
    regularLines = None
    comments = None
    
    with open(args["sourcefile"][0], 'r') as f:
        source = f.read().splitlines()
        (regularLines, comments) = CommentFilter().filterComments(source)
    
    fullFileMetrics = Halstead(regularLines)
    if args["showFileMetrics"]:
        Halstead.printStatistics([fullFileMetrics.getValuesVector()])
    
    for comment in comments:
        analyzeComment(comment, regularLines, args)

if __name__ == "__main__":
    main()


def plugin_run(ctx):
    files = get_files(ctx)
    handlers = get_metric_handlers()
    for f in files:
        file_path = ctx.getPath(f[2], file_name = f[1])
        process_metrics(ctx, f[0], file_path, f[3], handlers)


def process_metrics(ctx, file_id, file_path, package_id, handlers):
    with open(file_path, "r") as f:
        source = f.read().splitlines()
        (regularLines, comments) = CommentFilter().filterComments(source)
    
    metrics = Halstead(regularLines)
    handlers["hvol"](ctx, package_id, file_id, metrics.getVolume(), file_path)


def get_files(ctx):
    cpp = ctx.getFileInfo(ext="cpp")
    cc  = ctx.getFileInfo(ext="cc")
    cxx = ctx.getFileInfo(ext="cxx")
    c   = ctx.getFileInfo(ext="c")
    h   = ctx.getFileInfo(ext="h")
    hpp = ctx.getFileInfo(ext="hpp")
    hxx = ctx.getFileInfo(ext="hxx")
    return itertools.chain(h, hpp, hxx, c, cc, cpp, cxx)


def get_metric_handlers():
    return {
        "hvol": handle_hvol
    }

def handle_hvol(ctx, package_id, file_id, value, file_path):
    ctx.writeFileMetric(file_id, 15, value)

