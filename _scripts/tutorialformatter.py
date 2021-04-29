"""
    tutorialformatter
    ===========================

    This extension provides a directive to include a source code file
    in a document, but with certain comments from the file formatted
    as regular document text.  This allows code for a tutorial to look like:

        /// BEGIN_TUTORIAL
        /// This next line adds one.
        i = i + 1;
        /// Then we need to double it.
        i = i * 2;
        /// END_TUTORIAL

    And have it formatted as

    This next line adds one.::
        i = i + 1;

    Then we need to double it.::
        i = i * 2;

    The special-looking comment character sequence at the start of
    each text line can be anything not starting or ending with
    whitespace.  tutorialformatter starts by scanning the file for the
    string BEGIN_TUTORIAL.  When it finds it, it takes all the
    characters before BEGIN_TUTORIAL on that line, strips whitespace
    from the left, and uses that as the text marker.  So this would
    also be fine:

        #My Tutorial# BEGIN_TUTORIAL
        #My Tutorial# This next line adds one.
        i = i + 1
        #My Tutorial# Then we need to double it.
        i = i * 2
        #My Tutorial# END_TUTORIAL

    Sometimes the order that makes sense in the tutorial is not
    compatible with the computer language of the code, like when a
    callback function in C++ is defined outside of the main tutorial
    code.  To support this, you can use the tags BEGIN_SUB_TUTORIAL,
    END_SUB_TUTORIAL, and CALL_SUB_TUTORIAL.  They look like this:

        # BEGIN_SUB_TUTORIAL callbackFunction
        def callback():
            print("in callback")
        # END_SUB_TUTORIAL

        # BEGIN_TUTORIAL
        # Here we call a special callback:
        callback()
        # which is defined as:
        # CALL_SUB_TUTORIAL callbackFunction
        # and then we move on to the next topic.

    Both the BEGIN_SUB_TUTORIAL and CALL_SUB_TUTORIAL tags take an
    argument, which is the name of the "sub-tutorial".  That name does
    not need to correspond to anything in the code.  Sub-tutorials
    cannot be nested, and they only work within a single source file
    processed by tutorialformatter.  They have no outside meaning.
    The implementation simply slices out sub-tutorials from the input
    lines and copies them into the output lines where-ever the
    corresponding "call" tags are found.

    .. moduleauthor::  Dave Hershberger <hersh@willowgarage.com>
"""

# 0.1.0: First version.
# 0.1.1: fixed a bug in source file directory lookup: now source paths are
#        relative to the directory in which the including document lives.
# 0.1.2: Added SUB_TUTORIAL support.

from __future__ import print_function

__version__ = "0.1.2"

import os
from docutils.parsers import rst
from docutils.parsers.rst.directives import flag, unchanged
from docutils.statemachine import string2lines
from pygments.lexers import get_lexer_for_filename


class TutorialFormatterDirective(rst.Directive):
    has_content = False
    final_argument_whitespace = True
    required_arguments = 1

    option_spec = dict(
        shell=flag,
        prompt=flag,
        nostderr=flag,
        in_srcdir=flag,
        extraargs=unchanged,
        until=unchanged,
    )

    def flatten_sub_tutorials(self, file_):
        lines = []
        in_sub = False
        begin_sub_tutorial = "BEGIN_SUB_TUTORIAL"
        end_sub_tutorial = "END_SUB_TUTORIAL"
        call_sub_tutorial = "CALL_SUB_TUTORIAL"
        sub_name = ""
        subs = {}
        sub_lines = []
        regular_lines = []
        for line in file_:
            begin_pos = line.find(begin_sub_tutorial)
            if begin_pos != -1:
                sub_name = line[begin_pos + len(begin_sub_tutorial) :].strip()
                in_sub = True
            elif line.find(end_sub_tutorial) != -1 and in_sub:
                in_sub = False
                subs[sub_name] = sub_lines
                sub_lines = []
            elif in_sub:
                sub_lines.append(line)
            else:
                regular_lines.append(line)
        flattened_lines = []
        for line in regular_lines:
            call_pos = line.find(call_sub_tutorial)
            if call_pos != -1:
                sub_name = line[call_pos + len(call_sub_tutorial) :].strip()
                if sub_name in subs:
                    flattened_lines.extend(subs[sub_name])
                else:
                    print(
                        "tutorialformatter.py error: sub-tutorial %s not found."
                        % sub_name
                    )
            else:
                flattened_lines.append(line)
        return flattened_lines

    def run(self):
        filename = self.arguments[0]
        text_tag = None
        tag_len = 0

        filepath = os.path.dirname(self.state.document.settings.env.docname)
        absfilename = os.path.abspath(os.path.join(filepath, filename))
        if absfilename.endswith(".h"):
            language = "c++"
        elif absfilename.endswith("CMakeLists.txt"):
            language = "cmake"
        else:
            try:
                language = get_lexer_for_filename(absfilename).name.lower()
                if language == "text only":
                    language = "none"
            except:
                language = "none"
        code_prefix = "\n.. code-block:: " + language + "\n\n"
        code_suffix = "\n"

        print("tutorial-formatter running on " + absfilename)
        file_ = open(absfilename, "r")
        text_to_process = ""
        current_block = ""
        in_code = False
        in_text = False
        in_tutorial = False
        lines = self.flatten_sub_tutorials(file_)
        for line in lines:
            if not in_tutorial:
                begin_pos = line.find("BEGIN_TUTORIAL")
                if begin_pos != -1:
                    text_tag = line[:begin_pos].lstrip()
                    tag_len = len(text_tag)
                    in_tutorial = True
                continue
            if line.find("END_TUTORIAL") != -1:
                break
            stripped = line.lstrip()
            if stripped.startswith(text_tag.strip()):
                if in_code:
                    text_to_process += code_prefix + current_block + code_suffix
                    current_block = ""
                    in_code = False
                in_text = True
                addition = stripped[tag_len:]
                if addition == "" or addition[-1] != "\n":
                    addition += "\n"
                current_block += addition
            else:
                if in_text:
                    text_to_process += current_block
                    current_block = ""
                    in_text = False
                    in_code = True  # Code to show begins right after tagged text
                if in_code:
                    current_block += " " + line
        if in_code:
            text_to_process += code_prefix + current_block + code_suffix
        elif in_text:
            text_to_process += current_block

        # Debug writes...
        # print('text_to_process =')
        # print(text_to_process)
        # print('= text_to_process')

        lines = string2lines(text_to_process)
        self.state_machine.insert_input(lines, absfilename)

        return []


def setup(app):
    app.add_directive("tutorial-formatter", TutorialFormatterDirective)
