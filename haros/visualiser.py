
#Copyright (c) 2016 Andre Santos
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

from __future__ import print_function
from __future__ import unicode_literals
from future import standard_library
standard_library.install_aliases()
from builtins import input

import logging
import os
import subprocess
import threading

from distutils.dir_util import copy_tree
from pkg_resources import Requirement, resource_filename
from shutil import rmtree

import http.server
from http.server import HTTPServer
from http.server import SimpleHTTPRequestHandler

# Known online replacements for js libraries used by the report.
js_lib_replacements = {
    'backbone.min.js'       : 'https://backbonejs.org/backbone-min.js',
    'd3.min.js'             : 'https://d3js.org/d3.v4.min.js',
    'dagre.min.js'          : 'https://dagrejs.github.io/project/dagre/latest/dagre.min.js',
    'jquery-2.2.1.min.js'   : 'https://ajax.googleapis.com/ajax/libs/jquery/2.2.1/jquery.min.js',
    'underscore.min.js'     : 'https://underscorejs.org/underscore-min.js',
}

class BaseHTTPRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_my_headers()
        SimpleHTTPRequestHandler.end_headers(self)

    def send_my_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")


_log = logging.getLogger(__name__)


def install(dst, source_runner, force = False, minimal_output = False):
    if force and os.path.exists(dst):
        rmtree(dst)
    if not os.path.exists(dst):
        _log.info("Creating %s", dst)
        os.mkdir(dst)
    _log.info("Copying viz files.")
    if source_runner:
        src = os.path.abspath(os.path.join(os.path.dirname(__file__),
                              "..", "harosviz"))
    else:
        src = resource_filename(Requirement.parse("haros"), "harosviz")
    copy_tree(src, dst)

    if minimal_output:
        # remove external js libraries that we can link to from report
        index_html_path = os.path.join(dst, 'index.html')
        with open(index_html_path, 'r') as f:
            index_html = f.read()
        for js_lib, replacement in js_lib_replacements.items():
            if js_lib in index_html:
                index_html = index_html.replace('lib/js/'+js_lib, replacement)
                os.remove(os.path.join(dst,"lib", "js", js_lib))
        with open(index_html_path, 'w') as f:
            f.write(index_html)
    #

    data_dir = os.path.join(dst, "data")
    if not os.path.exists(data_dir):
        _log.info("Creating %s", data_dir)
        os.mkdir(data_dir)


def serve(directory, host_str, headless = False):
    host = host_str.split(":")
    if len(host) != 2:
        raise RuntimeError("Invalid host:port provided: " + host_str)
    server = None
    p = None
    wd = os.getcwd()
    try:
        os.chdir(directory)
        server = HTTPServer((host[0], int(host[1])), BaseHTTPRequestHandler)
        print("[HAROS] Serving visualisation at", host_str)
        if not headless:            
            thread = threading.Thread(target = server.serve_forever)
            thread.daemon = True
            thread.start()
            _log.info("Starting web browser process.")
            cmd = ["python", "-m", "webbrowser", "-t", "http://" + host_str]
            with open(os.devnull, "wb") as devnull:
                p = subprocess.Popen(cmd, stdout = devnull,
                                     stderr = subprocess.STDOUT)
                input("[HAROS] Press enter to shutdown the viz server:")
        else:
            server.serve_forever()
        return True
    except ValueError as e:
        _log.error("Invalid port for the viz server %s", host[1])
        return False
    finally:
        if server:
            server.shutdown()
        os.chdir(wd)
        if p:
            _log.debug("Killing web browser process.")
            p.kill()
