
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

import logging
import os
import subprocess
import threading

from distutils.dir_util import copy_tree
from pkg_resources import Requirement, resource_filename
from shutil import rmtree

try: 
    # Python 3
    from http.server import HTTPServer, BaseHTTPRequestHandler
except ImportError: 
    # Python 2
    import SimpleHTTPServer
    from BaseHTTPServer import HTTPServer
    from SimpleHTTPServer import SimpleHTTPRequestHandler

    class BaseHTTPRequestHandler(SimpleHTTPRequestHandler):
        def end_headers(self):
            self.send_my_headers()
            SimpleHTTPRequestHandler.end_headers(self)

        def send_my_headers(self):
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")


_log = logging.getLogger(__name__)


def install(dst):
    if os.path.exists(dst):
        rmtree(dst)
    if not os.path.exists(dst):
        data_dir = os.path.join(dst, "data")
        _log.info("Creating %s", dst)
        os.mkdir(dst)
        _log.info("Copying viz files.")
        src = resource_filename(Requirement.parse("haros"), "viz")
        copy_tree(src, dst)
        _log.info("Creating %s", data_dir)
        os.mkdir(data_dir)
        _log.info("Creating %s", os.path.join(data_dir, "compliance"))
        os.mkdir(os.path.join(data_dir, "compliance"))
        _log.info("Creating %s", os.path.join(data_dir, "metrics"))
        os.mkdir(os.path.join(data_dir, "metrics"))


def serve(directory, host_str):
    host = host_str.split(":")
    if len(host) != 2:
        raise RuntimeError("Invalid host:port provided: " + host_str)
    wd = os.getcwd()
    try:
        os.chdir(directory)
        server = HTTPServer((host[0], int(host[1])), BaseHTTPRequestHandler)
        print "[HAROS] Serving visualisation at", host_str
        thread = threading.Thread(target = server.serve_forever)
        thread.deamon = True
        thread.start()
        _log.info("Starting web browser process.")
        p = subprocess.Popen(["python", "-m", "webbrowser",
                        "-t", "http://" + host_str])
        raw_input("[HAROS] Press enter to shutdown the viz server:")
        return True
    except ValueError as e:
        _log.error("Invalid port for the viz server %s", host[1])
        return False
    finally:
        server.shutdown()
        os.chdir(wd)
        _log.debug("Killing web browser process.")
        p.kill()
