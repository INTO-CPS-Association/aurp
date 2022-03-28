import io
from contextlib import redirect_stdout
from io import StringIO
import sys
from odrive.utils import dump_errors
import re

class RedirectedStdout:
    def __init__(self):
        self._stdout = None
        self._string_io = None

    def __enter__(self):
        self._stdout = sys.stdout
        sys.stdout = self._string_io = StringIO()
        return self

    def __exit__(self, type, value, traceback):
        sys.stdout = self._stdout

    def __str__(self):
        return self._string_io.getvalue()

def has_error_stdout(odrv):
    

    with RedirectedStdout() as error_msg:
        dump_errors(odrv)

        msg = error_msg._string_io.getvalue()
    
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    msg = ansi_escape.sub('', msg)

    return "Error(s)" in msg


def has_errors(odrv):
    return odrv.axis0.motor.error != 0 or odrv.axis0.encoder.error != 0 or odrv.axis0.error != 0 or odrv.error != 0

def assert_no_errors(odrv, tag):
    if has_errors(odrv):
        print(f"An error has occurred at {tag}")
        dump_errors(odrv)
        exit(-1)