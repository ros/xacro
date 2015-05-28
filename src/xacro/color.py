from __future__ import print_function
import sys

# bold colors
_ansi = {'red': 91, 'yellow': 93}

def is_tty(stream): # taken from catkin_tools/common.py
    """Returns True if the given stream is a tty, else False"""
    return hasattr(stream, 'isatty') and stream.isatty()


def message(msg, *args, **kwargs):
    file  = kwargs.get('file', sys.stderr)
    alt_text = kwargs.get('alt_text', None)
    color = kwargs.get('color', None)

    if color and is_tty(file):
        msg = '\033[%dm%s\033[0m' % (_ansi[color], msg)
    elif alt_text:
        msg = '%s%s' % (alt_text, msg)
    print(msg, *args, file=file)


def warning(*args, **kwargs):
    defaults = dict(file=sys.stderr, alt_text='warning: ', color='yellow')
    defaults.update(kwargs)
    message(*args, **defaults)


def error(*args, **kwargs):
    defaults = dict(file=sys.stderr, alt_text='error: ', color='red')
    defaults.update(kwargs)
    message(*args, **defaults)
