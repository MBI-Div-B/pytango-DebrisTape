from .DebrisTape import DebrisTape


def main():
    import sys
    import tango.server

    args = ["DebrisTape"] + sys.argv[1:]
    tango.server.run((DebrisTape,), args=args)
