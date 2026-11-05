"""Compatibility entry point. See `python -m workflow.cli --help` for commands.

The old placeholder flags have been replaced by explicit demo/evaluate/run modes.
"""

from srdrone.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
