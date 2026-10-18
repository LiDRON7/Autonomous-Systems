#!/usr/bin/env python3
"""Compatibility alias for python -m scenario_framework run SCENARIO."""
from pathlib import Path
import signal
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scenario_framework.runner import main, interrupt

if __name__ == '__main__':
    signal.signal(signal.SIGTERM, interrupt)
    sys.exit(main(['run', *sys.argv[1:]]))
