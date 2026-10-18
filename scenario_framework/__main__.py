import signal
import sys
from .runner import main, interrupt

signal.signal(signal.SIGTERM, interrupt)
sys.exit(main())
