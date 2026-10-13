import sys

from . import main

if __name__ == "__main__":       # spawn re-imports __main__ in worker processes
    sys.exit(main())
