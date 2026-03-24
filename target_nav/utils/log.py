"""Centralized logging configuration for the target navigation system.

Provides ``get_logger()`` for per-module logger creation and
``setup_logging()`` for one-time root logger configuration.  All modules
in the package use ``get_logger(__name__)`` (or a short label) so log
output is consistently formatted and filterable by component.

Architecture:
    ``setup_logging()`` is called once from the main entry point
    (``app/main.py``).  Every other module calls ``get_logger(name)`` at
    import time.  In 3-process mode each child process (Nav, Camera)
    re-inherits the root logger configuration from spawn.

Key functions:
    get_logger    -- return a named ``logging.Logger``.
    setup_logging -- configure the root logger format and level.

Thread safety:
    ``logging`` module is thread-safe by design.  No additional locking.
"""

import logging
import sys


def get_logger(name: str) -> logging.Logger:
    """Return a named logger for the given module or component.

    Args:
        name: Dotted logger name (e.g. 'state.persistence', 'nav.navigator').

    Returns:
        A ``logging.Logger`` instance.
    """
    return logging.getLogger(name)


def setup_logging(level: str = "INFO") -> None:
    """Configure the root logger with a standard format.

    Should be called exactly once from the main entry point.  Sets up
    ``HH:MM:SS [name] LEVEL: message`` format on stdout.

    Args:
        level: Logging level name (e.g. 'DEBUG', 'INFO', 'WARNING').
    """
    logging.basicConfig(
        level=getattr(logging, level, logging.INFO),
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%H:%M:%S',
        stream=sys.stdout,
    )
