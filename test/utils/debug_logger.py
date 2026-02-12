#!/usr/bin/env python3
"""
Debug Logger for V2N Robot Tests
=================================

Comprehensive logging utilities for hardware and integration tests.
Provides colorized output, timing, and structured test results.

Usage:
    from test.utils import DebugLogger, TestResult

    logger = DebugLogger("ArduinoTest")
    logger.info("Starting test...")
    logger.debug("Detailed info here")
    logger.success("Test passed!")
    logger.error("Test failed!")
"""

import sys
import time
from datetime import datetime
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional, Any
from contextlib import contextmanager


# =============================================================================
# ANSI Color Codes
# =============================================================================

class Colors:
    """ANSI color codes for terminal output."""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    UNDERLINE = '\033[4m'

    # Foreground colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    # Bright foreground colors
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'

    # Background colors
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'


class LogLevel(Enum):
    """Log levels for filtering output."""
    DEBUG = 0
    INFO = 1
    SUCCESS = 2
    WARNING = 3
    ERROR = 4
    CRITICAL = 5


# =============================================================================
# Test Result Data Classes
# =============================================================================

@dataclass
class TestStep:
    """Single test step result."""
    name: str
    passed: bool
    duration: float
    message: str = ""
    details: Optional[dict] = None


@dataclass
class TestResult:
    """Complete test result with steps and timing."""
    test_name: str
    passed: bool = True
    start_time: float = field(default_factory=time.time)
    end_time: float = 0.0
    steps: List[TestStep] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def duration(self) -> float:
        """Total test duration in seconds."""
        if self.end_time == 0:
            return time.time() - self.start_time
        return self.end_time - self.start_time

    @property
    def passed_steps(self) -> int:
        """Count of passed steps."""
        return sum(1 for s in self.steps if s.passed)

    @property
    def failed_steps(self) -> int:
        """Count of failed steps."""
        return sum(1 for s in self.steps if not s.passed)

    def add_step(self, name: str, passed: bool, duration: float,
                 message: str = "", details: dict = None):
        """Add a test step result."""
        self.steps.append(TestStep(name, passed, duration, message, details))
        if not passed:
            self.passed = False

    def finish(self):
        """Mark test as complete."""
        self.end_time = time.time()


# =============================================================================
# Debug Logger Class
# =============================================================================

class DebugLogger:
    """
    Comprehensive debug logger for V2N robot tests.

    Features:
        - Colorized output with log levels
        - Automatic timestamps
        - Test step tracking
        - Timing measurements
        - Structured result reporting

    Args:
        name: Logger name (shown in output)
        level: Minimum log level to display
        use_colors: Enable/disable color output
    """

    def __init__(self, name: str, level: LogLevel = LogLevel.DEBUG,
                 use_colors: bool = True):
        self.name = name
        self.level = level
        self.use_colors = use_colors and sys.stdout.isatty()
        self.indent = 0
        self._step_start_time = 0.0

    def _color(self, text: str, color: str) -> str:
        """Apply color to text if colors enabled."""
        if self.use_colors:
            return f"{color}{text}{Colors.RESET}"
        return text

    def _timestamp(self) -> str:
        """Get formatted timestamp."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _prefix(self, level_str: str, color: str) -> str:
        """Generate log prefix with timestamp and level."""
        ts = self._color(self._timestamp(), Colors.DIM)
        name = self._color(f"[{self.name}]", Colors.CYAN)
        lvl = self._color(f"[{level_str}]", color)
        indent = "  " * self.indent
        return f"{ts} {name} {lvl}{indent} "

    def _log(self, level: LogLevel, level_str: str, color: str,
             message: str, *args):
        """Internal log method."""
        if level.value < self.level.value:
            return

        if args:
            message = message % args

        prefix = self._prefix(level_str, color)
        print(f"{prefix}{message}", flush=True)

    # -------------------------------------------------------------------------
    # Log Level Methods
    # -------------------------------------------------------------------------

    def debug(self, message: str, *args):
        """Log debug message (detailed information)."""
        self._log(LogLevel.DEBUG, "DEBUG", Colors.DIM, message, *args)

    def info(self, message: str, *args):
        """Log info message (general information)."""
        self._log(LogLevel.INFO, "INFO ", Colors.BLUE, message, *args)

    def success(self, message: str, *args):
        """Log success message (test passed, operation succeeded)."""
        self._log(LogLevel.SUCCESS, " OK  ", Colors.BRIGHT_GREEN, message, *args)

    def warning(self, message: str, *args):
        """Log warning message (non-fatal issues)."""
        self._log(LogLevel.WARNING, "WARN ", Colors.YELLOW, message, *args)

    def error(self, message: str, *args):
        """Log error message (test failed, operation failed)."""
        self._log(LogLevel.ERROR, "ERROR", Colors.BRIGHT_RED, message, *args)

    def critical(self, message: str, *args):
        """Log critical message (fatal errors)."""
        self._log(LogLevel.CRITICAL, "CRIT ", Colors.BG_RED + Colors.WHITE,
                  message, *args)

    # -------------------------------------------------------------------------
    # Structured Output Methods
    # -------------------------------------------------------------------------

    def header(self, title: str):
        """Print a section header."""
        width = 60
        line = "=" * width
        if self.use_colors:
            print(f"\n{Colors.CYAN}{Colors.BOLD}{line}{Colors.RESET}")
            print(f"{Colors.CYAN}{Colors.BOLD}  {title}{Colors.RESET}")
            print(f"{Colors.CYAN}{Colors.BOLD}{line}{Colors.RESET}\n")
        else:
            print(f"\n{line}")
            print(f"  {title}")
            print(f"{line}\n")

    def subheader(self, title: str):
        """Print a subsection header."""
        if self.use_colors:
            print(f"\n{Colors.YELLOW}--- {title} ---{Colors.RESET}\n")
        else:
            print(f"\n--- {title} ---\n")

    def separator(self):
        """Print a separator line."""
        print("-" * 50)

    def data(self, label: str, value: Any, unit: str = ""):
        """Print a data value with label."""
        label_str = self._color(f"{label}:", Colors.DIM)
        value_str = self._color(str(value), Colors.WHITE)
        unit_str = self._color(unit, Colors.DIM) if unit else ""
        indent = "  " * self.indent
        print(f"{indent}  {label_str} {value_str} {unit_str}")

    def table(self, headers: List[str], rows: List[List[Any]]):
        """Print a formatted table."""
        # Calculate column widths
        widths = [len(h) for h in headers]
        for row in rows:
            for i, cell in enumerate(row):
                widths[i] = max(widths[i], len(str(cell)))

        # Print header
        header_line = " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
        separator_line = "-+-".join("-" * w for w in widths)

        if self.use_colors:
            print(f"{Colors.BOLD}{header_line}{Colors.RESET}")
        else:
            print(header_line)
        print(separator_line)

        # Print rows
        for row in rows:
            row_line = " | ".join(str(cell).ljust(widths[i])
                                  for i, cell in enumerate(row))
            print(row_line)

    # -------------------------------------------------------------------------
    # Test Step Methods
    # -------------------------------------------------------------------------

    @contextmanager
    def step(self, name: str):
        """
        Context manager for test steps with timing.

        Usage:
            with logger.step("Connect to Arduino"):
                # ... test code ...
        """
        self._step_start_time = time.time()
        self.info(f"Step: {name}")
        self.indent += 1
        try:
            yield
            duration = time.time() - self._step_start_time
            self.indent -= 1
            self.success(f"Step passed ({duration:.3f}s)")
        except Exception as e:
            duration = time.time() - self._step_start_time
            self.indent -= 1
            self.error(f"Step failed ({duration:.3f}s): {e}")
            raise

    def step_start(self, name: str):
        """Start a named test step."""
        self._step_start_time = time.time()
        self.info(f"Step: {name}")
        self.indent += 1

    def step_pass(self, message: str = ""):
        """Mark current step as passed."""
        duration = time.time() - self._step_start_time
        self.indent -= 1
        msg = f"Step passed ({duration:.3f}s)"
        if message:
            msg += f": {message}"
        self.success(msg)
        return duration

    def step_fail(self, message: str = ""):
        """Mark current step as failed."""
        duration = time.time() - self._step_start_time
        self.indent -= 1
        msg = f"Step FAILED ({duration:.3f}s)"
        if message:
            msg += f": {message}"
        self.error(msg)
        return duration

    # -------------------------------------------------------------------------
    # Result Reporting
    # -------------------------------------------------------------------------

    def report(self, result: TestResult):
        """Print a complete test result report."""
        self.header(f"Test Report: {result.test_name}")

        # Summary
        status = self._color("PASSED", Colors.BRIGHT_GREEN) if result.passed \
            else self._color("FAILED", Colors.BRIGHT_RED)
        print(f"  Status:   {status}")
        print(f"  Duration: {result.duration:.3f}s")
        print(f"  Steps:    {result.passed_steps}/{len(result.steps)} passed")

        # Steps detail
        if result.steps:
            self.subheader("Test Steps")
            for step in result.steps:
                icon = self._color("[PASS]", Colors.GREEN) if step.passed \
                    else self._color("[FAIL]", Colors.RED)
                print(f"  {icon} {step.name} ({step.duration:.3f}s)")
                if step.message:
                    print(f"         {step.message}")

        # Errors
        if result.errors:
            self.subheader("Errors")
            for err in result.errors:
                self.error(err)

        # Warnings
        if result.warnings:
            self.subheader("Warnings")
            for warn in result.warnings:
                self.warning(warn)

        self.separator()


# =============================================================================
# Convenience Functions
# =============================================================================

def log_test_start(test_name: str) -> DebugLogger:
    """Create logger and print test start header."""
    logger = DebugLogger(test_name)
    logger.header(f"Starting Test: {test_name}")
    return logger


def log_test_end(logger: DebugLogger, passed: bool, duration: float = 0.0):
    """Print test end summary."""
    if passed:
        logger.success(f"Test completed successfully" +
                       (f" ({duration:.3f}s)" if duration else ""))
    else:
        logger.error(f"Test FAILED" +
                     (f" ({duration:.3f}s)" if duration else ""))


# =============================================================================
# Main (Demo)
# =============================================================================

if __name__ == "__main__":
    # Demo the logger
    logger = DebugLogger("Demo")

    logger.header("Debug Logger Demo")

    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.success("This is a success message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")

    logger.subheader("Data Output")
    logger.data("Distance", 1.5, "m")
    logger.data("Angle", 45.0, "deg")
    logger.data("Speed", 100, "rpm")

    logger.subheader("Table Output")
    logger.table(
        ["Component", "Status", "Value"],
        [
            ["Arduino", "OK", "/dev/ttyACM0"],
            ["LiDAR", "OK", "/dev/ttyUSB0"],
            ["Camera", "WARN", "/dev/video0"],
        ]
    )

    logger.subheader("Step Tracking")
    with logger.step("Connect hardware"):
        time.sleep(0.1)
        logger.debug("Hardware connected")

    # Test result
    result = TestResult("Demo Test")
    result.add_step("Step 1", True, 0.1, "Connected")
    result.add_step("Step 2", True, 0.2, "Initialized")
    result.add_step("Step 3", False, 0.05, "Timeout error")
    result.finish()

    logger.report(result)
