"""
Core Module (unused)
====================

Legacy infrastructure modules.  Not imported by the active codebase.
Retained for reference only.

Contents:
    - ``config.py`` -- Configuration dataclasses and YAML loader.
      Still usable, but the active codebase reads nav params from
      ``detection_store`` / ``calibration.json`` instead.
    - ``state_machine.py`` -- **LEGACY**.  Formal State-pattern state
      machine, superseded by the direct navigation loop in
      ``nav/navigator.py`` and the process-level state facades in
      ``state/``.
    - ``events.py`` -- Publish-subscribe event bus (threading mode only).
"""
