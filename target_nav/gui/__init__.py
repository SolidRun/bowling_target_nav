"""GUI package -- GTK3 windows and pure-Cairo rendering panels.

Modules:
    main_window      -- Fullscreen MainGUI with split radar/camera panels (15 FPS).
    settings_window  -- Notebook-tabbed settings with sliders and toggles.
    display          -- Auto-detect Wayland/X11 for V2N embedded board.
    theme            -- GitHub-dark CSS theme applied globally.

Subpackages:
    panels/          -- Cairo panel renderers (camera_panel, map_panel).
    settings_tabs/   -- Mixin classes for each settings tab (nav, detect,
                        radar, setup, tools).
"""
