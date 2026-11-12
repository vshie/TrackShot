"""Shared fixtures for every test module."""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def isolated_param_backups(tmp_path, monkeypatch):
    """Parameter backups go to a temporary directory, never the user's real one."""
    from trackshot.autopilot import backup
    directory = tmp_path / "param_backups"
    monkeypatch.setattr(backup, "backups_dir", lambda: directory)
    return directory
