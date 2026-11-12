"""Settings.validate: values that would start the camera but never produce VIO."""

import pytest

from trackshot.config import Settings


@pytest.mark.parametrize("width, ok", [(320, True), (480, True), (640, True), (300, False), (330, False)])
def test_rtabmap_width_must_be_a_multiple_of_16(width, ok):
    problems = Settings(backend="rtabmap", width=width, height=200).validate()
    assert (not problems) == ok
    if not ok:
        assert any("multiple of 16" in p for p in problems)


def test_the_message_suggests_the_nearest_widths():
    (problem,) = Settings(backend="rtabmap", width=300, height=200).validate()
    assert "use 288 or 304" in problem


def test_basalt_takes_any_width():
    # Basalt doesn't use the OAK's stereo depth: 300×200 ran in the field
    assert Settings(backend="basalt", width=300, height=200).validate() == []
