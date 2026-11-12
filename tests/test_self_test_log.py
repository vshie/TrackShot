"""--self-test can also write its report to a file (TRACKSHOT_SELFTEST_LOG): a windowed Windows exe
has no stdout, and CI needs the report from it."""

import sys

import trackshot.__main__ as entry


def fake_self_test() -> int:
    print("ok    something")
    print("self-test passed")
    return 0


def test_report_goes_to_the_log_file_and_stdout(tmp_path, monkeypatch, capsys):
    log = tmp_path / "selftest.txt"
    monkeypatch.setenv("TRACKSHOT_SELFTEST_LOG", str(log))
    monkeypatch.setattr(entry, "_self_test", fake_self_test)
    original = sys.stdout

    assert entry.self_test() == 0

    assert sys.stdout is original
    assert log.read_text(encoding="utf-8") == "ok    something\nself-test passed\n"
    assert "self-test passed" in capsys.readouterr().out


def test_log_file_works_without_a_console(tmp_path, monkeypatch):
    log = tmp_path / "selftest.txt"
    monkeypatch.setenv("TRACKSHOT_SELFTEST_LOG", str(log))
    monkeypatch.setattr(entry, "_self_test", fake_self_test)
    monkeypatch.setattr(sys, "stdout", None)     # what a windowed Windows exe has

    assert entry.self_test() == 0

    assert sys.stdout is None
    assert "self-test passed" in log.read_text(encoding="utf-8")


def test_without_the_variable_nothing_is_redirected(monkeypatch, capsys):
    monkeypatch.delenv("TRACKSHOT_SELFTEST_LOG", raising=False)
    monkeypatch.setattr(entry, "_self_test", fake_self_test)

    assert entry.self_test() == 0
    assert "self-test passed" in capsys.readouterr().out
