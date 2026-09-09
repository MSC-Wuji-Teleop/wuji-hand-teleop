"""Pin scripts/interactive/terminal.py: completion, dispatch, and Ctrl-C.

No terminal, no container, no robot. That is the whole reason the prompt loop
is a separate module (docs/spec/spec1_2.md, "Layout").

``candidates`` is pure by design: it takes the line and the word being
completed rather than reading readline state, so the completion rules are
testable without a tty.
"""

from __future__ import annotations

import io
import signal
import textwrap

import pytest

from interactive.terminal import Command, CommandError, Terminal


def _terminal(**overrides):
    calls: dict[str, list] = {"arms": [], "ls": [], "fallback": []}

    def record(key):
        def handler(words):
            calls[key].append(words)
        return handler

    def fallback(line):
        calls["fallback"].append(line)

    commands = overrides.pop("commands", None) or [
        Command("arms", record("arms"), "which arms", ("none", "left", "right", "both")),
        Command("hands", record("arms"), "which hands", ("none", "left", "right", "both")),
        Command("ls", record("ls"), "list clips"),
        Command("quit", record("ls"), "leave"),
    ]
    kwargs = dict(
        prompt="clip> ",
        commands=commands,
        fallback=fallback,
        fallback_candidates=lambda: ["90_sweep_joints_GT", "05_test_GT", "15_val_Ours"],
        stream=io.StringIO(),
    )
    kwargs.update(overrides)
    return Terminal(**kwargs), calls


# ---------------------------------------------------------------- completion


def test_empty_line_lists_both_the_commands_and_the_clips():
    """The behaviour bash's `read -e` cannot provide: Tab on nothing shows the
    whole surface, so an operator never has to know a clip name in advance."""
    terminal, _ = _terminal()
    assert terminal.candidates("", "") == [
        "arms", "hands", "ls", "quit",              # commands, sorted
        "05_test_GT", "15_val_Ours", "90_sweep_joints_GT",  # then clips, sorted
    ]


def test_a_prefix_completes_commands_and_clips_together():
    terminal, _ = _terminal()
    assert terminal.candidates("l", "l") == ["ls"]
    assert terminal.candidates("9", "9") == ["90_sweep_joints_GT"]
    assert terminal.candidates("1", "1") == ["15_val_Ours"]


def test_an_ambiguous_prefix_returns_every_match():
    terminal, _ = _terminal()
    assert terminal.candidates("h", "h") == ["hands"]
    assert sorted(terminal.candidates("0", "0")) == ["05_test_GT"]


def test_after_a_command_name_only_its_own_values_are_offered():
    terminal, _ = _terminal()
    assert terminal.candidates("arms ", "") == ["none", "left", "right", "both"]
    assert terminal.candidates("arms l", "l") == ["left"]
    assert terminal.candidates("arms x", "x") == []


def test_a_command_with_no_values_offers_nothing_after_it():
    """`ls` takes no argument, so Tab past it must not fall back to clip names."""
    terminal, _ = _terminal()
    assert terminal.candidates("ls ", "") == []


def test_a_clip_name_in_the_first_word_offers_nothing_after_it():
    terminal, _ = _terminal()
    assert terminal.candidates("90_sweep_joints_GT ", "") == []


def test_an_ambiguous_prefix_returns_every_match_not_just_the_first():
    """The fixture has no ambiguous prefix, so this builds one."""
    terminal, _ = _terminal(
        commands=[Command("state", lambda w: None, "s"),
                  Command("speed", lambda w: None, "p"),
                  Command("stop", lambda w: None, "t")],
        fallback_candidates=lambda: ["st_clip_a", "st_clip_b"],
    )
    assert terminal.candidates("st", "st") == ["state", "stop", "st_clip_a", "st_clip_b"]


def test_one_prefix_can_return_both_a_command_and_a_clip():
    terminal, _ = _terminal(
        commands=[Command("sweep", lambda w: None, "s")],
        fallback_candidates=lambda: ["sweep_clip"],
    )
    assert terminal.candidates("swe", "swe") == ["sweep", "sweep_clip"]


def test_an_empty_tab_offers_the_clips_it_was_given():
    terminal, _ = _terminal(fallback_candidates=lambda: ["a_clip", "b_clip"])
    offered = terminal.candidates("", "")
    assert "a_clip" in offered and "b_clip" in offered
    assert "arms" in offered


def test_an_empty_tab_past_a_command_still_offers_only_its_values():
    """Listing everything applies to the first word only."""
    terminal, _ = _terminal()
    assert terminal.candidates("arms ", "", 5) == ["none", "left", "right", "both"]
    assert terminal.candidates("ls ", "", 3) == []


def test_a_prefix_matching_nothing_completes_to_nothing():
    terminal, _ = _terminal()
    assert terminal.candidates("zzz", "zzz") == []


def test_the_completer_walks_states_then_returns_none(monkeypatch):
    """readline calls the completer with state 0, 1, 2 ... until it returns None."""
    terminal, _ = _terminal()
    monkeypatch.setattr("interactive.terminal.readline.get_line_buffer", lambda: "arms ")
    monkeypatch.setattr("interactive.terminal.readline.get_begidx", lambda: 5)
    assert terminal._completer("", 0) == "none"
    assert terminal._completer("", 1) == "left"
    assert terminal._completer("", 4) is None


def test_the_completer_uses_begidx_so_the_cursor_can_be_anywhere(monkeypatch):
    """get_line_buffer() is the whole line, not the text before the cursor.
    Completing with the cursor moved back into the line used to offer nothing,
    because len(line) - len(text) is only begidx at end of line."""
    terminal, _ = _terminal()
    # Cursor sits just after "90_" in "90_sweep_joints_GT": begidx 0, but the
    # length arithmetic would give 15 and look like a second word.
    monkeypatch.setattr("interactive.terminal.readline.get_line_buffer",
                        lambda: "90_sweep_joints_GT")
    monkeypatch.setattr("interactive.terminal.readline.get_begidx", lambda: 0)
    assert terminal._completer("90_", 0) == "90_sweep_joints_GT"


def test_candidates_prefers_begidx_over_the_length_fallback():
    terminal, _ = _terminal()
    # Without begidx the arithmetic misreads this as being past a first word.
    assert terminal.candidates("arms both", "arms") == []
    assert terminal.candidates("arms both", "arms", 0) == ["arms"]


def test_clip_candidates_are_read_fresh_each_time():
    """The clip list is a callable, not a snapshot, so a clip prepared during
    the session completes without a restart."""
    names = ["zz_one"]
    terminal, _ = _terminal(fallback_candidates=lambda: names)
    assert terminal.candidates("zz", "zz") == ["zz_one"]
    names.append("zz_two")
    assert terminal.candidates("zz", "zz") == ["zz_one", "zz_two"]


# ---------------------------------------------------------------- dispatch


def test_a_command_reaches_its_handler_with_the_rest_of_the_line():
    terminal, calls = _terminal()
    terminal.dispatch("arms left")
    assert calls["arms"] == [["left"]]


def test_a_line_that_is_not_a_command_goes_to_the_fallback():
    terminal, calls = _terminal()
    terminal.dispatch("90_sweep_joints_GT")
    assert calls["fallback"] == ["90_sweep_joints_GT"]
    assert calls["arms"] == []


def test_an_empty_line_does_nothing():
    """Enter reprompts, the way a shell does. Listing is `ls` or Tab."""
    terminal, calls = _terminal()
    terminal.dispatch("")
    terminal.dispatch("   ")
    assert calls == {"arms": [], "ls": [], "fallback": []}


def test_surrounding_whitespace_is_ignored():
    terminal, calls = _terminal()
    terminal.dispatch("  arms   left  ")
    assert calls["arms"] == [["left"]]


def test_a_refusal_is_printed_and_the_session_survives():
    """A bad command must never end a session that holds hardware connections."""
    def refuse(words):
        raise CommandError("only 'left' was connected at startup")

    stream = io.StringIO()
    terminal, calls = _terminal(
        commands=[Command("arms", refuse, "which arms", ("left",)),
                  Command("ls", lambda words: None, "list")],
        stream=stream,
    )
    terminal.dispatch("arms both")       # refused
    terminal.dispatch("ls")              # still working
    assert "only 'left' was connected at startup" in stream.getvalue()


def test_a_handler_raising_something_else_is_not_swallowed():
    """CommandError is the refusal channel. A bug must not look like a refusal."""
    def boom(words):
        raise ValueError("a real bug")

    terminal, _ = _terminal(commands=[Command("arms", boom, "which arms")])
    with pytest.raises(ValueError, match="a real bug"):
        terminal.dispatch("arms left")


# ---------------------------------------------------------------- fatal signals


def test_the_terminal_runs_no_command_of_its_own():
    """It records a signal and raises; the stopping is the caller's
    (docs/spec/spec1_2.md, "Ctrl-C").

    Asserted over the module's imports rather than its text, so it fails if
    this file ever grows the ability to shell out and cannot be satisfied by
    prose. It is the mechanical half of the split: a terminal that could run
    docker commands would hand the next consumer somebody else's shutdown.
    """
    import ast
    import inspect

    import interactive.terminal as module

    tree = ast.parse(inspect.getsource(module))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    assert "subprocess" not in imported
    assert "os" not in imported
    assert imported <= {"readline", "signal", "sys", "dataclasses", "pathlib",
                        "typing", "__future__"}, f"unexpected imports: {imported}"


def test_the_handler_does_no_work():
    """Python runs the handler at the next bytecode boundary of whatever the
    main thread was doing: readline, a docker command, a release half done.
    Anything it did would run nested inside that state. So its body assigns
    and raises, and calls nothing."""
    import ast
    import inspect

    tree = ast.parse(textwrap.dedent(inspect.getsource(Terminal._handle_signal)))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    assert calls == [], f"the handler calls: {[ast.unparse(c) for c in calls]}"


def test_a_fatal_signal_records_stops_and_raises():
    terminal, _ = _terminal()
    with pytest.raises(KeyboardInterrupt):
        terminal._handle_signal(signal.SIGINT, None)
    assert terminal.interrupted is True
    assert terminal._stopped is True


@pytest.mark.parametrize("number", [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_every_fatal_signal_is_handled_not_just_sigint(number):
    """SIGHUP is a closed terminal or a dropped ssh session, which is otherwise
    the quietest way to leave the robot holding a pose."""
    from interactive.terminal import FATAL_SIGNALS

    assert number in FATAL_SIGNALS
    terminal, _ = _terminal()
    assert terminal.install_signal_handlers() is True
    try:
        assert signal.getsignal(number) == terminal._handle_signal
    finally:
        terminal.restore_signal_handlers()


def test_ignore_fatal_signals_survives_a_signal():
    """What a second Ctrl-C does during the teardown: nothing. The caller sets
    this before stopping anything and restores its handlers after."""
    import os

    from interactive.terminal import FATAL_SIGNALS, ignore_fatal_signals

    before = {number: signal.getsignal(number) for number in FATAL_SIGNALS}
    try:
        ignore_fatal_signals()
        for number in FATAL_SIGNALS:
            assert signal.getsignal(number) is signal.SIG_IGN
        os.kill(os.getpid(), signal.SIGINT)
        survived = True
    finally:
        for number, handler in before.items():
            signal.signal(number, handler)
    assert survived


def test_install_is_idempotent_so_a_callers_handlers_win():
    """main() installs before startup; run() must not replace or double-save."""
    terminal, _ = _terminal()
    assert terminal.install_signal_handlers() is True
    assert terminal.install_signal_handlers() is False
    terminal.restore_signal_handlers()


def test_stop_ends_the_loop_after_the_current_line():
    terminal, _ = _terminal()
    assert terminal._stopped is False
    terminal.stop()
    assert terminal._stopped is True


def test_ctrl_c_at_the_prompt_raises_out_of_run(monkeypatch):
    """run() catches nothing: the caller's finally is the one teardown path."""
    terminal, _ = _terminal()
    monkeypatch.setattr("builtins.input", lambda prompt: (_ for _ in ()).throw(KeyboardInterrupt))
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    with pytest.raises(KeyboardInterrupt):
        terminal.run()


def test_ctrl_c_while_a_clip_plays_raises_out_of_run(monkeypatch):
    """The signal arrives inside the line's handler, not at input()."""
    def play(line):
        raise KeyboardInterrupt

    terminal, _ = _terminal(fallback=play)
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    lines = iter(["90_sweep_joints_GT"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(lines))
    with pytest.raises(KeyboardInterrupt):
        terminal.run()


def test_ctrl_d_returns_normally(monkeypatch):
    """EOF is `quit`: run() returns and main()'s finally tears down."""
    terminal, _ = _terminal()
    monkeypatch.setattr("builtins.input", lambda prompt: (_ for _ in ()).throw(EOFError))
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    terminal.run()
    assert terminal.interrupted is False


def test_run_restores_the_previous_sigint_handler(monkeypatch):
    terminal, _ = _terminal()
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    monkeypatch.setattr("builtins.input", lambda prompt: (_ for _ in ()).throw(EOFError))
    before = signal.getsignal(signal.SIGINT)
    terminal.run()
    assert signal.getsignal(signal.SIGINT) is before


def test_run_restores_handlers_when_interrupted(monkeypatch):
    terminal, _ = _terminal()
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    monkeypatch.setattr("builtins.input", lambda prompt: (_ for _ in ()).throw(KeyboardInterrupt))
    before = signal.getsignal(signal.SIGINT)
    with pytest.raises(KeyboardInterrupt):
        terminal.run()
    assert signal.getsignal(signal.SIGINT) is before


def test_a_hung_up_terminal_does_not_change_the_exit_path(monkeypatch):
    """After SIGHUP every write fails with EIO. The newline run() prints on the
    way out must not turn the KeyboardInterrupt into an OSError."""
    class Gone(io.StringIO):
        def write(self, s):
            raise OSError(5, "Input/output error")

    terminal, _ = _terminal(stream=Gone())
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    monkeypatch.setattr("builtins.input", lambda prompt: (_ for _ in ()).throw(KeyboardInterrupt))
    with pytest.raises(KeyboardInterrupt):
        terminal.run()


# ---------------------------------------------------------------- misc


def test_duplicate_command_names_are_refused_at_construction():
    with pytest.raises(ValueError, match="duplicate command names"):
        Terminal(
            prompt="> ",
            commands=[Command("arms", lambda w: None, "a"), Command("arms", lambda w: None, "b")],
            fallback=lambda line: None,
        )


def test_help_lists_every_command_with_its_values():
    terminal, _ = _terminal()
    text = terminal.help_text()
    assert "arms none|left|right|both" in text
    assert "ls" in text
    for command in ("arms", "hands", "ls", "quit"):
        assert command in text


def test_bind_tab_installs_the_completer_and_whitespace_only_delimiters(monkeypatch):
    """Clip names carry - and _, and readline's default delimiters include
    some of those, which would complete only the fragment after the last one.
    Asserted through bind_tab so deleting the call is caught."""
    recorded = {}
    monkeypatch.setattr("interactive.terminal.readline.set_completer",
                        lambda fn: recorded.__setitem__("completer", fn))
    monkeypatch.setattr("interactive.terminal.readline.set_completer_delims",
                        lambda d: recorded.__setitem__("delims", d))
    monkeypatch.setattr("interactive.terminal.readline.parse_and_bind",
                        lambda spec: recorded.__setitem__("bind", spec))
    terminal, _ = _terminal()
    terminal.bind_tab()
    assert recorded["completer"] == terminal._completer
    assert recorded["delims"].strip() == "", "only whitespace may split a word"
    for character in "-_.":
        assert character not in recorded["delims"]
    assert "rl_complete" in recorded["bind"] or "tab: complete" in recorded["bind"]


def test_run_binds_tab(monkeypatch):
    """Every other run() test patches bind_tab away, so one has to pin it."""
    called = []
    terminal, _ = _terminal()
    monkeypatch.setattr(terminal, "bind_tab", lambda: called.append(True))
    monkeypatch.setattr("builtins.input", lambda prompt: (_ for _ in ()).throw(EOFError))
    terminal.run()
    assert called == [True]


def test_run_returns_when_a_command_calls_stop(monkeypatch):
    """`quit` works by calling stop(); the loop has to actually honour it."""
    terminal = None

    def quit_(words):
        terminal.stop()

    terminal, _ = _terminal(commands=[Command("quit", quit_, "leave")])
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    lines = iter(["quit", "should never be read"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(lines))
    terminal.run()
    assert next(lines) == "should never be read"


def test_run_calls_a_callable_prompt_each_iteration(monkeypatch):
    """Pins Terminal's own callable() branch, not the lambda a test passes in."""
    seen = []
    state = {"n": 0}

    def prompt():
        state["n"] += 1
        return f"[{state['n']}] > "

    terminal, _ = _terminal(prompt=prompt)
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)
    lines = iter(["", "", EOFError])

    def fake_input(text):
        seen.append(text)
        value = next(lines)
        if value is EOFError:
            raise EOFError
        return value

    monkeypatch.setattr("builtins.input", fake_input)
    terminal.run()
    assert seen == ["[1] > ", "[2] > ", "[3] > "]


def test_a_plain_string_prompt_is_used_as_is(monkeypatch):
    seen = []
    terminal, _ = _terminal(prompt="clip> ")
    monkeypatch.setattr(terminal, "bind_tab", lambda: None)

    def fake_input(text):
        seen.append(text)
        raise EOFError

    monkeypatch.setattr("builtins.input", fake_input)
    terminal.run()
    assert seen == ["clip> "]
