"""A prompt loop with tab completion and a command registry. No ROS, no Docker.

Split out of scripts/replay_interactive.py so this half can be tested without a
terminal, a container or a robot (docs/spec/spec1_2.md, "Layout"). The reason
is testability, not reuse: a second consumer later is a bonus and should not
shape this interface now.

    terminal = Terminal(
        prompt="clip> ",
        commands=[Command("arms", set_arms, "which arms to drive", ("none", "left", "right", "both"))],
        fallback=play_clip,                  # a line that is not a command
        fallback_candidates=list_clip_names, # what Tab completes it against
        on_interrupt=teardown,               # Ctrl-C, from anywhere
    )
    terminal.run()

Tab does two things, which is what put this in Python rather than bash: bash's
``read -e`` has no hook to replace readline's filename completer, so it can
neither complete command names nor list anything but the working directory.

- partial input: complete over the command names plus whatever
  ``fallback_candidates()`` returns, or over one command's own values once its
  name is typed;
- empty input: list all of it, commands and candidates together.

**This module never owns teardown.** ``on_interrupt`` is called and that is
all. The order things must be stopped in is safety-relevant and belongs to the
caller (docs/spec/spec1_2.md, "Ctrl-C"). A reusable terminal that knew how to
stop things would hand the next consumer somebody else's shutdown semantics.

Readline backend. GNU readline and libedit (which is what Python links against
on macOS) need different bindings for Tab, and libedit's display hook is less
reliable. ``bind_tab`` handles both and degrades to readline's own match
display if the hook is refused, so the loop behaves the same either way even
though the rendering may not be identical.
"""

from __future__ import annotations

import readline
import signal
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

# libedit reports itself here. Python on macOS links it; the rig host is Ubuntu
# and gets GNU readline.
IS_LIBEDIT = "libedit" in (readline.__doc__ or "") or "EditLine" in (
    getattr(readline, "_READLINE_LIBRARY_VERSION", "") or ""
)

# Only whitespace splits a word. The default delimiter set includes characters
# that appear in clip names (- and others), which would make Tab complete only
# the fragment after the last one.
COMPLETER_DELIMS = " \t\n"

HISTORY_LENGTH = 200

# Every signal that means "this session is over". SIGHUP is the closed terminal
# or dropped ssh connection, which is otherwise the quietest way to leave the
# robot holding a pose. scripts/replay.sh traps INT and TERM for the same
# reason; SIGKILL cannot be caught by anyone.
FATAL_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


@dataclass(frozen=True)
class Command:
    """One in-session command: its name, what runs it, and what Tab offers.

    ``handler`` takes the words after the name and returns nothing. Raising
    ``CommandError`` is how it refuses: the loop prints the message and
    reprompts, so a bad command is never fatal to a session that holds
    hardware connections.
    """

    name: str
    handler: Callable[[list[str]], None]
    help: str
    values: tuple[str, ...] = field(default=())


class CommandError(Exception):
    """A command refused its arguments. The loop prints this and carries on."""


class Terminal:
    def __init__(
        self,
        *,
        prompt: str | Callable[[], str],
        commands: Sequence[Command],
        fallback: Callable[[str], None],
        fallback_candidates: Callable[[], Iterable[str]] = lambda: (),
        on_interrupt: Callable[[], None] = lambda: None,
        history_file: Path | None = None,
        stream=None,
    ) -> None:
        names = [command.name for command in commands]
        duplicates = {name for name in names if names.count(name) > 1}
        if duplicates:
            raise ValueError(f"duplicate command names: {sorted(duplicates)}")
        self._prompt = prompt
        self._commands = {command.name: command for command in commands}
        self._fallback = fallback
        self._fallback_candidates = fallback_candidates
        self._on_interrupt = on_interrupt
        self._history_file = history_file
        self._out = stream if stream is not None else sys.stdout
        self._interrupted = False
        self._stopped = False
        self._matches: list[str] = []
        self._previous_handlers: list[tuple[int, object]] | None = None

    # ---------------------------------------------------------------- completion

    def candidates(self, line: str, text: str, begidx: int | None = None) -> list[str]:
        """What Tab offers for ``text``, given the whole ``line`` and where
        ``text`` starts in it.

        ``begidx`` is readline's start index for the word being completed.
        ``line`` is the whole buffer, not the text up to the cursor, so
        ``len(line) - len(text)`` is the right prefix only when the cursor sits
        at the end of the line. Pass ``begidx`` and the cursor may be anywhere.
        The fallback exists for callers with no readline state.

        Pure: no readline state is read here, so the rules stay testable.
        """
        cut = begidx if begidx is not None else len(line) - len(text)
        words = line[:cut].split()
        if words:
            command = self._commands.get(words[0])
            if command is not None:
                # Past the command name: offer only that command's own values.
                return [value for value in command.values if value.startswith(text)]
            return []
        pool = sorted(self._commands) + sorted(self._fallback_candidates())
        if not text:
            # Empty line: everything that can be typed here, commands and
            # clips together, so Tab alone shows the whole surface.
            return pool
        return [entry for entry in pool if entry.startswith(text)]

    def _completer(self, text: str, state: int):
        if state == 0:
            self._matches = self.candidates(
                readline.get_line_buffer(), text, readline.get_begidx()
            )
        try:
            return self._matches[state]
        except IndexError:
            return None

    def bind_tab(self) -> None:
        """Install the completer, with the binding each backend needs."""
        readline.set_completer(self._completer)
        readline.set_completer_delims(COMPLETER_DELIMS)
        readline.parse_and_bind("bind ^I rl_complete" if IS_LIBEDIT else "tab: complete")
        try:
            readline.set_completion_display_matches_hook(self._display)
        except Exception:
            pass  # readline's own column display is a fine fallback

    def _display(self, substitution, matches, longest) -> None:
        print(file=self._out)
        print("  ".join(matches), file=self._out)
        prompt = self._prompt() if callable(self._prompt) else self._prompt
        print(f"{prompt}{readline.get_line_buffer()}", end="", file=self._out, flush=True)

    # ---------------------------------------------------------------- history

    def _load_history(self) -> None:
        readline.set_history_length(HISTORY_LENGTH)
        if self._history_file and self._history_file.is_file():
            try:
                readline.read_history_file(str(self._history_file))
            except OSError:
                pass

    def _save_history(self) -> None:
        if not self._history_file:
            return
        try:
            self._history_file.parent.mkdir(parents=True, exist_ok=True)
            readline.write_history_file(str(self._history_file))
        except OSError:
            pass

    # ---------------------------------------------------------------- help

    def help_text(self) -> str:
        width = max((len(self._render_name(c)) for c in self._commands.values()), default=0)
        lines = [
            f"  {self._render_name(command):<{width}}  {command.help}"
            for command in self._commands.values()
        ]
        return "\n".join(lines)

    @staticmethod
    def _render_name(command: Command) -> str:
        return f"{command.name} {'|'.join(command.values)}" if command.values else command.name

    # ---------------------------------------------------------------- signals

    def install_signal_handlers(self) -> bool:
        """Route every fatal signal here. Idempotent; True if it installed.

        Separate from ``run`` so a caller can cover work that happens before
        the loop starts. In this session that is the startup, which brings the
        G1 container up and can wait 30 s on a device: a signal there must
        still reach the teardown.
        """
        if self._previous_handlers is not None:
            return False
        self._previous_handlers = [
            (number, signal.getsignal(number)) for number in FATAL_SIGNALS
        ]
        for number, _ in self._previous_handlers:
            signal.signal(number, self._handle_signal)
        return True

    def restore_signal_handlers(self) -> None:
        if self._previous_handlers is None:
            return
        for number, handler in self._previous_handlers:
            signal.signal(number, handler)
        self._previous_handlers = None

    # ---------------------------------------------------------------- the loop

    def dispatch(self, line: str) -> None:
        """Run one line. Refusals print and return; they never end the session."""
        line = line.strip()
        if not line:
            return
        head, *rest = line.split()
        command = self._commands.get(head)
        try:
            if command is not None:
                command.handler(rest)
            else:
                self._fallback(line)
        except CommandError as refusal:
            print(f"  {refusal}", file=self._out)

    def run(self) -> None:
        """Read and dispatch until quit, EOF or Ctrl-C.

        Ctrl-C calls ``on_interrupt`` exactly once and returns, from whichever
        state the session was in. There is no mode in which it means something
        else.
        """
        self.bind_tab()
        self._load_history()
        # The caller may already have installed them to cover its own startup.
        installed_here = self.install_signal_handlers()
        try:
            while True:
                prompt = self._prompt() if callable(self._prompt) else self._prompt
                try:
                    line = input(prompt)
                except EOFError:  # Ctrl-D: leave the way `quit` does
                    print(file=self._out)
                    return
                except KeyboardInterrupt:
                    print(file=self._out)
                    self.interrupt()
                    return
                try:
                    self.dispatch(line)
                except KeyboardInterrupt:
                    # Ctrl-C while a clip was playing: _handle_sigint has
                    # already run the teardown, and this is how it unwinds.
                    self.interrupt()
                    return
                if self._stopped:
                    return
        finally:
            if installed_here:
                self.restore_signal_handlers()
            self._save_history()

    def stop(self) -> None:
        """Ask the loop to return after the current line. What `quit` calls."""
        self._stopped = True

    def interrupt(self) -> None:
        """Run the caller's teardown once, and let nothing interrupt it.

        The teardown stops a robot: it can take 20 s or more across its docker
        calls, and a second Ctrl-C arriving part-way through used to unwind it
        and leave the arms held at weight 1. Every fatal signal is ignored for
        its duration, which is what scripts/replay.sh does by clearing its
        traps as the first statement of cleanup().
        """
        if self._interrupted:
            return
        self._interrupted = True
        previous = [(number, signal.getsignal(number)) for number in FATAL_SIGNALS]
        for number, _ in previous:
            signal.signal(number, signal.SIG_IGN)
        try:
            self._on_interrupt()
        finally:
            for number, handler in previous:
                signal.signal(number, handler)

    @property
    def interrupted(self) -> bool:
        """True once a fatal signal has been handled. main() reports 130 on it."""
        return self._interrupted

    def _handle_signal(self, signum, frame) -> None:
        # Reached when a fatal signal arrives outside input(), for instance
        # while a clip is playing and the session waits on a subprocess.
        print(file=self._out)
        self.interrupt()
        self.stop()
        raise KeyboardInterrupt
