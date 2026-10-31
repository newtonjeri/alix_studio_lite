"""Lite's exit after the optimiser process and a process of its own were used (server/workers.py): the pools'
semaphores are released, so the resource tracker has nothing to report. The owner's terminal, 18 Oct, after a session with a collision check:
"resource_tracker: There appear to be 5 leaked semaphore objects to clean up at shutdown"."""
import subprocess
import sys
import textwrap

SCRIPT = textwrap.dedent("""
    import os, sys
    from alix_studio_lite.server import workers

    def main():
        assert workers.pool().submit(len, [1, 2, 3]).result() == 3
        assert workers.pool("plans_ahead").submit(len, [1, 2]).result() == 2     # a process of its own (place_correct's)
        workers.shutdown()
        sys.stderr.flush()
        os._exit(0)                       # as Lite leaves (server/__init__.py)

    if __name__ == "__main__":
        main()
""")


def test_lite_s_exit_leaves_no_semaphore_behind(tmp_path):
    script = tmp_path / "exit_like_lite.py"
    script.write_text(SCRIPT, encoding="utf-8")
    done = subprocess.run([sys.executable, str(script)], capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stderr
    assert "leaked semaphore" not in done.stderr, done.stderr
