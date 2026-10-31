"""Base transform steps: the verified FX1-FX5 edits, keyed by discovered roles (see base_steps/)."""
import os

from fixlib import rep

_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'base_steps')


def _run(name, R):
    with open(os.path.join(_DIR, name)) as f:
        src = f.read()
    exec(compile(src, name, 'exec'), {'rep': rep, 'R': R})


BASE_STEPS = [(lambda R, n=n: _run(n, R)) for n in sorted(os.listdir(_DIR)) if n.endswith('.py')]
