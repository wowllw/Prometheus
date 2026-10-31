"""Shared helpers of the fix_coupling_artifacts pipeline: anchored edits, gates, logging, hashing, commands."""
import datetime
import hashlib
import json
import os
import subprocess
import sys

STATE_ROOT = os.path.expanduser('~/.ros/ablation_fix')
PIPE_DIR = os.path.join(STATE_ROOT, 'pipeline')
TEMPLATE = {}  # @KEY@ -> text, filled by discovery (package name, config dir, include paths)
TOUCHED = []   # repository files written by the pipeline (for the commit)


class GateFailure(Exception):
    def __init__(self, gate, detail, data=None):
        super().__init__('%s: %s' % (gate, detail))
        self.gate, self.detail, self.data = gate, detail, data or {}


def now():
    return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def log(msg):
    line = '[%s] %s' % (now(), msg)
    print(line, flush=True)
    os.makedirs(PIPE_DIR, exist_ok=True)
    with open(os.path.join(PIPE_DIR, 'pipeline.log'), 'a') as f:
        f.write(line + '\n')


def fill(text):
    for k, v in TEMPLATE.items():
        text = text.replace('@%s@' % k, v)
    return text


def rep(path, old, new, count=1):
    """Exact anchored replacement; the anchor must occur exactly `count` times, otherwise the edit gate fails."""
    old, new = fill(old), fill(new)
    with open(path) as f:
        s = f.read()
    n = s.count(old)
    if n != count:
        raise GateFailure('Gate-1 edit', 'anchor found %d times (want %d) in %s:\n%s' % (n, count, path, old[:400]))
    with open(path, 'w') as f:
        f.write(s.replace(old, new))
    if path not in TOUCHED:
        TOUCHED.append(path)


def write_file(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)
    if path not in TOUCHED:
        TOUCHED.append(path)


def sh(cmd, cwd=None, env=None, timeout=None, check=True, capture=True):
    """Run a command (list or string). Returns CompletedProcess; raises GateFailure on a non-zero exit when check."""
    e = dict(os.environ)
    if env:
        e.update(env)
    r = subprocess.run(cmd, cwd=cwd, env=e, timeout=timeout, shell=isinstance(cmd, str), executable='/bin/bash' if isinstance(cmd, str) else None,
                       stdout=subprocess.PIPE if capture else None, stderr=subprocess.STDOUT if capture else None,
                       universal_newlines=True)
    if check and r.returncode != 0:
        raise GateFailure('command', 'exit %d: %s\n%s' % (r.returncode, cmd if isinstance(cmd, str) else ' '.join(cmd),
                                                           (r.stdout or '')[-4000:]))
    return r


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def tree_digest(root):
    """Digest of a directory tree: relative path, size and content hash of every file (sorted)."""
    h = hashlib.sha256()
    n = 0
    for d, dirs, files in os.walk(root):
        dirs.sort()
        for fn in sorted(files):
            p = os.path.join(d, fn)
            if os.path.islink(p):
                continue
            h.update(os.path.relpath(p, root).encode())
            h.update(str(os.path.getsize(p)).encode())
            h.update(sha256_file(p).encode())
            n += 1
    return '%s (%d files)' % (h.hexdigest(), n)


def save_json(name, obj):
    os.makedirs(PIPE_DIR, exist_ok=True)
    with open(os.path.join(PIPE_DIR, name), 'w') as f:
        json.dump(obj, f, indent=2, sort_keys=True, default=str)


def load_json(name, default=None):
    p = os.path.join(PIPE_DIR, name)
    if not os.path.isfile(p):
        return default
    with open(p) as f:
        return json.load(f)


def fail_report(exc, stage):
    """Gate failure: write the diagnostic report and stop the pipeline."""
    path = os.path.join(PIPE_DIR, 'GATE_FAILURE_%s.md' % stage)
    with open(path, 'w') as f:
        f.write('# Pipeline stopped at %s\n\n' % stage)
        f.write('- time: %s\n- gate: %s\n\n## Detail\n\n```\n%s\n```\n' % (now(), getattr(exc, 'gate', type(exc).__name__),
                                                                   getattr(exc, 'detail', str(exc))))
        data = getattr(exc, 'data', None)
        if data:
            f.write('\n## Data\n\n```json\n%s\n```\n' % json.dumps(data, indent=2, default=str))
    log('GATE FAILURE at %s: %s -> %s' % (stage, exc, path))
    sys.exit(2)
