#!/usr/bin/env python3
"""fix_coupling_artifacts pipeline: mandatory dry run in a sandbox, baseline protection, introspective discovery,
layered FX1-FX6 edits behind default-false flags, build + offline tests (Gate-T0), V1 smoke ladder and V2 screening
(3 seeds per configuration), V3 only after manual confirmation, report.

  pipeline.py [--repo R] run [--until dryrun|stage0|stage1|stage2|v1|v2|report]   (dry run always comes first)
  pipeline.py [--repo R] dryrun                                                  (sandbox only, no repository change)
  pipeline.py [--repo R] run-v3 --confirm-v3                                     (manual confirmation required)
  pipeline.py [--repo R] new-iteration --note "<what was changed and why>"
  pipeline.py [--repo R] stop-report | integrity

A gate failure writes ~/.ros/ablation_fix/pipeline/GATE_FAILURE_<stage>_<time>.md and exits with code 2; the pipeline
never retries or repairs itself. Completed stages (state.json, per iteration) are skipped on the next invocation."""
import argparse
import difflib
import glob
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# never write bytecode caches: the project's scripts directory holds tracked __pycache__ files
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
sys.path.insert(0, HERE)
import fixlib as L  # noqa: E402
from fixlib import log, sh, GateFailure, save_json, load_json, PIPE_DIR, STATE_ROOT  # noqa: E402
import discover as D  # noqa: E402
import transforms_new as TN  # noqa: E402
import analysis as A  # noqa: E402

TAG = 'thesis-frozen-v1'
TAG_MSG = 'frozen baseline for thesis chapter4'
BRANCH = 'fix_coupling_artifacts'
NEW_TESTS = ['gen_golden', 'test_fixes']
TEMPLATES = ['golden_scenarios.h', 'gen_golden.cpp', 'test_fixes.cpp']
STAGES = ['dryrun', 'dryrun_check', 'stage0', 'stage1', 'stage2', 'v1', 'v2', 'report']
V1_SEEDS, V1_VARIANTS, V1_GROUPS = [101, 102, 103], ['S3_climb_ne', 'head_on', 'M1_three_way'], ['g6']
V2_SEEDS, V2_GROUPS = [1, 2, 3], ['g3', 'g5', 'g6']
V3_SEEDS, V3_GROUPS, V3_OPTIONAL = list(range(21, 41)), ['g3', 'g6'], ['g5']
DIST_MARGIN, M1_FACTOR, R4_WINDOW = 0.05, 1.2, 2.5
TRAILER = 'Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>'
CODE_EXT = ('.cpp', '.cc', '.h', '.hpp', '.launch', '.yaml', '.sh', 'CMakeLists.txt', '.xml')
PER_ITER = ['done_stages', 'dryrun', 'dryrun_check', 'dryrun_superseded', 'stage2_result', 'v1', 'v2', 'v3',
            'report', 'iteration_status']


# ---------------------------------------------------------------- state
def state():
    return load_json('state.json', {})


def set_state(key, val):
    s = state()
    s[key] = val
    save_json('state.json', s)


def iteration():
    return int(state().get('iteration', 1))


def out_root():
    """Simulation outputs of the current iteration (iteration 1 kept its original layout)."""
    return STATE_ROOT if iteration() == 1 else os.path.join(STATE_ROOT, 'iter%d' % iteration())


def git(R, *args, check=True):
    return sh(['git'] + list(args), cwd=R['repo'], check=check).stdout.strip()


# ---------------------------------------------------------------- integrity of the read-only thesis assets
def assets(R):
    a = {'chapter4_doc': L.sha256_file(R['chapter4_doc']), 'frozen_dataset': L.tree_digest(R['frozen_out']),
         'tag_commit': git(R, 'rev-parse', TAG + '^{commit}'), 'tag_object': git(R, 'rev-parse', TAG),
         'base_branch_commit': git(R, 'rev-parse', 'refs/heads/' + state()['stage0']['base_branch'])}
    for d in R['frozen_analysis_dirs']:
        a['frozen_analysis:' + os.path.basename(d)] = L.tree_digest(d)
    return a


def integrity(R, where):
    base = state()['stage0']['assets']
    now = assets(R)
    bad = {k: (base[k], now.get(k)) for k in base if now.get(k) != base[k]}
    if bad:
        raise GateFailure('Gate-I integrity (%s)' % where, 'read-only assets changed', bad)
    log('integrity OK (%s): chapter-4 doc, frozen dataset, frozen analysis outputs, tag, base branch unchanged' % where)


# ---------------------------------------------------------------- stage 0: git baseline + discovery
def discover_only(repo):
    R = D.discover(repo)
    D.report(R)
    return R


def stage0(repo):
    R = {'repo': repo}
    cur = git(R, 'rev-parse', '--abbrev-ref', 'HEAD')
    porcelain = sh(['git', 'status', '--porcelain'], cwd=repo).stdout.rstrip('\n').split('\n')
    mods = [l[3:] for l in porcelain if l and not l.startswith('??')]
    blocked = [m for m in mods if m.endswith(CODE_EXT)]
    if blocked and cur != BRANCH:
        raise GateFailure('Gate-0 git', 'tracked source/config files have uncommitted changes; the tag would not '
                                        'describe the code that ran: %s' % blocked)
    prev = state().get('stage0', {})
    if 'assets' in prev:  # hashes of the read-only assets, recorded before anything was changed
        R = D.discover(repo)
        integrity(R, 'stage0 entry')
    if not git(R, 'rev-parse', '-q', '--verify', 'refs/tags/' + TAG, check=False):
        if cur == BRANCH:
            raise GateFailure('Gate-0 git', 'tag %s missing while on %s' % (TAG, BRANCH))
        sh(['git', 'tag', '-a', TAG, '-m', TAG_MSG], cwd=repo)
        log('created annotated tag %s at %s' % (TAG, git(R, 'rev-parse', 'HEAD')))
    if git(R, 'cat-file', '-t', TAG) != 'tag':
        raise GateFailure('Gate-0 git', '%s exists but is not an annotated tag' % TAG)
    base_branch = prev.get('base_branch') or (cur if cur != BRANCH else None)
    if not base_branch:
        raise GateFailure('Gate-0 git', 'cannot tell the protected base branch')
    if not git(R, 'rev-parse', '-q', '--verify', 'refs/heads/' + BRANCH, check=False):
        sh(['git', 'checkout', '-b', BRANCH, TAG], cwd=repo)
        log('created and checked out %s from %s' % (BRANCH, TAG))
    elif cur != BRANCH:
        sh(['git', 'checkout', BRANCH], cwd=repo)
    if sh(['git', 'merge-base', '--is-ancestor', TAG, BRANCH], cwd=repo, check=False).returncode != 0:
        raise GateFailure('Gate-0 git', '%s does not descend from %s' % (BRANCH, TAG))
    R = discover_only(repo)
    s0 = dict(prev)
    s0.update(base_branch=base_branch, tag=TAG, tag_commit=git(R, 'rev-parse', TAG + '^{commit}'), branch=BRANCH,
              uncommitted_user_files=mods, post_process_sha256=L.sha256_file(R['post_process']))
    set_state('stage0', s0)
    if 'assets' not in prev:
        s0['assets'] = assets(R)
        set_state('stage0', s0)
    integrity(R, 'stage0')
    return R


def ensure_branch(R):
    if git(R, 'rev-parse', '--abbrev-ref', 'HEAD') != BRANCH:
        raise GateFailure('Gate-B branch', 'not on %s' % BRANCH)


def commit(R, paths, msg):
    rel = sorted({os.path.relpath(p, R['repo']) for p in paths})
    if not rel:
        return None
    sh(['git', 'add', '--'] + rel, cwd=R['repo'])
    if sh(['git', 'diff', '--cached', '--quiet'], cwd=R['repo'], check=False).returncode == 0:
        return None
    sh(['git', 'commit', '-q', '-m', msg + '\n\n' + TRAILER], cwd=R['repo'])
    h = git(R, 'rev-parse', '--short', 'HEAD')
    log('commit %s on %s: %s' % (h, BRANCH, msg))
    return h


def sync_copy(R, commit_now=True):
    """Keep the committed copy of the pipeline (next to the analysis script) identical to the code that runs."""
    dst = os.path.join(os.path.dirname(R['post_process']), D.PIPELINE_DIRNAME)
    if os.path.realpath(HERE) == os.path.realpath(dst):
        return
    files = []
    for f in glob.glob(os.path.join(HERE, '*.py')) + glob.glob(os.path.join(HERE, 'templates', '*')) + \
            glob.glob(os.path.join(HERE, 'base_steps', '*.py')):
        p = os.path.join(dst, os.path.relpath(f, HERE))
        text = open(f).read()
        if not os.path.isfile(p) or open(p).read() != text:
            L.write_file(p, text)
            files.append(p)
    if files and commit_now:
        commit(R, files, 'Pipeline update (%s)' % ', '.join(sorted(os.path.relpath(p, dst) for p in files)))


# ---------------------------------------------------------------- edits: layers + generated files
def templates_fill(R):
    L.TEMPLATE.update(FIND_CONFIG=R['find_config'], INC_CORE=R['inc_core'],
                      TEST_COMMON_H=os.path.basename(R['test_common_h']))


def apply_layers(R, only_missing):
    applied = []
    for name, role, marker, fn in TN.layers():
        if only_missing and re.search(marker, open(R[role]).read()):
            continue
        fn(R)
        applied.append(name)
    TN.cmake_tests(R, NEW_TESTS)
    return applied


def generate(R):
    for fn in TEMPLATES:
        L.write_file(os.path.join(R['test_dir'], fn), L.fill(open(os.path.join(HERE, 'templates', fn)).read()))
    for name, (flags, note) in TN.default_presets().items():
        TN.write_preset(R, name, flags, note)


# ---------------------------------------------------------------- compile helpers
def flags_make(path):
    t = open(path).read()
    cxx = re.search(r'^# compile CXX with (\S+)', t, flags=re.M).group(1)
    flags = re.search(r'^CXX_FLAGS = (.*)$', t, flags=re.M).group(1).split()
    incs = re.search(r'^CXX_INCLUDES = (.*)$', t, flags=re.M).group(1).split()
    defs = [d.replace('\\"', '"') for d in re.search(r'^CXX_DEFINES = (.*)$', t, flags=re.M).group(1).split()]  # make escaping
    eig = []
    for i, x in enumerate(incs):
        if 'eigen' in x.lower():
            eig += ([incs[i - 1]] if incs[i - 1] in ('-isystem', '-I') else []) + [x]
    return cxx, flags, eig, incs, defs


def compile_unit(cxx, args, src, out=None, syntax_only=False):
    cmd = [cxx] + args + (['-fsyntax-only'] if syntax_only else ['-c', '-o', out]) + [src]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    warnings = [l for l in r.stdout.split('\n') if ': warning:' in l]
    return r.returncode, r.stdout, warnings


def node_log_format_check(R, frozen_pkg, patched_node_cpp):
    """cycles.csv / paths.csv identity at source level: every function of the node that writes these streams keeps
    every frozen line in order; additions are guarded (ext_open_ / fx.any()) and never write to these streams."""
    rel = os.path.relpath(R['node_cpp'], R['pkg_root'])
    old, new = open(os.path.join(frozen_pkg, rel)).read(), open(patched_node_cpp).read()

    def bodies(text):
        out = {}
        for m in re.finditer(r'\n  \w[\w:<>, ]* PlannerNode::(\w+)\([^)]*\)[^{;]*\n  \{', text):
            i, depth = m.end() - 1, 0
            for j in range(i, len(text)):
                depth += {'{': 1, '}': -1}.get(text[j], 0)
                if depth == 0:
                    out[m.group(1)] = text[i:j + 1]
                    break
        return out

    ob, nb = bodies(old), bodies(new)
    streams = re.compile(r'\b(cycles_csv_|paths_csv_)\b')
    checked, problems = [], []
    for fn, body in ob.items():
        if not streams.search(body):
            continue
        checked.append(fn)
        a, b = body.split('\n'), nb.get(fn, '').split('\n')
        for op, i1, i2, j1, j2 in difflib.SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
            if op in ('replace', 'delete'):
                problems.append('%s: frozen lines changed: %s' % (fn, a[i1:i2]))
            elif op == 'insert':
                ins = b[j1:j2]
                if any(streams.search(x) for x in ins):
                    problems.append('%s: added lines write to a frozen stream: %s' % (fn, ins))
                if not re.search(r'if \((ext_open_|core_cfg_\.sw\.fx\.any\(\)|sw\.fx\.any\(\))\)', ins[0]) and \
                        not all(re.match(r'\s*(//.*)?$', x) for x in ins):
                    problems.append('%s: unguarded addition: %s' % (fn, ins[:3]))
    if problems or not checked:
        raise GateFailure('Gate-T0 log format', 'node writers of cycles.csv/paths.csv not provably unchanged',
                          {'checked': checked, 'problems': problems})
    return checked


# ---------------------------------------------------------------- dry run (sandbox only)
def edit_fingerprint():
    """Hash of everything that decides the edits (layers, base steps, templates); a dry run is valid only for it."""
    import hashlib
    h = hashlib.sha256()
    for f in sorted(glob.glob(os.path.join(HERE, 'transforms_*.py')) + glob.glob(os.path.join(HERE, 'base_steps', '*.py')) +
                    glob.glob(os.path.join(HERE, 'templates', '*'))):
        h.update(os.path.relpath(f, HERE).encode())
        h.update(open(f, 'rb').read())
    return h.hexdigest()


def console_path():
    return os.path.join(PIPE_DIR, 'dryrun_iter%d_console.log' % iteration())


def dlog(msg):
    log(msg)
    with open(console_path(), 'a') as f:
        f.write('[%s] %s\n' % (L.now(), msg))


def sandbox_map(R, root):
    """Role map pointing into a sandbox copy of the repository subtree."""
    S = {}
    for k, v in R.items():
        if isinstance(v, str) and v.startswith(R['repo'] + os.sep):
            S[k] = os.path.join(root, os.path.relpath(v, R['repo']))
        elif isinstance(v, list) and v and all(isinstance(x, str) and x.startswith(R['repo'] + os.sep) for x in v):
            S[k] = [os.path.join(root, os.path.relpath(x, R['repo'])) for x in v]
        else:
            S[k] = v
    S['repo'] = root
    return S


def dryrun(R):
    """Apply every layer to a sandbox copy of the tag, write the full diff, compile every translation unit with the
    package's own flags (node: syntax only, with its ROS flags), run frozen vs. sandbox golden dumps and all tests.
    The repository is not touched."""
    t_start = time.time()
    open(console_path(), 'w').close()
    dlog('dry run of iteration %d, edit fingerprint %s' % (iteration(), edit_fingerprint()[:16]))
    ref = TAG if git(R, 'rev-parse', '-q', '--verify', 'refs/tags/' + TAG, check=False) else 'HEAD'
    sb = os.path.join(PIPE_DIR, 'dryrun_iter%d' % iteration())
    shutil.rmtree(sb, ignore_errors=True)
    rels = [os.path.relpath(R[k], R['repo']) for k in ('pkg_root', 'start_script', 'batch_script')]
    trees = {}
    for name in ('frozen', 'patched'):
        trees[name] = os.path.join(sb, name)
        os.makedirs(trees[name])
        sh('git -C %s archive %s %s | tar -x -C %s' % (R['repo'], ref, ' '.join(rels), trees[name]))
    Sf, Sp = sandbox_map(R, trees['frozen']), sandbox_map(R, trees['patched'])
    templates_fill(Sp)
    saved, L.TOUCHED[:] = list(L.TOUCHED), []
    try:
        layers = apply_layers(Sp, only_missing=False)  # every anchor is checked against the frozen sources
        generate(Sp)
        touched = sorted(set(L.TOUCHED))
    finally:
        L.TOUCHED[:] = saved
    out = {'ref': ref, 'layers': layers, 'files': [os.path.relpath(p, trees['patched']) for p in touched]}
    diff = sh(['diff', '-ruN', 'frozen', 'patched'], cwd=sb, check=False).stdout
    open(os.path.join(sb, 'full.diff'), 'w').write(diff)
    out['diff'] = {'path': os.path.join(sb, 'full.diff'), 'sha256': L.sha256_file(os.path.join(sb, 'full.diff')),
                   'files': len(re.findall(r'^diff -ruN ', diff, flags=re.M)),
                   'added': len(re.findall(r'^\+(?!\+\+)', diff, flags=re.M)),
                   'removed': len(re.findall(r'^-(?!--)', diff, flags=re.M))}
    dlog('dry run: %d layers applied in the sandbox, diff %d files +%d/-%d lines' % (
        len(layers), out['diff']['files'], out['diff']['added'], out['diff']['removed']))
    # ---- compilation with the package flags: errors, and warnings that the frozen sources do not have
    cxx, cflags, eig, _, _ = flags_make(R['flags_make'])
    ncxx, nflags, _, nincs, ndefs = flags_make(R['node_flags_make'])
    bins = {k: os.path.join(sb, 'bin_' + k) for k in trees}
    comp = {'errors': [], 'new_warnings': []}
    objs = {k: [] for k in trees}
    test_rel = os.path.relpath(R['test_dir'], R['pkg_root'])
    for k, S in (('frozen', Sf), ('patched', Sp)):
        os.makedirs(bins[k])
        args = cflags + ['-pthread', '-I', os.path.join(S['pkg_root'], 'include'), '-I', S['test_dir'],
                         '-I', Sp['test_dir']] + eig + ['-D%s="%s"' % (R['pkg_dir_macro'], S['pkg_root'])]
        for src in S['core_lib_srcs']:
            o = os.path.join(bins[k], os.path.basename(src) + '.o')
            rc, txt, w = compile_unit(cxx, args, src, o)
            objs[k].append(o)
            out.setdefault('warnings', {}).setdefault(k, {})[os.path.basename(src)] = len(w)
            if rc != 0:
                comp['errors'].append('%s %s:\n%s' % (k, src, txt[-2000:]))
        # node translation units: syntax and warnings with the node target's own include path and defines
        nargs = nflags + [x.replace(os.path.join(R['pkg_root'], 'include'), os.path.join(S['pkg_root'], 'include'))
                          for x in nincs] + ndefs
        for src in S['node_srcs']:
            rc, txt, w = compile_unit(ncxx, nargs, src, syntax_only=True)
            out['warnings'][k][os.path.basename(src)] = len(w)
            if rc != 0:
                comp['errors'].append('%s %s:\n%s' % (k, src, txt[-2000:]))
    for src, n in out['warnings']['patched'].items():
        if n > out['warnings']['frozen'].get(src, 0):
            comp['new_warnings'].append('%s: %d warnings (frozen %d)' % (src, n, out['warnings']['frozen'].get(src, 0)))
    # test programs: frozen gen_golden (tag sources), sandbox gen_golden, existing tests and test_fixes (sandbox)
    fm = re.search(r'foreach\(\s*t\s+([^)]*)\)', open(Sf['cmake']).read())
    existing = fm.group(1).split()
    progs = [('frozen', 'gen_golden')] + [('patched', t) for t in existing + NEW_TESTS]
    for k, t in progs:
        S = Sf if k == 'frozen' else Sp
        args = cflags + ['-pthread', '-I', os.path.join(S['pkg_root'], 'include'), '-I', S['test_dir'],
                         '-I', Sp['test_dir']] + eig + ['-D%s="%s"' % (R['pkg_dir_macro'], S['pkg_root'])]
        src = os.path.join(Sp['test_dir'], t + '.cpp')
        r = subprocess.run([cxx] + args + [src] + objs[k] + ['-o', os.path.join(bins[k], t)], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True)
        # warnings count only where they point into a file this patch wrote (unchanged tests may already warn)
        mine = {os.path.join(trees['patched'], f) for f in out['files']}
        w = [l for l in r.stdout.split('\n') if ': warning:' in l and l.split(':')[0] in mine]
        if w:
            comp['new_warnings'].append('%s/%s: %d warnings' % (k, t, len(w)))
        if r.returncode != 0:
            comp['errors'].append('%s/%s:\n%s' % (k, t, r.stdout[-2000:]))
    out['compile'] = comp
    if comp['errors'] or comp['new_warnings']:
        raise GateFailure('Gate-D compile', 'dry-run compilation failed', comp)
    dlog('dry run: compilation clean (core %d TUs, node %d TUs syntax, %d test programs), no new warnings' % (
        len(Sp['core_lib_srcs']), len(Sp['node_srcs']), len(progs)))
    # ---- tests
    gold = os.path.join(sb, 'golden_frozen.txt')
    sh([os.path.join(bins['frozen'], 'gen_golden'), gold], timeout=1800)
    pd = os.path.join(sb, 'golden_sandbox_flags_off.txt')
    sh([os.path.join(bins['patched'], 'gen_golden'), pd], timeout=1800)
    if open(gold, 'rb').read() != open(pd, 'rb').read():
        a, b = open(gold).read().split('\n'), open(pd).read().split('\n')
        raise GateFailure('Gate-D T0', 'sandbox dump with every flag false differs from the frozen reference',
                          {'diff': list(difflib.unified_diff(a, b, 'frozen', 'sandbox', n=1, lineterm=''))[:60]})
    out['T0_sandbox'] = 'identical (%d bytes)' % os.path.getsize(gold)
    for t in existing:
        r = sh([os.path.join(bins['patched'], t)], check=False, timeout=1800)
        out['existing_' + t] = {'exit': r.returncode, 'pass': r.stdout.count('[PASS]'), 'fail': r.stdout.count('[FAIL]')}
        if r.returncode != 0:
            raise GateFailure('Gate-D tests', 'existing test %s fails in the sandbox' % t, {'tail': r.stdout[-3000:]})
    dlog('dry run: frozen vs sandbox dump identical; existing tests pass in the sandbox; running test_fixes')
    r = sh([os.path.join(bins['patched'], 'test_fixes'), gold], check=False, timeout=7200)
    open(os.path.join(sb, 'test_fixes.out'), 'w').write(r.stdout)
    fails = [l for l in r.stdout.split('\n') if l.startswith('[FAIL]')]
    out['test_fixes'] = {'exit': r.returncode, 'pass': r.stdout.count('[PASS]'), 'fail': len(fails),
                         'info': [l for l in r.stdout.split('\n') if l.startswith('[INFO]')]}
    if r.returncode != 0 or fails:
        raise GateFailure('Gate-D tests', 'test_fixes fails in the sandbox', {'failures': fails})
    out['log_format_functions'] = node_log_format_check(R, Sf['pkg_root'], Sp['node_cpp'])
    out['passed'] = True
    out['fingerprint'] = edit_fingerprint()
    out['seconds'] = round(time.time() - t_start)
    set_state('dryrun', out)
    dlog('dry run passed in %d s: %d PASS / 0 FAIL in test_fixes, T0 identical, log writers unchanged' % (
        out['seconds'], out['test_fixes']['pass']))


REQUIRED_TESTS = ['T0', 'T0b', 'T1', 'T2', 'T2e', 'T3', 'T4', 'T5', 'T5b', 'T6', 'T6b', 'T7', 'T8', 'T9', 'T10', 'T11']
CHECKLIST = {'FX1 (R1a, R1c, R1d, R1e)': ['r1a_reachable_cone', 'r1c_heading_memory', 'r1d_feasible_cone', 'r1e_lowspeed_angle'],
             'R1b retained for comparison': ['r1b_continuous_slack'],
             'FX2 (R2a, R2b)': ['r2a_kappa_decay', 'r2b_kappa_local_excluded'],
             'FX3 (R3 positive increments)': ['r3_relevant_trigger'],
             'FX4 (R4, R4-m, R4-new evidence, R4-s override)': ['r4_switch_hysteresis', 'r4m_dwell_free_only',
                                                               'r4n_leaky_evidence', 'r4s_safety_override'],
             'FX5 (R5)': ['r5_mode_gating'], 'FX6 (S1)': ['s1_eps_constraint']}


def dryrun_check(R):
    """Phase A: parse the iteration's dry-run console log and the sandbox test output; check every anchor layer, the
    compilation result, every required test ID and the implementation checklist (flags declared, default false)."""
    dr = state().get('dryrun', {})
    con = open(console_path()).read() if os.path.isfile(console_path()) else ''
    sb = os.path.join(PIPE_DIR, 'dryrun_iter%d' % iteration())
    problems, out = [], {'console': console_path()}
    m = re.search(r'dry run: (\d+) layers applied in the sandbox', con)
    out['layers'] = '%s of %d' % (m.group(1) if m else 0, len(TN.layers()))
    if not m or int(m.group(1)) != len(TN.layers()):
        problems.append('anchor layers applied: %s' % out['layers'])
    for needle in ('compilation clean', 'frozen vs sandbox dump identical', 'dry run passed'):
        if needle not in con:
            problems.append('console log lacks "%s"' % needle)
    if dr.get('fingerprint') != edit_fingerprint():
        problems.append('dry-run fingerprint does not match the current edit sources')
    tests = {}
    tf = os.path.join(sb, 'test_fixes.out')
    for l in (open(tf).read().split('\n') if os.path.isfile(tf) else []):
        mm = re.match(r'\[(PASS|FAIL)\] (T\d+[a-z]?)\b', l)
        if mm:
            tests.setdefault(mm.group(2), []).append(mm.group(1))
    out['tests'] = {t: '%d PASS / %d FAIL' % (v.count('PASS'), v.count('FAIL')) for t, v in sorted(tests.items())}
    for t in REQUIRED_TESTS:
        if t not in tests or 'FAIL' in tests[t]:
            problems.append('test %s: %s' % (t, out['tests'].get(t, 'missing')))
    sw_text = open(os.path.join(sb, 'patched', os.path.relpath(R['switches_h'], R['repo']))).read()
    tests_text = open(os.path.join(sb, 'patched', os.path.relpath(R['test_dir'], R['repo']), 'test_fixes.cpp')).read()
    out['checklist'] = {}
    for item, flags in CHECKLIST.items():
        miss = [f for f in flags if not re.search(r'\bbool\s+%s\s*=\s*false;' % f, sw_text)]
        out['checklist'][item] = 'declared, default false' if not miss else 'MISSING %s' % miss
        if miss:
            problems.append('%s: %s' % (item, miss))
    for t in ('T2e', 'T11'):
        if '// ---- %s ' % t not in tests_text:
            problems.append('test %s not in test_fixes.cpp' % t)
    out['problems'] = problems
    set_state('dryrun_check', out)
    if problems:
        raise GateFailure('Gate-A dry-run check', '%d dry-run check failures' % len(problems), out)
    log('Phase A: dry-run console and sandbox tests verified (%s layers, %d test IDs, checklist complete)' % (
        out['layers'], len(REQUIRED_TESTS)))


# ---------------------------------------------------------------- stage 1: deployment of the dry-run result
def stage1(R):
    ensure_branch(R)
    dr = state().get('dryrun', {})
    if not dr.get('passed') or dr.get('fingerprint') != edit_fingerprint() or 'dryrun_check' not in state().get('done_stages', []):
        raise GateFailure('Gate-1 precondition', 'no passed and checked dry run for the current edit sources')
    templates_fill(R)
    applied = apply_layers(R, only_missing=True)
    generate(R)
    sync_copy(R, commit_now=False)
    h = commit(R, L.TOUCHED, 'Iteration %d: apply %s; regenerate tests and presets' % (
        iteration(), ', '.join(applied) or 'no new edit layer'))
    # the branch must now hold exactly the files the dry run produced
    sb = os.path.join(PIPE_DIR, 'dryrun_iter%d' % iteration(), 'patched')
    bad = [f for f in dr['files'] if open(os.path.join(sb, f), 'rb').read() != open(os.path.join(R['repo'], f), 'rb').read()]
    if bad:
        raise GateFailure('Gate-1 deployment', 'branch files differ from the dry-run result', {'files': bad})
    log('stage1: layers applied %s; %d generated/edited files identical to the dry run (commit %s)' % (applied, len(dr['files']), h))
    integrity(R, 'stage1')


# ---------------------------------------------------------------- stage 2: build + offline tests
def stage2(R):
    ensure_branch(R)
    out = {}
    log('stage2: building %s with %s' % (R['pkg_name'], ' '.join(R['build_cmd'])))
    b = sh('source %s && %s' % (R['devel_setup'], ' '.join(R['build_cmd'])), cwd=R['ws'], check=False, timeout=3600)
    open(os.path.join(PIPE_DIR, 'build.log'), 'w').write(b.stdout)
    if b.returncode != 0:
        raise GateFailure('Gate-2 build', 'build failed (exit %d), tail:\n%s' % (b.returncode, b.stdout[-3000:]))
    frozen = os.path.join(PIPE_DIR, 'frozen_tree')
    shutil.rmtree(frozen, ignore_errors=True)
    os.makedirs(frozen)
    pkg_rel = os.path.relpath(R['pkg_root'], R['repo'])
    sh('git -C %s archive %s %s | tar -x -C %s' % (R['repo'], TAG, pkg_rel, frozen))
    fpkg = os.path.join(frozen, pkg_rel)
    fm = re.search(r'foreach\(\s*t\s+([^)]*)\)', open(os.path.join(fpkg, os.path.basename(R['cmake']))).read())
    existing = fm.group(1).split()
    out['existing_tests'] = existing
    for t in existing:
        r = sh([os.path.join(R['bin_dir'], t)], check=False, timeout=1800)
        open(os.path.join(PIPE_DIR, '%s.out' % t), 'w').write(r.stdout)
        out[t] = {'exit': r.returncode, 'pass': r.stdout.count('[PASS]'), 'fail': r.stdout.count('[FAIL]')}
        log('existing test %s: exit %d, %d PASS, %d FAIL' % (t, r.returncode, out[t]['pass'], out[t]['fail']))
        if r.returncode != 0:
            raise GateFailure('Gate-2 existing tests', '%s failed:\n%s' % (t, r.stdout[-3000:]))
    cxx, cflags, eig, _, _ = flags_make(R['flags_make'])
    tf = os.path.join(os.path.dirname(os.path.dirname(R['flags_make'])), 'test_fixes.dir', 'flags.make')
    if flags_make(tf)[1] != cflags:
        raise GateFailure('Gate-T0', 'test_fixes and %s are compiled with different flags' % R['core_lib'])
    gen = os.path.join(PIPE_DIR, 'gen_golden_frozen')
    fsrc = [os.path.join(fpkg, os.path.relpath(s, R['pkg_root'])) for s in R['core_lib_srcs']]
    test_rel = os.path.relpath(R['test_dir'], R['pkg_root'])
    sh([cxx] + cflags + ['-pthread', '-I', os.path.join(fpkg, 'include'), '-I', os.path.join(fpkg, test_rel),
                         '-I', R['test_dir']] + eig + ['-D%s="%s"' % (R['pkg_dir_macro'], fpkg)] + fsrc +
       [os.path.join(R['test_dir'], 'gen_golden.cpp'), '-o', gen], timeout=1800)
    gold = os.path.join(PIPE_DIR, 'golden_frozen_v1.txt')
    sh([gen, gold], timeout=1800)
    out['golden'] = {'path': gold, 'bytes': os.path.getsize(gold), 'sha256': L.sha256_file(gold), 'flags': ' '.join(cflags)}
    log('frozen reference from %s: %d bytes (flags %s)' % (TAG, out['golden']['bytes'], ' '.join(cflags)))
    branch_dump = os.path.join(PIPE_DIR, 'golden_branch_flags_off.txt')
    sh([os.path.join(R['bin_dir'], 'gen_golden'), branch_dump], timeout=1800)
    if open(gold, 'rb').read() != open(branch_dump, 'rb').read():
        a, b = open(gold).read().split('\n'), open(branch_dump).read().split('\n')
        diff = list(difflib.unified_diff(a, b, 'thesis-frozen-v1', BRANCH + ' (flags off)', n=1, lineterm=''))[:80]
        raise GateFailure('Gate-T0', 'branch dump with every flag false differs from the frozen reference', {'diff': diff})
    out['T0_branch_gen_golden'] = 'identical'
    out['log_format_functions'] = node_log_format_check(R, fpkg, R['node_cpp'])
    log('Gate-T0 log format: frozen writer lines unchanged in %s' % out['log_format_functions'])
    log('running test_fixes (T0-T10)')
    r = sh([os.path.join(R['bin_dir'], 'test_fixes'), gold], check=False, timeout=7200)
    open(os.path.join(PIPE_DIR, 'test_fixes.out'), 'w').write(r.stdout)
    lines = r.stdout.split('\n')
    t0 = [l for l in lines if re.match(r'\[(PASS|FAIL)\] T0 all fixes off', l)]
    fails = [l for l in lines if l.startswith('[FAIL]')]
    out['test_fixes'] = {'exit': r.returncode, 'pass': r.stdout.count('[PASS]'), 'fail': len(fails), 'T0': t0,
                         'info': [l for l in lines if l.startswith('[INFO]')]}
    if not t0 or not t0[0].startswith('[PASS]'):
        raise GateFailure('Gate-T0', 'T0 failed: %s' % t0)
    if r.returncode != 0 or fails:
        raise GateFailure('Gate-2 new tests', 'test_fixes failures:\n%s' % '\n'.join(fails))
    log('test_fixes: %d PASS, 0 FAIL; Gate-T0 passed' % out['test_fixes']['pass'])
    sc = open(R['scenario_file']).read()
    dmin = re.search(r'--d_min\s+(\S+)', sc).group(1)
    for seeds in ('%d-%d' % (V1_SEEDS[0], V1_SEEDS[-1]), '%d-%d' % (V2_SEEDS[0], V2_SEEDS[-1]), '%d-%d' % (V3_SEEDS[0], V3_SEEDS[-1])):
        c = sh(['python3', R['check_scenarios'], R['scenario_file'], '--d_min', dmin, '--seeds', seeds], check=False)
        if c.returncode != 0:
            raise GateFailure('Gate-2 scenarios', 'check_scenarios failed for seeds %s:\n%s' % (seeds, c.stdout[-2000:]))
    log('scenario check passed for the V1/V2/V3 seeds')
    set_state('stage2_result', out)
    integrity(R, 'stage2')


# ---------------------------------------------------------------- simulation helpers
def preset_flags(R, name):
    y = A._yaml(os.path.join(R['fixes_dir'], name + '.yaml'))
    return {k: bool(v) for k, v in (y.get('fixes') or {}).items()}


def run_batch(R, out, tag, preset, seeds, variants, groups):
    out = os.path.abspath(out)
    inside_frozen = (os.path.realpath(out) + os.sep).startswith(os.path.realpath(R['frozen_out']) + os.sep)
    if not out.startswith(STATE_ROOT + os.sep) or inside_frozen:
        raise GateFailure('Gate-I output', 'refusing to write outside %s: %s' % (STATE_ROOT, out))
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, OUT=out, TAG=tag, FIXES_FILE=preset, SEEDS=' '.join(map(str, seeds)), VARIANTS=' '.join(variants),
               GROUPS_SEL=' '.join(groups), MAX_ATTEMPTS='3', RECORD='0')
    log('batch %s: preset %s seeds %s variants %s groups %s' % (out, preset, seeds, variants, groups))
    with open(os.path.join(out, 'pipeline_batch.log'), 'a') as f:
        r = subprocess.run(['bash', R['batch_script']], cwd=R['repo'], env=env, stdout=f, stderr=subprocess.STDOUT,
                           timeout=len(seeds) * len(variants) * len(groups) * 3 * 600 + 600)
    if r.returncode != 0:
        raise GateFailure('Gate-S driver', 'batch driver aborted (exit %d): %s' % (r.returncode, out))


def evaluate_arm(R, out, tag, preset, cells):
    """post-processing of one arm + extras; checks completeness and that the preset was really loaded."""
    A.load_pp(R)
    flags = preset_flags(R, preset)
    rows = A.post_process_arm(R, out, tag)
    valid = {(r['variant'], r['group'], str(r['seed'])): r for r in rows if str(r['valid']) == '1'}
    missing = sorted(c for c in cells if c not in valid)
    ext, wrong = {}, []
    for c in cells:
        if c in valid:
            ext[c] = A.run_extras(valid[c]['log_dir'], window=R4_WINDOW)
            if not A.fixes_match(ext[c]['fixes'], flags):
                wrong.append((c, ext[c]['fixes']))
    if wrong:
        raise GateFailure('Gate-S config', 'runs did not load preset %s' % preset, {'runs': [str(w) for w in wrong]})
    return {'rows': valid, 'ext': ext, 'missing': missing, 'flags': flags}


def unsafe_runs(ev):
    return [{'cell': c, 'end_reason': r['end_reason'], 'M4': r.get('M4_success'), 'log_dir': r['log_dir']}
            for c, r in ev['rows'].items() if A.collided(r) or not A.success(r)]


def summarize_arm(ev):
    out = []
    for c in sorted(ev['rows']):
        r, x = ev['rows'][c], ev['ext'][c]
        out.append({'variant': c[0], 'group': c[1], 'seed': c[2], 'end_reason': r['end_reason'], 'success': A.success(r),
                    'M1_mean': r['M1_mean_ms'], 'M1_p95': r['M1_p95_ms'], 'M2': r['M2_path_length_m'],
                    'M3': r['M3_max_turn_deg_1s'], 'M5': r['M5_mean_m'], 'M6': r['M6_flip_backs'],
                    'stall_breaks': r['stall_breaks'], 'min_static': r['min_static_surface_dist_odom_m'],
                    'min_dyn': r['min_dyn_surface_dist_odom_m'], 'duration': r['duration_s'],
                    'low_speed_share': round(x['low_speed_share'], 4), 'path_switches': x['path_switches'],
                    'r4_held_under_threat': len(x['r4_held_under_threat']), 'r4_overrides': x['r4_overrides'],
                    'r4_held': x['r4_held'], 'gap_time': round(x['gap_time'], 2), 'gap_median_speed': round(x['gap_median_speed'], 3),
                    'gap_low_share': round(x['gap_low_share'], 3),
                    'replans': ' '.join('%s:%d' % kv for kv in sorted(x['replans'].items()))})
    return out


def write_series(out, ev):
    d = os.path.join(out, 'series')
    os.makedirs(d, exist_ok=True)
    for c, x in ev['ext'].items():
        if x['dyn_series']:
            with open(os.path.join(d, '%s_%s_seed%s_dyn.csv' % c), 'w') as f:
                f.write('t,min_dyn_surface_dist_odom_m\n' + ''.join('%.3f,%.4f\n' % p for p in x['dyn_series']))


def r4_delays(ev, ref):
    """dwell-held switches under a threat after which the dynamic clearance fell more than DIST_MARGIN below the
    diag_only run of the same variant and seed."""
    out = []
    for c, x in ev['ext'].items():
        if c not in ref['ext'] or not x['r4_held_under_threat']:
            continue
        ref_min = ref['ext'][c]['dyn_min']
        for h in x['r4_held_under_threat']:
            if h['win_min_dyn'] == h['win_min_dyn'] and ref_min == ref_min and h['win_min_dyn'] < ref_min - DIST_MARGIN:
                out.append({'cell': c, 't': h['t'], 'window_min_dyn': h['win_min_dyn'], 'reference_min_dyn': ref_min})
    return out


# ---------------------------------------------------------------- V1 smoke ladder
GAP_FACTOR = 2.0  # pre-registered: narrow-gap traversal at most twice the diag_only time of the same seed


def v1_gates(ev, ref, name):
    """Pre-registered V1 gates: (1) no collision and every goal reached; (2) minimum dynamic clearance not more than
    DIST_MARGIN below diag_only on the same variant and seed; (3) no prolonged crawl in the M1_three_way narrow gap."""
    fails = [{'gate': 'collision or mission failure', **u} for u in unsafe_runs(ev)]
    for c, r in ev['rows'].items():
        if c not in ref['rows']:
            continue
        a, b = A.fnum(r['min_dyn_surface_dist_odom_m']), A.fnum(ref['rows'][c]['min_dyn_surface_dist_odom_m'])
        if a == a and b == b and a < b - DIST_MARGIN:
            fails.append({'gate': 'dynamic clearance', 'cell': c, 'min_dyn': a, 'diag_only': b})
        if c[0] == 'M1_three_way':
            ta, tb = ev['ext'][c]['gap_time'], ref['ext'][c]['gap_time']
            if ta == ta and tb == tb and ta > GAP_FACTOR * tb:
                fails.append({'gate': 'narrow-gap crawl', 'cell': c, 'gap_time': ta, 'diag_only': tb,
                              'gap_median_speed': ev['ext'][c]['gap_median_speed'], 'gap_low_share': ev['ext'][c]['gap_low_share']})
    return fails


def v1(R):
    ensure_branch(R)
    integrity(R, 'v1 start')
    cells = [(v, g, str(s)) for v in V1_VARIANTS for g in V1_GROUPS for s in V1_SEEDS]
    st = state().get('v1', {'steps': {}})
    defaults = TN.default_presets()
    root = os.path.join(out_root(), 'v1')
    ref = None
    for name, units, r4m in TN.LADDER:
        out = os.path.join(root, name)
        if name in st['steps'] and st['steps'][name].get('done'):
            continue
        flags = TN.flags_of(units, r4m=bool(r4m))
        if defaults[name][0] != flags:
            raise GateFailure('Gate-V1 config', 'preset %s differs from the ladder definition' % name)
        for s in V1_SEEDS:
            run_batch(R, out, 'v1', name, [s], V1_VARIANTS, V1_GROUPS)
        ev = evaluate_arm(R, out, 'v1', name, cells)
        write_series(out, ev)
        A.write_rows(os.path.join(out, 'v1_summary.csv'), summarize_arm(ev))
        if ref is None:
            ref = ev if name == 'diag_only' else evaluate_arm(R, os.path.join(root, 'diag_only'), 'v1', 'diag_only', cells)
        fails = v1_gates(ev, ref, name) + ([{'gate': 'infrastructure', 'missing': ev['missing']}] if ev['missing'] else [])
        rec = {'preset': name, 'units': units, 'flags': flags, 'gate_failures': fails, 'r4_delays': r4_delays(ev, ref),
               'summary': summarize_arm(ev)}
        st['steps'][name] = rec
        set_state('v1', st)
        if fails:  # no fallback, no retry: stop with the data
            raise GateFailure('Gate-V1', '%d V1 gate failures at %s' % (len(fails), name), {'failures': fails, 'step': name})
        rec['done'] = True
        set_state('v1', st)
        log('V1 %s: %d runs, gates passed, R4 overrides %d, dwell-held switches under threat %d' % (
            name, len(ev['rows']), sum(x['r4_overrides'] for x in ev['ext'].values()), len(rec['r4_delays'])))
    st['all_on'] = defaults['all_on'][0]
    st['done'] = True
    set_state('v1', st)
    integrity(R, 'v1 end')


# ---------------------------------------------------------------- V2 screening (3 seeds)
def v2_gates(off, on, cells):
    fails = []
    for c in cells:
        if c in on['rows'] and A.collided(on['rows'][c]):
            fails.append({'gate': 'collision', 'cell': c})
    for g in sorted({c[1] for c in cells}):
        k_off = sum(A.success(off['rows'][c]) for c in cells if c[1] == g and c in off['rows'])
        k_on = sum(A.success(on['rows'][c]) for c in cells if c[1] == g and c in on['rows'])
        if k_on < k_off:
            fails.append({'gate': 'success', 'group': g, 'off': k_off, 'on': k_on})
        p_off = [A.fnum(off['rows'][c]['M1_p95_ms']) for c in cells if c[1] == g and c in off['rows']]
        p_on = [A.fnum(on['rows'][c]['M1_p95_ms']) for c in cells if c[1] == g and c in on['rows']]
        if p_off and p_on and sum(p_on) / len(p_on) > M1_FACTOR * sum(p_off) / len(p_off):
            fails.append({'gate': 'M1_p95', 'group': g, 'off_mean': sum(p_off) / len(p_off), 'on_mean': sum(p_on) / len(p_on)})
    for vg in sorted({(c[0], c[1]) for c in cells}):
        for col in ('min_static_surface_dist_odom_m', 'min_dyn_surface_dist_odom_m'):
            a = [A.fnum(off['rows'][c][col]) for c in cells if (c[0], c[1]) == vg and c in off['rows']]
            b = [A.fnum(on['rows'][c][col]) for c in cells if (c[0], c[1]) == vg and c in on['rows']]
            a, b = [x for x in a if x == x], [x for x in b if x == x]
            if a and b and min(b) < min(a) - DIST_MARGIN:
                fails.append({'gate': 'clearance', 'cell': vg, 'metric': col, 'off_min': min(a), 'on_min': min(b)})
    return fails


def v2_stats(off, on, cells):
    """Descriptive per variant: medians per group x arm, and the paired g6 - g3 gap (C2 + C3 jointly) off vs on."""
    import statistics as stt
    rows = []
    metrics = [('M1_p95_ms', 'M1 P95 [ms]'), ('M2_path_length_m', 'M2 [m]'), ('M3_max_turn_deg_1s', 'M3 [deg]'),
               ('M5_mean_m', 'M5 mean [m]'), ('M6_flip_backs', 'M6'), ('stall_breaks', 'stall breaks')]
    for v in sorted({c[0] for c in cells}):
        for m, label in metrics:
            rec = {'variant': v, 'metric': label}
            for arm, ev in (('off', off), ('on', on)):
                for g in V2_GROUPS:
                    vals = [A.fnum(ev['rows'][c][m]) for c in cells if c[0] == v and c[1] == g and c in ev['rows']]
                    vals = [x for x in vals if x == x]
                    rec['%s_%s' % (g, arm)] = round(stt.median(vals), 3) if vals else ''
            for lab, (ea, ga), (eb, gb) in (('gap_off g6off-g3off', (off, 'g6'), (off, 'g3')),
                                            ('gap_on g6on-g3off', (on, 'g6'), (off, 'g3')),
                                            ('g6on-g6off', (on, 'g6'), (off, 'g6'))):
                d = []
                for s in V2_SEEDS:
                    ca, cb = (v, ga, str(s)), (v, gb, str(s))
                    if ca in ea['rows'] and cb in eb['rows']:
                        x, y = A.fnum(ea['rows'][ca][m]), A.fnum(eb['rows'][cb][m])
                        if x == x and y == y:
                            d.append(x - y)
                rec[lab] = round(stt.median(d), 3) if d else ''
            rows.append(rec)
    return rows


# pre-registered side-effect screen (only after every V2 safety gate passed): ON worse than OFF in >= 2 variants of a
# group for the same metric; median for M2 (relative), M3 and M6 (absolute), sum over seeds for stall breaks
SIDE = {'M2_path_length_m': ('rel', 0.05), 'M3_max_turn_deg_1s': ('abs', 15.0), 'M6_flip_backs': ('abs', 1.0),
        'stall_breaks': ('sum', 2.0)}


def worse(off, on, cells, g, v, m):
    import statistics as stt
    a = [A.fnum(off['rows'][c][m]) for c in cells if c[:2] == (v, g) and c in off['rows']]
    b = [A.fnum(on['rows'][c][m]) for c in cells if c[:2] == (v, g) and c in on['rows']]
    a, b = [x for x in a if x == x], [x for x in b if x == x]
    if not a or not b:
        return False
    kind, th = SIDE[m]
    if kind == 'sum':
        return sum(b) - sum(a) > th
    d = stt.median(b) - stt.median(a)
    return d > th * stt.median(a) if kind == 'rel' else d > th


def side_effects(off, on, cells):
    hits = {}
    for g in sorted({c[1] for c in cells}):
        for v in sorted({c[0] for c in cells}):
            for m in SIDE:
                if worse(off, on, cells, g, v, m):
                    hits.setdefault('%s|%s' % (g, m), []).append(v)
    return {k: vs for k, vs in hits.items() if len(vs) >= 2}


def bisect_side_effect(R, st, off, hits, cells):
    """Binary search over the repair units of all_on for the one that reproduces the flagged side effect."""
    flags = preset_flags(R, 'all_on')
    cand = [u for u in TN.UNITS if any(flags.get(f) for f in TN.UNITS[u])]
    groups = sorted({k.split('|')[0] for k in hits})
    variants = sorted({v for vs in hits.values() for v in vs})
    sub = [c for c in cells if c[0] in variants and c[1] in groups]
    n = 0
    while len(cand) > 1:
        half = cand[:len(cand) // 2]
        name = 'v2_bisect_%d' % n
        TN.write_preset(R, name, TN.flags_of(half, r4m=bool(flags.get('r4m_dwell_free_only'))), 'V2 bisection: %s' % '+'.join(half))
        commit(R, [os.path.join(R['fixes_dir'], name + '.yaml')], 'V2: bisection preset %s' % name)
        out = os.path.join(out_root(), 'v2', name)
        for s in V2_SEEDS:
            run_batch(R, out, 'v2', name, [s], variants, groups)
        ev = evaluate_arm(R, out, 'v2', name, sub)
        repro = any(sum(worse(off, ev, sub, k.split('|')[0], v, k.split('|')[1]) for v in vs) * 2 >= len(vs) for k, vs in hits.items())
        st.setdefault('bisect', []).append({'preset': name, 'units': half, 'cells': sub, 'reproduced': repro,
                                            'unsafe': unsafe_runs(ev)})
        set_state('v2', st)
        cand = half if repro else cand[len(cand) // 2:]
        n += 1
    return cand[0]


def v2(R):
    ensure_branch(R)
    integrity(R, 'v2 start')
    pp = A.load_pp(R)
    st = state().get('v2', {})
    cells = [(v, g, str(s)) for v in pp.VARIANTS for g in V2_GROUPS for s in V2_SEEDS]
    root = os.path.join(out_root(), 'v2')
    rnd = st.get('round', 0)
    outs = {'off': os.path.join(root, 'all_off'), 'on': os.path.join(root, 'all_on' if rnd == 0 else 'all_on_r%d' % rnd)}
    for s in V2_SEEDS:  # arms interleaved per seed, so drift of the machine state hits both arms alike
        run_batch(R, outs['off'], 'v2', 'all_off', [s], pp.VARIANTS, V2_GROUPS)
        run_batch(R, outs['on'], 'v2', 'all_on', [s], pp.VARIANTS, V2_GROUPS)
    off = evaluate_arm(R, outs['off'], 'v2', 'all_off', cells)
    on = evaluate_arm(R, outs['on'], 'v2', 'all_on', cells)
    for ev, o in ((off, outs['off']), (on, outs['on'])):
        write_series(o, ev)
        A.write_rows(os.path.join(o, 'v2_summary.csv'), summarize_arm(ev))
    stats = v2_stats(off, on, cells)
    A.write_rows(os.path.join(root, 'v2_stats_r%d.csv' % rnd), stats)
    fails = v2_gates(off, on, cells) + ([{'gate': 'infrastructure', 'off': off['missing'], 'on': on['missing']}]
                                        if off['missing'] or on['missing'] else [])
    st.update(round=rnd, on_flags=on['flags'], gate_failures=fails, stats=stats, off_summary=summarize_arm(off),
              on_summary=summarize_arm(on), unsafe_off=unsafe_runs(off), unsafe_on=unsafe_runs(on))
    set_state('v2', st)
    if fails:  # no fault identification, no retry: the pipeline stops with the data
        raise GateFailure('Gate-V2 safety', '%d V2 gate failures (round %d)' % (len(fails), rnd), {'failures': fails})
    hits = side_effects(off, on, cells)
    st['side_effects_r%d' % rnd] = hits
    set_state('v2', st)
    if hits and rnd == 0:
        culprit = bisect_side_effect(R, st, off, hits, cells)
        units = [u for u in TN.UNITS if any(on['flags'].get(f) for f in TN.UNITS[u]) and u != culprit]
        TN.write_preset(R, 'all_on', TN.flags_of(units, r4m=bool(on['flags'].get('r4m_dwell_free_only')) and 'FX4' in units),
                        'V2: %s disabled after the side-effect bisection' % culprit)
        commit(R, [os.path.join(R['fixes_dir'], 'all_on.yaml')], 'V2: %s disabled (side effect located by bisection: %s)' % (
            culprit, sorted(hits)))
        st.update(round=1, disabled_units=[culprit])
        set_state('v2', st)
        log('V2: side effect %s located in %s; all_on updated, ON arm re-run' % (sorted(hits), culprit))
        return v2(R)
    st['passed'] = True
    set_state('v2', st)
    log('V2 gates passed (round %d); side effects: %s' % (rnd, hits or 'none'))
    integrity(R, 'v2 end')


# ---------------------------------------------------------------- V3 confirmation (manual only)
def v3(R, with_g5=False):
    ensure_branch(R)
    integrity(R, 'v3 start')
    if not state().get('v2', {}).get('passed'):
        raise GateFailure('Gate-V3 precondition', 'V2 gates have not passed')
    pp = A.load_pp(R)
    groups = V3_GROUPS + (V3_OPTIONAL if with_g5 else [])
    cells = [(v, g, str(s)) for v in pp.VARIANTS for g in groups for s in V3_SEEDS]
    root = os.path.join(out_root(), 'v3')
    outs = {'off': os.path.join(root, 'all_off'), 'on': os.path.join(root, 'all_on')}
    presets = {'off': 'all_off', 'on': 'all_on'}
    for s in V3_SEEDS:
        for arm in ('off', 'on'):
            run_batch(R, outs[arm], 'v3', presets[arm], [s], pp.VARIANTS, groups)
    evs = {arm: evaluate_arm(R, outs[arm], 'v3', presets[arm], cells) for arm in outs}
    recs = A.v3_tests({arm: list(evs[arm]['rows'].values()) for arm in evs})
    A.write_rows(os.path.join(root, 'v3_tests.csv'), recs)
    for arm in evs:
        write_series(outs[arm], evs[arm])
        A.write_rows(os.path.join(outs[arm], 'v3_summary.csv'), summarize_arm(evs[arm]))
    set_state('v3', {'done': True, 'missing': {a: evs[a]['missing'] for a in evs}, 'on_flags': evs['on']['flags'],
                     'unsafe': {a: unsafe_runs(evs[a]) for a in evs}, 'tests_csv': os.path.join(root, 'v3_tests.csv')})
    log('V3 done: %d tests -> %s' % (len(recs), os.path.join(root, 'v3_tests.csv')))
    integrity(R, 'v3 end')


# ---------------------------------------------------------------- driver
def write_report(R, msg):
    import report
    report.write(R, state(), TAG, BRANCH, out_root())
    commit(R, L.TOUCHED, msg)
    integrity(R, 'report')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', default=None)
    ap.add_argument('cmd', choices=['run', 'dryrun', 'run-v3', 'new-iteration', 'stop-report', 'integrity'])
    ap.add_argument('--until', choices=STAGES, default='report')
    ap.add_argument('--confirm-v3', action='store_true')
    ap.add_argument('--with-g5', action='store_true')
    ap.add_argument('--note', default='')
    a = ap.parse_args()
    repo = a.repo or sh(['git', 'rev-parse', '--show-toplevel']).stdout.strip()
    os.makedirs(PIPE_DIR, exist_ok=True)
    cur = a.cmd
    try:
        if a.cmd == 'new-iteration':
            s = state()
            hist = s.get('history', [])
            hist.append({'iteration': iteration(), 'closed': L.now(), 'note': a.note, **{k: s.get(k) for k in PER_ITER if k in s}})
            for k in PER_ITER:
                s.pop(k, None)
            s['history'], s['iteration'], s['iteration_started'] = hist, iteration() + 1, time.time()
            save_json('state.json', s)
            log('iteration %d opened: %s' % (s['iteration'], a.note))
            return
        if a.cmd in ('dryrun', 'run'):
            s = state()
            if 'dryrun' in s.get('done_stages', []) and s.get('dryrun', {}).get('fingerprint') != edit_fingerprint():
                # the edit sources changed after the dry run: it is archived as superseded, and every later stage of
                # this iteration has to run again on the new sources
                s['dryrun_superseded'] = s.get('dryrun_superseded', []) + [dict(s['dryrun'], superseded=L.now())]
                s['done_stages'] = []
                for k in ('dryrun', 'dryrun_check', 'stage2_result', 'v1', 'v2'):
                    s.pop(k, None)
                save_json('state.json', s)
                log('dry run of iteration %d superseded: the edit sources changed' % iteration())
            for stg in ('dryrun', 'dryrun_check'):
                if stg in state().get('done_stages', []):
                    continue
                cur = stg
                log('==== %s (iteration %d) ====' % (stg, iteration()))
                globals()[stg](discover_only(repo))
                set_state('done_stages', state().get('done_stages', []) + [stg])
            if a.cmd == 'dryrun' or a.until in ('dryrun', 'dryrun_check'):
                return
        cur = 'stage0'
        R = stage0(repo)
        if a.cmd == 'integrity':
            return
        if 'stage1' in state().get('done_stages', []):
            sync_copy(R)
        if a.cmd == 'stop-report':
            cur = 'stop-report'
            write_report(R, 'Report (iteration %d): pipeline state at a protocol gate' % iteration())
            return
        if a.cmd == 'run-v3':
            cur = 'v3'
            if not a.confirm_v3:
                raise GateFailure('Gate-V3 confirmation', 'V3 needs explicit manual confirmation (--confirm-v3)')
            v3(R, with_g5=a.with_g5)
            write_report(R, 'Report (iteration %d) after V3' % iteration())
            return
        done = state().get('done_stages', [])
        if 'stage0' not in done:
            set_state('done_stages', done + ['stage0'])
        for stg in STAGES[STAGES.index('stage1'):max(STAGES.index(a.until), STAGES.index('stage0')) + 1]:
            cur = stg
            if stg in state().get('done_stages', []):
                log('%s already completed (iteration %d), skipped' % (stg, iteration()))
                continue
            log('==== %s (iteration %d) ====' % (stg, iteration()))
            if stg == 'report':
                write_report(R, 'Report (iteration %d): V1/V2 screening, V3 awaiting manual confirmation' % iteration())
            else:
                globals()[stg](R)
            set_state('done_stages', state().get('done_stages', []) + [stg])
        log('pipeline finished up to %s (iteration %d)' % (a.until, iteration()))
    except GateFailure as e:
        L.fail_report(e, '%s_iter%d' % (cur, iteration()))
    except Exception:  # any unexpected fault stops the pipeline with a report as well
        import traceback
        L.fail_report(GateFailure('unexpected error', traceback.format_exc()), '%s_iter%d' % (cur, iteration()))


if __name__ == '__main__':
    main()
