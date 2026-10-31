#!/usr/bin/env python3
"""fix_coupling_artifacts pipeline: baseline protection, introspective discovery, FX1-FX6 implementation behind
default-false flags, build + offline tests (Gate-T0), V1 smoke ladder, V2 screening, V3 confirmation, final report.

  pipeline.py [--repo <git repo>] run [--until stage0|stage1|stage2|v1|v2|v3|report]
  pipeline.py [--repo <git repo>] integrity

Every gate failure writes ~/.ros/ablation_fix/pipeline/GATE_FAILURE_<stage>.md and exits with code 2.
Stages already completed (state.json) are skipped, so a stopped run resumes where it stopped."""
import argparse
import difflib
import glob
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# never write bytecode caches: the project's scripts directory holds tracked __pycache__ files
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
sys.path.insert(0, HERE)
import fixlib as L  # noqa: E402
from fixlib import log, sh, GateFailure, save_json, load_json, PIPE_DIR, STATE_ROOT  # noqa: E402
import discover as D  # noqa: E402
import transforms_base as TB  # noqa: E402
import transforms_new as TN  # noqa: E402
import analysis as A  # noqa: E402

TAG = 'thesis-frozen-v1'
TAG_MSG = 'frozen baseline for thesis chapter4'
BRANCH = 'fix_coupling_artifacts'
NEW_TESTS = ['gen_golden', 'test_fixes']
STAGES = ['stage0', 'stage1', 'stage2', 'v1', 'v2', 'v3', 'report']
V1_SEEDS, V1_VARIANTS, V1_GROUPS = [101, 102, 103], ['S3_climb_ne', 'head_on', 'M1_three_way'], ['g6']
V2_SEEDS, V2_GROUPS = [1, 2, 3, 4, 5], ['g3', 'g5', 'g6']
V3_SEEDS, V3_GROUPS = list(range(21, 41)), ['g3', 'g6']
DIST_MARGIN, M1_FACTOR, R4_WINDOW = 0.05, 1.2, 2.5
TRAILER = 'Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>'
CODE_EXT = ('.cpp', '.cc', '.h', '.hpp', '.launch', '.yaml', '.sh', 'CMakeLists.txt', '.xml')


def state():
    return load_json('state.json', {})


def set_state(key, val):
    s = state()
    s[key] = val
    save_json('state.json', s)


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
def stage0(repo):
    R = {'repo': repo}
    cur = git(R, 'rev-parse', '--abbrev-ref', 'HEAD')
    porcelain = sh(['git', 'status', '--porcelain'], cwd=repo).stdout.rstrip('\n').split('\n')
    mods = [l[3:] for l in porcelain if l and not l.startswith('??')]
    blocked = [m for m in mods if m.endswith(CODE_EXT)]
    if blocked and cur != BRANCH:
        raise GateFailure('Gate-0 git', 'tracked source/config files have uncommitted changes; the tag would not '
                                        'describe the code that ran: %s' % blocked)
    if not git(R, 'rev-parse', '-q', '--verify', 'refs/tags/' + TAG, check=False):
        if cur == BRANCH:
            raise GateFailure('Gate-0 git', 'tag %s missing while on %s' % (TAG, BRANCH))
        sh(['git', 'tag', '-a', TAG, '-m', TAG_MSG], cwd=repo)
        log('created annotated tag %s at %s' % (TAG, git(R, 'rev-parse', 'HEAD')))
    if git(R, 'cat-file', '-t', TAG) != 'tag':
        raise GateFailure('Gate-0 git', '%s exists but is not an annotated tag' % TAG)
    prev = state().get('stage0', {})
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
    R = D.discover(repo)
    D.report(R)
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
    sh(['git', 'add', '--'] + rel, cwd=R['repo'])
    if sh(['git', 'diff', '--cached', '--quiet'], cwd=R['repo'], check=False).returncode == 0:
        return None
    sh(['git', 'commit', '-q', '-m', msg + '\n\n' + TRAILER], cwd=R['repo'])
    h = git(R, 'rev-parse', '--short', 'HEAD')
    log('commit %s on %s: %s' % (h, BRANCH, msg))
    return h


# ---------------------------------------------------------------- stage 1: implementation
def stage1(R):
    ensure_branch(R)
    L.TEMPLATE.update(FIND_CONFIG=R['find_config'], INC_CORE=R['inc_core'],
                      TEST_COMMON_H=os.path.basename(R['test_common_h']))
    if re.search(r'\bstruct\s+Fixes\b', open(R['switches_h']).read()):
        log('stage1: Fixes struct already present on %s (resumed run), edits not re-applied' % BRANCH)
    else:
        for step in TB.BASE_STEPS:
            step(R)
        TN.r3_positive_part(R)
        TN.r4m_dwell_waiver(R)
        TN.cmake_tests(R, NEW_TESTS)
        for fn in ('golden_scenarios.h', 'gen_golden.cpp', 'test_fixes.cpp'):
            L.write_file(os.path.join(R['test_dir'], fn), L.fill(open(os.path.join(HERE, 'templates', fn)).read()))
        for name, (flags, note) in TN.default_presets().items():
            TN.write_preset(R, name, flags, note)
        TN.start_script(R)
        TN.batch_script(R)
    # the pipeline itself, for reproducibility (next to the analysis script)
    dst = os.path.join(os.path.dirname(R['post_process']), 'fix_pipeline')
    for f in glob.glob(os.path.join(HERE, '*.py')) + glob.glob(os.path.join(HERE, 'templates', '*')) + \
            glob.glob(os.path.join(HERE, 'base_steps', '*.py')):
        L.write_file(os.path.join(dst, os.path.relpath(f, HERE)), open(f).read())
    h = commit(R, L.TOUCHED, 'Add FX1-FX6 coupling repairs behind default-false flags, presets, tests, pipeline')
    changed = git(R, 'diff', '--name-only', TAG, 'HEAD').split('\n')
    log('stage1: %d files differ from %s on %s (commit %s)' % (len([c for c in changed if c]), TAG, BRANCH, h))
    integrity(R, 'stage1')


# ---------------------------------------------------------------- stage 2: build + offline tests
def flags_make(path):
    t = open(path).read()
    cxx = re.search(r'^# compile CXX with (\S+)', t, flags=re.M).group(1)
    flags = re.search(r'^CXX_FLAGS = (.*)$', t, flags=re.M).group(1).split()
    incs = re.search(r'^CXX_INCLUDES = (.*)$', t, flags=re.M).group(1).split()
    eig = []
    for i, x in enumerate(incs):
        if 'eigen' in x.lower():
            eig += ([incs[i - 1]] if incs[i - 1] in ('-isystem', '-I') else []) + [x]
    return cxx, flags, eig


def node_log_format_check(R, frozen_pkg):
    """cycles.csv / paths.csv identity at source level: every function of the node that writes these streams keeps
    every frozen line in order; additions are guarded (ext_open_ / fx.any()) and never write to these streams."""
    rel = os.path.relpath(R['node_cpp'], R['pkg_root'])
    old, new = open(os.path.join(frozen_pkg, rel)).read(), open(R['node_cpp']).read()

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


def stage2(R):
    ensure_branch(R)
    out = {}
    log('stage2: building %s with %s' % (R['pkg_name'], ' '.join(R['build_cmd'])))
    b = sh('source %s && %s' % (R['devel_setup'], ' '.join(R['build_cmd'])), cwd=R['ws'], check=False, timeout=3600)
    open(os.path.join(PIPE_DIR, 'build.log'), 'w').write(b.stdout)
    if b.returncode != 0:
        raise GateFailure('Gate-2 build', 'build failed (exit %d), tail:\n%s' % (b.returncode, b.stdout[-3000:]))
    # existing offline tests
    for t in R['existing_tests']:
        r = sh([os.path.join(R['bin_dir'], t)], check=False, timeout=1800)
        open(os.path.join(PIPE_DIR, '%s.out' % t), 'w').write(r.stdout)
        out[t] = {'exit': r.returncode, 'pass': r.stdout.count('[PASS]'), 'fail': r.stdout.count('[FAIL]')}
        log('existing test %s: exit %d, %d PASS, %d FAIL' % (t, r.returncode, out[t]['pass'], out[t]['fail']))
        if r.returncode != 0:
            raise GateFailure('Gate-2 existing tests', '%s failed:\n%s' % (t, r.stdout[-3000:]))
    # frozen reference: the tag's sources, compiled with the package's own compiler flags
    frozen = os.path.join(PIPE_DIR, 'frozen_tree')
    shutil.rmtree(frozen, ignore_errors=True)
    os.makedirs(frozen)
    pkg_rel = os.path.relpath(R['pkg_root'], R['repo'])
    sh('git -C %s archive %s %s | tar -x -C %s' % (R['repo'], TAG, pkg_rel, frozen))
    fpkg = os.path.join(frozen, pkg_rel)
    cxx, cflags, eig = flags_make(R['flags_make'])
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
    # independent identity check: the branch's gen_golden (all flags at their default false) vs the reference
    branch_dump = os.path.join(PIPE_DIR, 'golden_branch_flags_off.txt')
    sh([os.path.join(R['bin_dir'], 'gen_golden'), branch_dump], timeout=1800)
    if open(gold, 'rb').read() != open(branch_dump, 'rb').read():
        a, b = open(gold).read().split('\n'), open(branch_dump).read().split('\n')
        diff = list(difflib.unified_diff(a, b, 'thesis-frozen-v1', BRANCH + ' (flags off)', n=1, lineterm=''))[:80]
        open(os.path.join(PIPE_DIR, 'T0_diff.txt'), 'w').write('\n'.join(diff))
        raise GateFailure('Gate-T0', 'branch dump with every flag false differs from the frozen reference', {'diff': diff})
    out['T0_branch_gen_golden'] = 'identical'
    out['log_format_functions'] = node_log_format_check(R, fpkg)
    log('Gate-T0 log format: frozen writer lines unchanged in %s' % out['log_format_functions'])
    # new tests T0-T10 (+T0b, T5b, T6b)
    log('running test_fixes (T0-T10)')
    r = sh([os.path.join(R['bin_dir'], 'test_fixes'), gold], check=False, timeout=7200)
    open(os.path.join(PIPE_DIR, 'test_fixes.out'), 'w').write(r.stdout)
    lines = r.stdout.split('\n')
    t0 = [l for l in lines if re.match(r'\[(PASS|FAIL)\] T0 all fixes off', l)]
    fails = [l for l in lines if l.startswith('[FAIL]')]
    out['test_fixes'] = {'exit': r.returncode, 'pass': r.stdout.count('[PASS]'), 'fail': len(fails), 'T0': t0,
                         'info': [l for l in lines if l.startswith('[INFO]')]}
    if not t0 or not t0[0].startswith('[PASS]'):
        raise GateFailure('Gate-T0', 'T0 failed: %s (see T0 diff above / test_fixes.out)' % t0)
    if r.returncode != 0 or fails:
        raise GateFailure('Gate-2 new tests', 'test_fixes failures:\n%s' % '\n'.join(fails))
    log('test_fixes: %d PASS, 0 FAIL; Gate-T0 passed' % out['test_fixes']['pass'])
    # the evaluation seeds must be valid scenario draws
    sc = open(R['scenario_file']).read()
    dmin = re.search(r'--d_min\s+(\S+)', sc).group(1)
    for seeds in ('%d-%d' % (V1_SEEDS[0], V1_SEEDS[-1]), '%d-%d' % (V2_SEEDS[0], V2_SEEDS[-1]), '%d-%d' % (V3_SEEDS[0], V3_SEEDS[-1])):
        c = sh(['python3', R['check_scenarios'], R['scenario_file'], '--d_min', dmin, '--seeds', seeds], check=False)
        out['scenario_check_' + seeds] = c.stdout[-1500:]
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
    if not out.startswith(STATE_ROOT + os.sep) or os.path.realpath(out).startswith(os.path.realpath(R['frozen_out'])):
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
    flags = preset_flags(R, preset)
    rows = A.post_process_arm(R, out, tag)
    valid = {(r['variant'], r['group'], str(r['seed'])): r for r in rows if str(r['valid']) == '1'}
    missing = sorted(c for c in cells if c not in valid)
    ext = {}
    wrong = []
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
                    'M2': r['M2_path_length_m'], 'M3': r['M3_max_turn_deg_1s'], 'M6': r['M6_flip_backs'],
                    'M1_p95': r['M1_p95_ms'], 'stall_breaks': r['stall_breaks'], 'min_static': r['min_static_surface_dist_odom_m'],
                    'min_dyn': r['min_dyn_surface_dist_odom_m'], 'low_speed_share': round(x['low_speed_share'], 4),
                    'path_switches': x['path_switches'], 'r4_held_under_threat': len(x['r4_held_under_threat']),
                    'replans': ' '.join('%s:%d' % kv for kv in sorted(x['replans'].items()))})
    return out


def write_series(out, ev):
    d = os.path.join(out, 'series')
    os.makedirs(d, exist_ok=True)
    for c, x in ev['ext'].items():
        if x['dyn_series']:
            with open(os.path.join(d, '%s_%s_seed%s_dyn.csv' % c), 'w') as f:
                f.write('t,min_dyn_surface_dist_odom_m\n' + ''.join('%.3f,%.4f\n' % p for p in x['dyn_series']))


# ---------------------------------------------------------------- V1 smoke ladder
def v1(R):
    ensure_branch(R)
    integrity(R, 'v1 start')
    cells = [(v, g, str(s)) for v in V1_VARIANTS for g in V1_GROUPS for s in V1_SEEDS]
    st = state().get('v1', {'steps': {}, 'adjust': [], 'r4m': False, 'r4_disabled': False})
    defaults = TN.default_presets()

    def run_step(name, flags, note):
        if name not in defaults or defaults[name][0] != flags:
            TN.write_preset(R, name, flags, note)
            commit(R, [os.path.join(R['fixes_dir'], name + '.yaml')], 'V1: preset %s (%s)' % (name, note))
        out = os.path.join(STATE_ROOT, 'v1', name)
        for s in V1_SEEDS:
            run_batch(R, out, 'v1', name, [s], V1_VARIANTS, V1_GROUPS)
        ev = evaluate_arm(R, out, 'v1', name, cells)
        if ev['missing']:
            raise GateFailure('Gate-V1 infrastructure', 'cells without a valid run after 3 attempts: %s' % ev['missing'])
        write_series(out, ev)
        A.write_rows(os.path.join(out, 'v1_summary.csv'), summarize_arm(ev))
        return ev

    ref = None
    for name, units in TN.LADDER:
        if name in st['steps'] and st['steps'][name].get('done'):
            continue
        eff = [u for u in units if not (u == 'FX4' and st['r4_disabled'])]
        flags = TN.flags_of(eff, r4m=st['r4m'])
        tag_name = name + ('_r4m' if st['r4m'] and 'FX4' in eff else '') + ('_nor4' if st['r4_disabled'] and 'FX4' in units else '')
        ev = run_step(tag_name, flags, 'V1 step %s, units %s' % (name, '+'.join(eff) or 'none'))
        if ref is None:
            ref = ev if name == 'diag_only' else evaluate_arm(R, os.path.join(STATE_ROOT, 'v1', 'diag_only'), 'v1', 'diag_only', cells)
        bad, delayed = unsafe_runs(ev), r4_delays(ev, ref)
        rec = {'preset': tag_name, 'units': eff, 'unsafe': bad, 'r4_delays': delayed}
        if (bad or delayed) and 'FX4' in eff and not st['r4m']:
            st['r4m'] = True
            st['adjust'].append('step %s: %s -> r4m_dwell_free_only enabled' % (name, 'unsafe runs' if bad else 'avoidance delayed by the dwell'))
            ev = run_step(name + '_r4m', TN.flags_of(eff, r4m=True), 'V1 step %s with R4-m fallback' % name)
            bad, delayed = unsafe_runs(ev), r4_delays(ev, ref)
            rec.update(preset=name + '_r4m', unsafe_after_r4m=bad, r4_delays_after_r4m=delayed)
            if bad:
                set_state('v1', st)
                raise GateFailure('Gate-V1 safety', 'unsafe after the R4-m fallback at %s' % name, rec)
            if delayed:
                st['r4_disabled'] = True
                st['adjust'].append('step %s: delay persists with R4-m -> r4_switch_hysteresis disabled' % name)
                eff = [u for u in eff if u != 'FX4']
                ev = run_step(name + '_nor4', TN.flags_of(eff, r4m=False), 'V1 step %s without R4' % name)
                bad = unsafe_runs(ev)
                rec.update(preset=name + '_nor4', units=eff, unsafe_without_r4=bad)
                if bad:
                    set_state('v1', st)
                    raise GateFailure('Gate-V1 safety', 'unsafe with R4 disabled at %s' % name, rec)
        elif bad:
            set_state('v1', st)
            raise GateFailure('Gate-V1 safety', 'collision or mission failure at %s (R4-m fallback not applicable: '
                                                 'R4 not active in this step)' % name, rec)
        rec['done'] = True
        rec['summary'] = summarize_arm(ev)
        st['steps'][name] = rec
        set_state('v1', st)
        log('V1 %s: %d runs, unsafe %d, R4 delays %d' % (rec['preset'], len(ev['rows']), len(bad), len(delayed)))
    # the evaluated configuration of the last ladder step becomes all_on
    last = st['steps'][TN.LADDER[-1][0]]
    final = TN.flags_of(last['units'], r4m=st['r4m'])
    if final != defaults['all_on'][0]:
        TN.write_preset(R, 'all_on', final, 'all repairs kept after V1: %s%s' % ('+'.join(last['units']), ' with R4-m' if st['r4m'] else ''))
        commit(R, [os.path.join(R['fixes_dir'], 'all_on.yaml')], 'V1: all_on updated (%s)' % '; '.join(st['adjust']))
    st['all_on'] = final
    st['done'] = True
    set_state('v1', st)
    integrity(R, 'v1 end')


def r4_delays(ev, ref):
    """dwell-held switches under a threat after which the dynamic clearance fell more than DIST_MARGIN below the
    diag_only run of the same variant and seed."""
    out = []
    for c, x in ev['ext'].items():
        rc = (c[0], c[1], c[2])
        if rc not in ref['ext'] or not x['r4_held_under_threat']:
            continue
        ref_min = ref['ext'][rc]['dyn_min']
        for h in x['r4_held_under_threat']:
            if h['win_min_dyn'] == h['win_min_dyn'] and ref_min == ref_min and h['win_min_dyn'] < ref_min - DIST_MARGIN:
                out.append({'cell': c, 't': h['t'], 'window_min_dyn': h['win_min_dyn'], 'reference_min_dyn': ref_min})
    return out


# ---------------------------------------------------------------- V2 screening
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


def v2(R):
    ensure_branch(R)
    integrity(R, 'v2 start')
    pp = A.load_pp(R)
    st = state().get('v2', {'round': 0, 'bisect': [], 'disabled_units': []})
    cells = [(v, g, str(s)) for v in pp.VARIANTS for g in V2_GROUPS for s in V2_SEEDS]
    off_out = os.path.join(STATE_ROOT, 'v2', 'all_off')
    while True:
        on_name = 'all_on'
        on_out = os.path.join(STATE_ROOT, 'v2', 'all_on' if st['round'] == 0 else 'all_on_round%d' % st['round'])
        for s in V2_SEEDS:  # arms interleaved per seed, so drift of the machine state hits both arms alike
            run_batch(R, off_out, 'v2', 'all_off', [s], pp.VARIANTS, V2_GROUPS)
            run_batch(R, on_out, 'v2', on_name, [s], pp.VARIANTS, V2_GROUPS)
        off = evaluate_arm(R, off_out, 'v2', 'all_off', cells)
        on = evaluate_arm(R, on_out, 'v2', on_name, cells)
        if off['missing'] or on['missing']:
            raise GateFailure('Gate-V2 infrastructure', 'missing valid cells', {'off': off['missing'], 'on': on['missing']})
        for ev, o in ((off, off_out), (on, on_out)):
            write_series(o, ev)
            A.write_rows(os.path.join(o, 'v2_summary.csv'), summarize_arm(ev))
        fails = v2_gates(off, on, cells)
        rr = {'round': st['round'], 'on_out': on_out, 'on_flags': on['flags'], 'gate_failures': fails,
              'off_summary': summarize_arm(off), 'on_summary': summarize_arm(on)}
        st.setdefault('rounds', []).append(rr)
        set_state('v2', st)
        if not fails:
            st['passed'] = True
            st['final_on_flags'] = on['flags']
            set_state('v2', st)
            log('V2 gates passed (round %d)' % st['round'])
            break
        log('V2 gate failures (round %d): %s' % (st['round'], fails))
        if st['round'] >= 2:
            raise GateFailure('Gate-V2 safety', 'gates still failing after %d bisection rounds' % st['round'], rr)
        culprit = bisect(R, st, off, fails, cells)
        units = [u for u in TN.UNITS if any(on['flags'].get(f) for f in TN.UNITS[u]) and u != culprit]
        r4m = bool(on['flags'].get('r4m_dwell_free_only')) and 'FX4' in units
        TN.write_preset(R, 'all_on', TN.flags_of(units, r4m=r4m), 'V2 round %d: %s disabled by bisection' % (st['round'] + 1, culprit))
        commit(R, [os.path.join(R['fixes_dir'], 'all_on.yaml')], 'V2: %s disabled (negative impact located by bisection)' % culprit)
        st['disabled_units'].append(culprit)
        st['round'] += 1
        set_state('v2', st)
    integrity(R, 'v2 end')


def bisect(R, st, off, fails, cells):
    """Binary search over the active repair units for the one whose enabling reproduces the gate failure."""
    bad_vg = sorted({tuple(f['cell'][:2]) if 'cell' in f else None for f in fails} - {None})
    bad_groups = sorted({f['group'] for f in fails if 'group' in f})
    variants = sorted({vg[0] for vg in bad_vg}) or A.PP.VARIANTS
    groups = sorted({vg[1] for vg in bad_vg} | set(bad_groups))
    sub = [c for c in cells if c[0] in variants and c[1] in groups]
    on_flags = preset_flags(R, 'all_on')
    cand = [u for u in TN.UNITS if any(on_flags.get(f) for f in TN.UNITS[u])]
    r4m = bool(on_flags.get('r4m_dwell_free_only'))
    n = len(st['bisect'])
    while len(cand) > 1:
        half = cand[:len(cand) // 2]
        name = 'v2_bisect_%d' % n
        TN.write_preset(R, name, TN.flags_of(half, r4m=r4m), 'V2 bisection: %s' % '+'.join(half))
        commit(R, [os.path.join(R['fixes_dir'], name + '.yaml')], 'V2: bisection preset %s' % name)
        out = os.path.join(STATE_ROOT, 'v2', name)
        for s in V2_SEEDS:
            run_batch(R, out, 'v2', name, [s], variants, groups)
        ev = evaluate_arm(R, out, 'v2', name, sub)
        f = v2_gates(off, ev, sub) if not ev['missing'] else [{'gate': 'infrastructure', 'missing': ev['missing']}]
        st['bisect'].append({'preset': name, 'units': half, 'cells': sub, 'failures': f})
        set_state('v2', st)
        cand = half if f else cand[len(cand) // 2:]
        n += 1
    log('V2 bisection: negative impact attributed to %s' % cand[0])
    return cand[0]


# ---------------------------------------------------------------- V3 confirmation
def v3(R):
    ensure_branch(R)
    integrity(R, 'v3 start')
    if not state().get('v2', {}).get('passed'):
        raise GateFailure('Gate-V3 precondition', 'V2 gates have not passed')
    pp = A.load_pp(R)
    cells = [(v, g, str(s)) for v in pp.VARIANTS for g in V3_GROUPS for s in V3_SEEDS]
    outs = {'off': os.path.join(STATE_ROOT, 'v3', 'all_off'), 'on': os.path.join(STATE_ROOT, 'v3', 'all_on')}
    presets = {'off': 'all_off', 'on': 'all_on'}
    for s in V3_SEEDS:
        for arm in ('off', 'on'):
            run_batch(R, outs[arm], 'v3', presets[arm], [s], pp.VARIANTS, V3_GROUPS)
    evs = {arm: evaluate_arm(R, outs[arm], 'v3', presets[arm], cells) for arm in outs}
    arms = {arm: list(evs[arm]['rows'].values()) for arm in evs}
    recs = A.v3_tests(arms)
    A.write_rows(os.path.join(STATE_ROOT, 'v3', 'v3_tests.csv'), recs)
    for arm in evs:
        write_series(outs[arm], evs[arm])
        A.write_rows(os.path.join(outs[arm], 'v3_summary.csv'), summarize_arm(evs[arm]))
    set_state('v3', {'done': True, 'missing': {a: evs[a]['missing'] for a in evs}, 'on_flags': evs['on']['flags'],
                     'unsafe': {a: unsafe_runs(evs[a]) for a in evs}, 'tests_csv': os.path.join(STATE_ROOT, 'v3', 'v3_tests.csv')})
    log('V3 done: %d tests -> %s' % (len(recs), os.path.join(STATE_ROOT, 'v3', 'v3_tests.csv')))
    integrity(R, 'v3 end')


# ---------------------------------------------------------------- driver
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', default=None)
    ap.add_argument('cmd', choices=['run', 'integrity'])
    ap.add_argument('--until', choices=STAGES, default='report')
    a = ap.parse_args()
    repo = a.repo or sh(['git', 'rev-parse', '--show-toplevel']).stdout.strip()
    os.makedirs(PIPE_DIR, exist_ok=True)
    cur = 'stage0'
    try:
        R = stage0(repo)
        if a.cmd == 'integrity':
            return
        done = state().get('done_stages', [])
        for stg in STAGES[1:STAGES.index(a.until) + 1]:
            cur = stg
            if stg in done:
                log('%s already completed, skipped' % stg)
                continue
            log('==== %s ====' % stg)
            if stg == 'report':
                import report
                report.write(R, state(), TAG, BRANCH)
                commit(R, L.TOUCHED, 'Report of the fix_coupling_artifacts pipeline (supplementary analysis, section 4.5)')
                integrity(R, 'final')
            else:
                globals()[stg](R)
            done = state().get('done_stages', []) + [stg]
            set_state('done_stages', done)
        log('pipeline finished up to %s' % a.until)
    except GateFailure as e:
        L.fail_report(e, cur)
    except Exception as e:  # any unexpected fault stops the pipeline with a report as well
        import traceback
        L.fail_report(GateFailure('unexpected error', traceback.format_exc()), cur)


if __name__ == '__main__':
    main()
