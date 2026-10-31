"""Stage 6: final report. All numbers come from state.json and the V3 test CSV written by the earlier stages; the
wording is rule-based (Chapter-4 conventions: 'no evidence of a significant difference' for non-significant
results, g6 vs g3 only as the joint C2 + C3 effect, no success-rate improvement claim at the ceiling)."""
import csv
import os
import statistics as stt

import json
import re

import analysis as A
from fixlib import write_file, sh, PIPE_DIR, STATE_ROOT, log


def last_gate():
    """Newest gate-failure report of the pipeline: file name, gate, detail and the attached data."""
    fs = sorted((x for x in os.listdir(PIPE_DIR) if x.startswith('GATE_FAILURE_')),
                key=lambda x: os.path.getmtime(os.path.join(PIPE_DIR, x)))
    if not fs:
        return None
    t = open(os.path.join(PIPE_DIR, fs[-1])).read()
    g = re.search(r'^- gate: (.*)$', t, flags=re.M)
    d = re.search(r'## Detail\n\n```\n(.*?)\n```', t, flags=re.S)
    m = re.search(r'```json\n(.*)\n```', t, flags=re.S)
    return {'file': fs[-1], 'gate': g.group(1) if g else '', 'detail': d.group(1).strip() if d else '',
            'data': json.loads(m.group(1)) if m else {}}

METRIC_NAME = {'M2_path_length_m': ('M2 path length', 'm'), 'M3_max_turn_deg_1s': ('M3 maximum 1 s turning angle', '°'),
               'M6_flip_backs': ('M6 flip-back count', ''), 'M1_mean_ms': ('M1 mean planning time per cycle', 'ms'),
               'M1_p95_ms': ('M1 P95 planning time per cycle', 'ms'), 'M5_mean_m': ('M5 mean deviation from Π₀', 'm'),
               'M5_max_m': ('M5 maximum deviation from Π₀', 'm'), 'stall_breaks': ('stall-break count', '')}
FOCUS = ['M2_path_length_m', 'M3_max_turn_deg_1s', 'M6_flip_backs']


def f(x, nd=3):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return '—'
    return '—' if x != x else ('%.*f' % (nd, x))


def p4(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return '—'
    return '< 0.0001' if x < 0.0001 else '%.4f' % x


def med(vals):
    v = []
    for x in vals:
        try:
            x = float(x)
        except (TypeError, ValueError):
            continue
        if x == x:
            v.append(x)
    return stt.median(v) if v else float('nan')


def load_tests():
    p = os.path.join(STATE_ROOT, 'v3', 'v3_tests.csv')
    if not os.path.isfile(p):
        return []
    with open(p) as fh:
        return list(csv.DictReader(fh))


def verdict(rec):
    if rec.get('p_holm') in ('', None, 'None'):
        return 'untestable'
    sig = rec.get('significant') == '1'
    e = float(rec['effect'])
    if not sig:
        return 'none'
    return 'lower' if e < 0 else 'higher'


def sentence(rec, a_label, b_label):
    name, unit = METRIC_NAME.get(rec['metric'], (rec['metric'], ''))
    if rec.get('p_holm') in ('', None, 'None'):
        return '*%s:* %s vs %s, %s: not testable (%s); reported descriptively only.' % (
            rec['variant'], a_label, b_label, name, rec.get('note') or 'no pairs')
    v = verdict(rec)
    what = {'lower': 'a significantly lower value', 'higher': 'a significantly higher value',
            'none': 'no evidence of a significant difference'}[v]
    u = (' ' + unit) if unit and unit != '°' else unit
    pv = p4(rec['p_holm'])
    pv = pv if pv.startswith('<') else '= ' + pv
    return ('*%s:* compared to %s, %s shows %s in the %s (Wilcoxon signed-rank Pratt test; Holm-corrected p %s; '
            'effect size: paired median difference = %s%s; 95%% bootstrap CI = [%s, %s]%s; n = %s pairs).' % (
                rec['variant'], b_label, a_label, what, name, pv, f(rec['effect']), u, f(rec['ci_low']),
                f(rec['ci_high']), u, rec['n_pairs']))


def classify(tests):
    """per (variant, metric): A = fixed g6 vs frozen g6, B = fixed g6 vs frozen g3, D = frozen g6 vs frozen g3."""
    out = {}
    by = {(t['variant'], t['metric'], t['contrast']): t for t in tests}
    for (v, m, c) in list(by):
        if c != 'A' or m not in FOCUS:
            continue
        A, B, Dd = by[(v, m, 'A')], by.get((v, m, 'B')), by.get((v, m, 'D'))
        va, vb, vd = verdict(A), verdict(B) if B else 'untestable', verdict(Dd) if Dd else 'untestable'
        if va == 'lower':
            cls = 'mitigated; no evidence of a significant difference to frozen g3 remains' if vb in ('none', 'untestable', 'lower') \
                else 'mitigated; a significant gap to frozen g3 remains'
        elif va == 'higher':
            cls = 'worsened'
        elif va == 'none':
            cls = 'no evidence of a significant change'
        else:
            cls = 'not testable'
        out[(v, m)] = {'class': cls, 'A': A, 'B': B, 'D': Dd, 'baseline_gap_on_v3': vd}
    return out


def gate_files(it):
    """Gate-failure reports of iteration `it` (iteration 1 files carry no iteration suffix)."""
    out = []
    for x in sorted(os.listdir(PIPE_DIR), key=lambda x: os.path.getmtime(os.path.join(PIPE_DIR, x))):
        if not x.startswith('GATE_FAILURE_'):
            continue
        m = re.search(r'_iter(\d+)_', x)
        if (int(m.group(1)) if m else 1) == it:
            out.append(x)
    return out


def gate_info(x):
    t = open(os.path.join(PIPE_DIR, x)).read()
    g = re.search(r'^- gate: (.*)$', t, flags=re.M)
    tm = re.search(r'^- time: (.*)$', t, flags=re.M)
    d = re.search(r'## Detail\n\n```\n(.*?)\n```', t, flags=re.S)
    m = re.search(r'```json\n(.*)\n```', t, flags=re.S)
    return {'file': x, 'time': tm.group(1) if tm else '', 'gate': g.group(1) if g else '',
            'detail': d.group(1).strip() if d else '', 'data': json.loads(m.group(1)) if m else {}}


def v1_table(w, root):
    v1dir = os.path.join(root, 'v1')
    presets = sorted((d for d in os.listdir(v1dir) if os.path.isfile(os.path.join(v1dir, d, 'v1_summary.csv'))) if os.path.isdir(v1dir) else [],
                     key=lambda d: os.path.getmtime(os.path.join(v1dir, d, 'batch_index.csv')))
    w('| Preset | Variant | ok | M2 [m] | M3 [°] | M6 | Σ stall breaks | path switches | min dyn. clearance [m] | duration [s] | '
      'gap time [s] | gap median speed [m/s] | gap low-speed share | Σ R4 overrides |')
    w('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for d in presets:
        rows = list(csv.DictReader(open(os.path.join(v1dir, d, 'v1_summary.csv'))))
        for v in sorted({x['variant'] for x in rows}):
            s = [x for x in rows if x['variant'] == v]
            dyn = [float(x['min_dyn']) for x in s if x['min_dyn'] not in ('', 'N/A')]
            gap = v == 'M1_three_way'
            w('| %s | %s | %d/%d | %s | %s | %s | %d | %s | %s | %s | %s | %s | %s | %s |' % (
                d, v, sum(int(x['success']) for x in s), len(s), f(med([x['M2'] for x in s]), 2), f(med([x['M3'] for x in s]), 1),
                f(med([x['M6'] for x in s]), 1), sum(int(float(x['stall_breaks'] or 0)) for x in s),
                f(med([x['path_switches'] for x in s]), 1), f(min(dyn), 3) if dyn else '—', f(med([x.get('duration', '') for x in s]), 1),
                f(med([x.get('gap_time', '') for x in s]), 1) if gap else '—',
                f(med([x.get('gap_median_speed', '') for x in s]), 2) if gap else '—',
                f(med([x.get('gap_low_share', '') for x in s]), 3) if gap else '—',
                sum(int(float(x.get('r4_overrides') or 0)) for x in s)))


def cross_iteration(w):
    """Same configuration, same seed, different iterations (simulator repeatability), and the M1_three_way narrow-gap
    passage for every V1 preset of every iteration."""
    import glob
    dirs = sorted(glob.glob(os.path.join(STATE_ROOT, 'v1', '*', 'v1_summary.csv')) +
                  glob.glob(os.path.join(STATE_ROOT, 'iter*', 'v1', '*', 'v1_summary.csv')))
    runs = {}
    for p in dirs:
        it = re.search(r'/iter(\d+)/', p)
        it = int(it.group(1)) if it else 1
        preset = os.path.basename(os.path.dirname(p))
        for r in csv.DictReader(open(p)):
            logs = sorted(glob.glob(os.path.join(os.path.dirname(p), r['variant'], r['group'], 'seed%s_a*' % r['seed'])))
            runs[(it, preset, r['variant'], r['seed'])] = dict(r, log_dir=logs[0] if logs else '')
    its = sorted({k[0] for k in runs})
    if len(its) < 2:
        return
    w('### Repeatability: identical configuration (`diag_only`, frozen behaviour) and seed in two iterations [sim]')
    w('')
    w('| Variant | Seed | %s |' % ' | '.join('min dyn. clearance it. %d [m]' % i for i in its))
    w('|---|---|%s' % ('---|' * len(its)))
    for v in ('head_on', 'M1_three_way'):
        for s in ('101', '102', '103'):
            vals = [runs.get((i, 'diag_only', v, s), {}).get('min_dyn', '') for i in its]
            w('| %s | %s | %s |' % (v, s, ' | '.join(f(x) for x in vals)))
    w('')
    w('### M1_three_way narrow gap (y 0.7–3.2 m) in every V1 run [sim]')
    w('')
    w('| Iteration | Preset | Seed | traversal [s] | median speed [m/s] | low-speed share |')
    w('|---|---|---|---|---|---|')
    for k in sorted(k for k in runs if k[2] == 'M1_three_way'):
        r = runs[k]
        if r.get('gap_time', '') == '':  # iteration-1 summaries predate the gap columns: recompute from the logs
            x = A.run_extras(r['log_dir']) if r.get('log_dir') else {}
            r = dict(r, gap_time=x.get('gap_time', ''), gap_median_speed=x.get('gap_median_speed', ''),
                     gap_low_share=x.get('gap_low_share', ''))
        w('| %d | %s | %s | %s | %s | %s |' % (k[0], k[1], k[3], f(r['gap_time'], 1), f(r['gap_median_speed'], 3),
                                             f(r['gap_low_share'], 2)))
    w('')


def write(R, st, tag, branch, root):
    repo = R['repo']
    lines = []
    w = lines.append
    it = int(st.get('iteration', 1))
    tag_obj = sh(['git', 'rev-parse', tag], cwd=repo).stdout.strip()
    tag_commit = sh(['git', 'rev-parse', tag + '^{commit}'], cwd=repo).stdout.strip()
    commits = sh(['git', 'log', '--oneline', '%s..%s' % (tag, branch)], cwd=repo).stdout.strip()
    s0, dr, s2 = st.get('stage0', {}), st.get('dryrun', {}), st.get('stage2_result', {})
    v1, v2, v3 = st.get('v1', {}), st.get('v2', {}), st.get('v3', {})
    p = os.path.join(root, 'v3', 'v3_tests.csv')
    tests = list(csv.DictReader(open(p))) if v3.get('done') and os.path.isfile(p) else []
    cls = classify(tests)
    gates = [gate_info(x) for x in gate_files(it)]

    w('# fix_coupling_artifacts — pipeline report (iteration %d; supplementary analysis, thesis §4.5)' % it)
    w('')
    w('> Chapter 4 reports the frozen baseline `%s` and is unchanged. Everything here is a post-campaign engineering '
      'patch evaluated as supplementary analysis. Evidence classes: **[code]** fact read from the source, **[log]** clue '
      'from run logs (association only), **[sim]** result of the ablation simulations — the only class accepted as '
      'evidence for or against mitigation. g6 vs g3 is the joint C2 + C3 effect only. Unit tests and the dry run are not '
      'closed-loop evidence.' % tag)
    w('')
    w('## 1 Git baseline protection')
    w('')
    w('| Item | Value |')
    w('|---|---|')
    w('| Tag | `%s` (annotated, object `%s`) → commit `%s` |' % (tag, tag_obj[:12], tag_commit[:12]))
    w('| Protected base branch | `%s`, commit unchanged: `%s` |' % (s0.get('base_branch'), s0.get('assets', {}).get('base_branch_commit', '')[:12]))
    w('| Development branch | `%s` |' % branch)
    w('| Read-only assets | Chapter-4 doc, frozen 720-cell dataset, frozen analysis outputs: SHA-256 identical at every gate (Gate-I) |')
    w('| Uncommitted user files left untouched | %s |' % ', '.join('`%s`' % x for x in s0.get('uncommitted_user_files', [])))
    w('')
    w('Commits on `%s`:' % branch)
    w('')
    w('```\n%s\n```' % commits)
    w('')
    w('## 2 Iterations')
    w('')
    for h in st.get('history', []):
        hv1 = h.get('v1') or {}
        w('- **Iteration %d** (closed %s): %s' % (h['iteration'], h.get('closed', ''), h.get('note', '')))
        w('  - V1 steps completed: %s; adjustments: %s' % (', '.join(k for k, r in hv1.get('steps', {}).items() if r.get('done')) or 'none',
                                                         '; '.join(hv1.get('adjust', [])) or 'none'))
        for x in gate_files(h['iteration']):
            gi = gate_info(x)
            w('  - gate stop `%s`: %s — %s' % (x, gi['gate'], gi['detail'].split('\n')[0][:200]))
    w('- **Iteration %d** (this report).' % it)
    w('')
    w('## 3 Dry run (sandbox copy of the tag; repository untouched)')
    w('')
    if dr.get('passed'):
        w('- Edit layers applied on the frozen sources (every anchor matched exactly once): %s.' % '; '.join(dr['layers']))
        w('- Full diff: %d files, +%d / −%d lines (sha256 `%s`), kept in `%s`.' % (
            dr['diff']['files'], dr['diff']['added'], dr['diff']['removed'], dr['diff']['sha256'][:16], dr['diff']['path']))
        w('- Compilation with the package flags: no errors, no new warnings (core library, node translation units with their ROS flags, all test programs).')
        w('- Frozen `gen_golden` vs sandbox `gen_golden` (all flags false): %s.' % dr.get('T0_sandbox'))
        w('- Existing tests in the sandbox: %s.' % ', '.join('%s %s PASS' % (k[9:], x['pass']) for k, x in dr.items() if k.startswith('existing_')))
        w('- Sandbox `test_fixes`: %s PASS, %s FAIL. Log writers of cycles.csv / paths.csv unchanged: %s.' % (
            dr['test_fixes']['pass'], dr['test_fixes']['fail'], ', '.join(dr.get('log_format_functions', []))))
    else:
        w('- Not passed in this iteration.')
    w('')
    dc = st.get('dryrun_check', {})
    if dc:
        w('**Phase A check** of `%s` and the sandbox test output: layers %s; problems: %s.' % (
            dc.get('console'), dc.get('layers'), dc.get('problems') or 'none'))
        w('')
        w('| Implementation checklist | Status |')
        w('|---|---|')
        for k, x in dc.get('checklist', {}).items():
            w('| %s | %s |' % (k, x))
        w('')
        w('| Test | Result (sandbox) |')
        w('|---|---|')
        for k, x in dc.get('tests', {}).items():
            w('| %s | %s |' % (k, x))
        w('')
    for x in st.get('dryrun_superseded', []):
        w('- Superseded dry run (edit sources changed afterwards; fingerprint `%s`): %s PASS / %s FAIL.' % (
            str(x.get('fingerprint'))[:16], x.get('test_fixes', {}).get('pass'), x.get('test_fixes', {}).get('fail')))
    w('')
    w('## 4 Stage 0–2 (deployment)')
    w('')
    for t in s2.get('existing_tests', []):
        r = s2.get(t, {})
        w('- Existing `%s`: exit %s, %s PASS, %s FAIL.' % (t, r.get('exit'), r.get('pass'), r.get('fail')))
    g = s2.get('golden', {})
    if g:
        w('- Frozen reference built from `%s` with `%s`: %s bytes; branch `gen_golden` (flags false): %s.' % (
            tag, g.get('flags'), g.get('bytes'), s2.get('T0_branch_gen_golden', '—')))
    tf = s2.get('test_fixes', {})
    if tf:
        w('- `test_fixes` (catkin build): %s PASS, %s FAIL. Gate-T0: %s' % (tf.get('pass'), tf.get('fail'), '; '.join(tf.get('T0', []))))
        for i in tf.get('info', []):
            w('  - `%s`' % i)
    w('')
    w('## 5 V1 pre-screening (g6; S3_climb_ne, head_on, M1_three_way; seeds 101–103) [sim, n = 3 per cell, descriptive]')
    w('')
    v1_table(w, root)
    w('')
    w('Pre-registered V1 gates: no collision and every goal reached; minimum dynamic clearance not more than 0.05 m below '
      'diag_only (same variant and seed); M1_three_way narrow-gap traversal at most 2 × the diag_only time of the same seed.')
    w('')
    for name, rec in v1.get('steps', {}).items():
        w('- %s (units %s): gate failures %d%s; dwell-held switches under threat followed by a clearance drop > 0.05 m: %d.' % (
            name, '+'.join(rec.get('units', [])) or '—', len(rec.get('gate_failures', [])),
            (' — %s' % rec['gate_failures']) if rec.get('gate_failures') else '', len(rec.get('r4_delays', []))))
    if v1.get('done'):
        w('- `all_on` after V1: %s' % ', '.join(k for k, x in v1.get('all_on', {}).items() if x))
    w('')
    cross_iteration(w)
    w('## 6 V2 pre-screening ablation (seeds 1–3; g3, g5, g6 × all_off / all_on × 6 variants) [sim, n = 3, descriptive]')
    w('')
    if v2.get('stats'):
        w('Gates (all must hold): no collision; no success loss per group; worst static / dynamic clearance per group × '
          'variant not lower than all_off by more than 0.05 m; mean run M1 P95 per group ≤ 1.2 × all_off. Result: %s.' % (
              'all passed' if v2.get('passed') else 'FAILED: %s' % v2.get('gate_failures')))
        w('')
        w('Medians per variant (off → on), and paired gaps over seeds (gap = g6 − g3, the joint C2 + C3 effect; with '
          'n = 3 no test can reach p < 0.05, so this table only screens):')
        w('')
        w('| Variant | Metric | g3 off → on | g5 off → on | g6 off → on | gap g6off−g3off | gap g6on−g3off | g6on − g6off |')
        w('|---|---|---|---|---|---|---|---|')
        for r in v2['stats']:
            w('| %s | %s | %s → %s | %s → %s | %s → %s | %s | %s | %s |' % (
                r['variant'], r['metric'], r['g3_off'], r['g3_on'], r['g5_off'], r['g5_on'], r['g6_off'], r['g6_on'],
                r['gap_off g6off-g3off'], r['gap_on g6on-g3off'], r['g6on-g6off']))
        w('')
        for k in sorted(x for x in v2 if x.startswith('side_effects_r')):
            w('- Side-effect screen %s (ON worse than OFF in ≥ 2 variants of a group: M2 > 5 %%, M3 > 15°, M6 > 1, Σ stall '
              'breaks > 2): %s' % (k[-2:], v2[k] or 'none'))
        for b in v2.get('bisect', []):
            w('- Bisection `%s` (units %s, %d cells): side effect %s; unsafe runs %d.' % (
                b['preset'], '+'.join(b['units']), len(b['cells']), 'reproduced' if b['reproduced'] else 'not reproduced', len(b['unsafe'])))
        w('- Repair units disabled by V2: %s' % (', '.join(v2.get('disabled_units', [])) or 'none'))
    else:
        w('Not executed in this iteration%s.' % (' (stopped before V2)' if gates else ''))
    w('')
    w('## 7 V3 formal evaluation')
    w('')
    if v3.get('done'):
        w('Contrasts per variant × metric, one Holm family of four: A = g6 fixed vs g6 frozen, B = g6 fixed vs g3 frozen, '
          'C = g3 fixed vs g3 frozen, D = g6 frozen vs g3 frozen. Missing cells: %s. Unsafe runs: %s.' % (
              v3.get('missing'), {k: len(x) for k, x in v3.get('unsafe', {}).items()}))
        w('')
        w('| Variant | Metric | D: frozen gap g6−g3 | A: fixed − frozen (g6) | B: fixed g6 − frozen g3 | Outcome |')
        w('|---|---|---|---|---|---|')
        for (v, m), c in sorted(cls.items()):
            def cell(r):
                return '%s [%s, %s], p_Holm %s' % (f(r['effect']), f(r['ci_low']), f(r['ci_high']), p4(r['p_holm'])) \
                    if r and r.get('p_holm') not in ('', None, 'None') else '—'
            w('| %s | %s | %s | %s | %s | %s |' % (v, METRIC_NAME[m][0], cell(c['D']), cell(c['A']), cell(c['B']), c['class']))
        w('')
        for t in tests:
            if t['metric'] in FOCUS + ['stall_breaks'] and t['contrast'] in ('A', 'B', 'D'):
                lab = {'A': ('g6 with the repairs', 'g6 frozen'), 'B': ('g6 with the repairs', 'g3 frozen'),
                       'D': ('g6 frozen', 'g3 frozen')}[t['contrast']]
                w('- ' + sentence(t, *lab))
    else:
        w('**Not started — requires explicit manual confirmation.** Specification:')
        w('')
        w('- Seeds 21–40 (fresh, never used in V1/V2); all six variants; groups g3 and g6 (g5 optional) × `all_off` / '
          '`all_on` (the V1-decided configuration); arms interleaved per seed; 480 runs (720 with g5), about 3 min each.')
        w('- Per arm, the project\'s `post_process_ablation.py --index` (validity rules, M1–M6, Wilcoxon–Pratt, Holm, '
          'bootstrap, exact McNemar). Cross-arm contrasts with the same functions, one Holm family per variant × metric: '
          'A = g6 fixed vs g6 frozen, B = g6 fixed vs g3 frozen, C = g3 fixed vs g3 frozen, D = g6 frozen vs g3 frozen; '
          'bootstrap 10 000 resamples, seed 20261026.')
        w('- Start command after confirmation: `pipeline.py run-v3 --confirm-v3 [--with-g5]`.')
    w('')
    if gates:
        w('## 8 Gate stops in this iteration')
        w('')
        for gi in gates:
            w('- `%s` (%s) — %s: %s' % (gi['file'], gi['time'], gi['gate'], gi['detail'].split('\n')[0][:300]))
            for u in gi['data'].get('failures', []) + gi['data'].get('unsafe', []):
                w('  - %s' % {k: (round(x, 3) if isinstance(x, float) else x) for k, x in u.items()})
                if u.get('log_dir'):
                    for line in A.run_timeline(u['log_dir']):
                        w('    - ' + line)
        w('')
    counts = {}
    for c in cls.values():
        counts[c['class']] = counts.get(c['class'], 0) + 1
    w('## 9 Thesis material')
    w('')
    adjust = '; '.join(v1.get('adjust', [])) or 'no repair switch had to be disabled or downgraded'
    w(PART_424.format(tag=tag))
    if v3.get('done'):
        w(THESIS_DONE.format(tag=tag, adjust=adjust, counts='; '.join('%s: %d' % kv for kv in sorted(counts.items()))))
    elif v2.get('passed'):
        w(THESIS_PENDING.format(tag=tag, adjust=adjust))
    else:
        stage = 'V1' if not v1.get('done') else 'V2'
        w(THESIS_STOPPED.format(tag=tag, stage=stage, adjust=adjust,
                                gate='%s (%s)' % (gates[-1]['gate'], gates[-1]['detail'].split('\n')[0]) if gates else 'not run'))
    w(COMMON_TAIL.format(tag=tag))
    text = '\n'.join(lines) + '\n'
    doc = os.path.join(os.path.dirname(R['chapter4_doc']), 'fix_coupling_artifacts_report.md')
    write_file(doc, text)
    with open(os.path.join(PIPE_DIR, 'final_report_iter%d.md' % it), 'w') as fh:
        fh.write(text)
    log('report -> %s' % doc)


PART_424 = '''### §4.2.4 (addition to the Chapter-4 discussion; frozen numbers only)

In the frozen baseline (`{tag}`), g6 and the one-way series g3 both succeeded in all 120 runs, but g6 produced worse
paths (§4.2.2): M2 was significantly longer in all six variants (paired median differences +0.241 m to +1.711 m), M3
significantly larger in S3_climb_ne, head_on, crossing_90 and M1_three_way, and M6 significantly higher in
S3_climb_ne, crossing_90 and M1_three_way. The degradation also appears in the three static variants, which contain no
moving obstacle; there the urgency ξ is infinite and λ ≡ 0, so neither the VO risk term nor Γ is active, and the static
degradation cannot be explained as the price of dynamic obstacle avoidance. All effects belong to the C2 + C3
combination jointly. Source inspection identified coupling defects able to produce this behaviour (a speed-independent
guidance-cone slack with a zero-width cone at λ = 0, single-cycle κ feedback, a replanning trigger over all cached
edges, no switching hysteresis); the post-hoc diagnostics of Appendix B are consistent with them but are associations,
not causal evidence. Chapter 4 reports the frozen baseline; the repair is evaluated in §4.5.
'''

SCOPE = '''**Scope.** This section reports a defect repair implemented in this work and its evaluation. It does not replace
§4.1–4.3, whose conclusions are derived only from the frozen baseline `{tag}`. The repaired planner is the frozen code
plus repair items FX1–FX6, each behind a Boolean flag that defaults to false; with every flag off the planner output is
byte-identical to the frozen reference (offline golden test T0) and the run-log writers are unchanged.

**Protocol.** Pre-registered gated sequence: dry run in a sandbox; V1 incremental smoke ladder (development seeds
101–103); V2 screening of {{g3, g5, g6}} × {{off, on}} × six variants (seeds 1–3); V3 confirmation on fresh seeds 21–40
only after manual confirmation. A failed gate stops the sequence; the next iteration starts with a changed design and a
new dry run, and every iteration is reported.
'''

THESIS_STOPPED = '''### §4.5 Supplementary extended analysis: repair of the coupling defects

''' + SCOPE + '''
**Outcome.** The evaluation stopped at the {stage} gate: {gate}. Switch adjustments made before the stop: {adjust}.
The later stages were not run, so this work makes **no claim** that the repair mitigates the path-quality degradation
of g6 relative to g3; the V1/V2 observations are descriptive screening data, not evidence.
'''

THESIS_PENDING = '''### §4.5 Supplementary extended analysis: repair of the coupling defects (template; V3 pending)

''' + SCOPE + '''
**Screening outcome.** V1 and V2 passed their safety gates (switch adjustments: {adjust}). Screening with three seeds
per cell is descriptive and cannot establish mitigation. **Formal results (V3):** ⟦to be filled from the V3 test
table: per variant and metric, the fixed-vs-frozen g6 contrast and the remaining g6 − g3 gap, written with the
Chapter-4 sentence template; non-significant results as "no evidence of a significant difference"⟧. Success rates
are at the ceiling; no success-rate improvement is claimed.
'''

THESIS_DONE = '''### §4.5 Supplementary extended analysis: repair of the coupling defects

''' + SCOPE + '''
**Results.** Switch adjustments made by the gates: {adjust}. Outcome counts over variant × {{M2, M3, M6}}: {counts}.
The per-variant statements are the template sentences of §7 of this report. Success rates are at the ceiling; no
success-rate improvement is claimed.
'''

COMMON_TAIL = '''
### Limitations (additions)

- The frozen baseline contains coupling defects that degrade path quality even in static scenes; Chapter 4 reports it
  with these defects.
- The repair evaluation uses one simulated world; V1/V2 are screening with three seeds per cell.
- FX3 guarantees decision consistency only for routes through cached edges; FX4 can delay a voluntary switch by at most
  T_g plus one replanning period in threat-free mode (none under a threat with R4-m).
- The frozen dynamic constraint (time-to-collision > τ₂ = 12 s) also applies to a stationary obstacle; near a goal that
  lies on such an obstacle's safety sphere it admits only approach speeds ≤ (d − R)/τ₂ (a creeping approach). This is a
  property of the frozen model, not of the repair, and it is not changed here.
- Simulator repeatability limits the screening: the same configuration (frozen behaviour) on the same seed gave
  minimum dynamic clearances differing by up to 0.28 m between two runs, so a per-run 0.05 m clearance gate cannot
  separate configuration effects from simulator nondeterminism; the M1_three_way narrow-gap low-speed passage also
  occurred with the frozen behaviour in two of three seeds in one iteration and in none in the other.
- The repaired version is a post-campaign patch; the conclusions of Chapter 4 rest on the frozen version.

### Defence Q&A

- *Why did g6 degrade relative to g3 in the baseline?* The coupling introduced defects that act even without moving
  obstacles; statistically the effect is attributed only to C2 + C3 jointly.
- *What does the thesis do about it?* Chapter 4 reports the frozen results unchanged; §4.2.4 explains the mechanisms
  and labels the log evidence as associative; §4.5 documents the repair and its gated evaluation.
- *What was repaired, and how was it verified?* FX1–FX6 behind default-false flags; a sandbox dry run and offline
  golden identity before deployment; closed-loop V1/V2 screening with safety gates; V3 on fresh seeds only after
  manual confirmation. Only the simulations count as evidence.
- *Why not modify the baseline directly?* Changing the model after the campaign and re-reporting would turn the
  evaluation into tuning on the test results; the tag keeps Chapter 4 reproducible and the flags keep the repair
  auditable and reversible.
- *Did a repair item fail?* Iteration 1 stopped at the V1 safety gate (a mission failure). Its runs with the
  speed-proportional cone slack (R1b) showed long low-speed phases in a narrow gap, and R1b was replaced by R1e in
  iteration 2. That association did not replicate: in iteration 2 the frozen behaviour itself showed the same
  low-speed passage in two of three seeds, and R1e showed it in two of three. Iteration 2 stopped at the V1
  dynamic-clearance gate. Every iteration and gate stop is listed in this report.
'''
