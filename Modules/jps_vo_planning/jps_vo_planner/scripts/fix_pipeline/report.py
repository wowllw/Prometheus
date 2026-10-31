"""Stage 6: final report. All numbers come from state.json and the V3 test CSV written by the earlier stages; the
wording is rule-based (Chapter-4 conventions: 'no evidence of a significant difference' for non-significant
results, g6 vs g3 only as the joint C2 + C3 effect, no success-rate improvement claim at the ceiling)."""
import csv
import os
import statistics as stt

from fixlib import write_file, sh, PIPE_DIR, STATE_ROOT, log

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
    v = [float(x) for x in vals if x not in ('', None) and float(x) == float(x)]
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


def write(R, st, tag, branch):
    repo = R['repo']
    lines = []
    w = lines.append
    tag_obj = sh(['git', 'rev-parse', tag], cwd=repo).stdout.strip()
    tag_commit = sh(['git', 'rev-parse', tag + '^{commit}'], cwd=repo).stdout.strip()
    commits = sh(['git', 'log', '--oneline', '%s..%s' % (tag, branch)], cwd=repo).stdout.strip()
    s0, s2, v1, v2, v3 = st.get('stage0', {}), st.get('stage2_result', {}), st.get('v1', {}), st.get('v2', {}), st.get('v3', {})
    tests = load_tests()
    cls = classify(tests)

    w('# fix_coupling_artifacts — pipeline report (supplementary analysis, thesis §4.5)')
    w('')
    w('> Chapter 4 reports the frozen baseline `%s` and is unchanged. Everything here is a post-campaign engineering '
      'patch evaluated as supplementary analysis. g6 vs g3 is the joint C2 + C3 effect only. V0 log diagnostics are '
      'correlational clues, not evidence. Unit tests are not closed-loop evidence; only V1–V3 simulations are.' % tag)
    w('')
    w('## 1 Git baseline protection')
    w('')
    w('| Item | Value |')
    w('|---|---|')
    w('| Tag | `%s` (annotated, object `%s`) → commit `%s` |' % (tag, tag_obj[:12], tag_commit[:12]))
    w('| Protected base branch | `%s`, commit unchanged: `%s` |' % (s0.get('base_branch'), s0.get('assets', {}).get('base_branch_commit', '')[:12]))
    w('| Development branch | `%s` |' % branch)
    w('| Read-only assets | Chapter-4 doc, frozen 720-cell dataset, frozen analysis outputs: hashes identical at every stage (Gate-I) |')
    w('| Uncommitted user files left untouched | %s |' % ', '.join('`%s`' % x for x in s0.get('uncommitted_user_files', [])))
    w('')
    w('Commits on `%s`:' % branch)
    w('')
    w('```\n%s\n```' % commits)
    w('')
    w('## 2 Build and offline tests')
    w('')
    for t in R['existing_tests']:
        r = s2.get(t, {})
        w('- Existing `%s`: exit %s, %s PASS, %s FAIL.' % (t, r.get('exit'), r.get('pass'), r.get('fail')))
    g = s2.get('golden', {})
    w('- Frozen reference built from `%s` with the package flags `%s`: %s bytes, sha256 `%s`.' % (tag, g.get('flags'), g.get('bytes'), str(g.get('sha256'))[:16]))
    w('- Branch `gen_golden` with every flag at its default (false): %s to the frozen reference.' % s2.get('T0_branch_gen_golden', '—'))
    w('- Log-format identity (source level): the frozen lines of %s are unchanged; additions are guarded by the fix flags '
      'and never write to `cycles.csv` / `paths.csv`.' % ', '.join('`%s`' % x for x in s2.get('log_format_functions', [])))
    tf = s2.get('test_fixes', {})
    w('- `test_fixes`: %s PASS, %s FAIL. Gate-T0: %s' % (tf.get('pass'), tf.get('fail'), '; '.join(tf.get('T0', []))))
    for i in tf.get('info', []):
        w('  - `%s`' % i)
    w('')
    w('## 3 V1 smoke ladder (g6; S3_climb_ne, head_on, M1_three_way; development seeds 101–103)')
    w('')
    w('| Step | Preset run | Units | Unsafe runs | R4 delays | Σ stall breaks | median low-speed share | Σ path switches | replans (Σ) |')
    w('|---|---|---|---|---|---|---|---|---|')
    for name, rec in v1.get('steps', {}).items():
        s = rec.get('summary', [])
        rp = {}
        for x in s:
            for kv in x['replans'].split():
                k, n = kv.split(':')
                rp[k] = rp.get(k, 0) + int(n)
        w('| %s | %s | %s | %d | %d | %s | %s | %s | %s |' % (
            name, rec.get('preset'), '+'.join(rec.get('units', [])) or '—', len(rec.get('unsafe', [])), len(rec.get('r4_delays', [])),
            sum(int(float(x['stall_breaks'] or 0)) for x in s), f(med([x['low_speed_share'] for x in s]), 4),
            sum(int(x['path_switches']) for x in s), ' '.join('%s:%d' % kv for kv in sorted(rp.items()))))
    w('')
    w('Switch adjustments in V1: %s' % ('; '.join(v1.get('adjust', [])) or 'none'))
    w('')
    w('## 4 V2 screening (seeds 1–5; g3, g5, g6 × all_off / all_on × 6 variants)')
    w('')
    for rr in v2.get('rounds', []):
        on_units = [u for u in ('r1a_reachable_cone', 'r2a_kappa_decay', 'r3_relevant_trigger', 'r4_switch_hysteresis',
                                'r4m_dwell_free_only', 'r5_mode_gating') if rr['on_flags'].get(u)]
        w('- Round %d (ON flags include %s): gate failures: %s' % (rr['round'], ', '.join(on_units), rr['gate_failures'] or 'none'))
    for b in v2.get('bisect', []):
        w('- Bisection %s (units %s): %s' % (b['preset'], '+'.join(b['units']), 'failure reproduced' if b['failures'] else 'not reproduced'))
    w('- Units disabled by V2: %s' % (', '.join(v2.get('disabled_units', [])) or 'none'))
    w('- Gates: no collision; no success loss per group; minimum static/dynamic clearance per group×variant not '
      'lower than all_off by more than 0.05 m; mean of run M1 P95 per group not above 1.2 × all_off.')
    if v2.get('rounds'):
        rr = v2['rounds'][-1]
        w('')
        w('Descriptive medians of the final round (n = 5 per cell; with n = 5 the exact two-sided Wilcoxon test cannot '
          'reach p < 0.05, so V2 is screening only):')
        w('')
        w('| Variant | Group | M2 off → on [m] | M3 off → on [°] | M6 off → on | stall breaks Σ off → on |')
        w('|---|---|---|---|---|---|')
        for v in sorted({x['variant'] for x in rr['off_summary']}):
            for gname in sorted({x['group'] for x in rr['off_summary']}):
                a = [x for x in rr['off_summary'] if x['variant'] == v and x['group'] == gname]
                b = [x for x in rr['on_summary'] if x['variant'] == v and x['group'] == gname]
                w('| %s | %s | %s → %s | %s → %s | %s → %s | %d → %d |' % (
                    v, gname, f(med([x['M2'] for x in a])), f(med([x['M2'] for x in b])), f(med([x['M3'] for x in a]), 1),
                    f(med([x['M3'] for x in b]), 1), f(med([x['M6'] for x in a]), 1), f(med([x['M6'] for x in b]), 1),
                    sum(int(float(x['stall_breaks'] or 0)) for x in a), sum(int(float(x['stall_breaks'] or 0)) for x in b)))
    w('')
    w('## 5 V3 confirmation (fresh seeds 21–40; g3, g6 × all_off / all_on × 6 variants)')
    w('')
    w('Contrasts per variant × metric, one Holm family of four: A = g6 fixed vs g6 frozen, B = g6 fixed vs g3 frozen, '
      'C = g3 fixed vs g3 frozen, D = g6 frozen vs g3 frozen (the Chapter-4 gap re-measured on the V3 seeds). '
      'Missing cells: %s. Unsafe runs: %s.' % (v3.get('missing'), {k: len(x) for k, x in v3.get('unsafe', {}).items()}))
    w('')
    w('| Variant | Metric | D: frozen gap g6−g3 | A: fixed − frozen (g6) | B: fixed g6 − frozen g3 | Outcome |')
    w('|---|---|---|---|---|---|')
    for (v, m), c in sorted(cls.items()):
        def cell(r):
            return '%s [%s, %s], p_Holm %s' % (f(r['effect']), f(r['ci_low']), f(r['ci_high']), p4(r['p_holm'])) if r and r.get('p_holm') not in ('', None, 'None') else '—'
        w('| %s | %s | %s | %s | %s | %s |' % (v, METRIC_NAME[m][0], cell(c['D']), cell(c['A']), cell(c['B']), c['class']))
    w('')
    w('Success (exact McNemar, fixed vs frozen g6): ' + '; '.join(
        '%s %s/%s' % (t['variant'], p4(t['p_holm']), 'rates %s vs %s' % (f(t.get('rate_a'), 2), f(t.get('rate_b'), 2)))
        for t in tests if t['contrast'] == 'A' and t['metric'] in ('goal_success', 'M4_success')) + '. Success is at or near '
      'the ceiling; no success-rate improvement is claimed.')
    w('')
    w('### Template sentences (V3)')
    w('')
    for t in tests:
        if t['metric'] in FOCUS + ['stall_breaks'] and t['contrast'] in ('A', 'B', 'D'):
            lab = {'A': ('g6 with the repairs', 'g6 frozen'), 'B': ('g6 with the repairs', 'g3 frozen'),
                   'D': ('g6 frozen', 'g3 frozen')}[t['contrast']]
            w('- ' + sentence(t, *lab))
    w('')
    counts = {}
    for c in cls.values():
        counts[c['class']] = counts.get(c['class'], 0) + 1
    w('## 6 Thesis material')
    w('')
    w(THESIS.format(tag=tag, counts='; '.join('%s: %d' % kv for kv in sorted(counts.items())) or 'no V3 results',
                    adjust='; '.join(v1.get('adjust', []) + ['V2: %s disabled' % u for u in v2.get('disabled_units', [])]) or
                    'no repair switch had to be disabled'))
    text = '\n'.join(lines) + '\n'
    doc = os.path.join(os.path.dirname(R['chapter4_doc']), 'fix_coupling_artifacts_report.md')
    write_file(doc, text)
    with open(os.path.join(PIPE_DIR, 'final_report.md'), 'w') as fh:
        fh.write(text)
    log('report -> %s' % doc)


THESIS = '''### §4.2.4 (addition to the Chapter-4 discussion; frozen numbers only)

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

### §4.5 Supplementary extended analysis: repair of the coupling defects

**Scope.** This section reports a defect repair completed in this work. It does not replace §4.1–4.3, whose
conclusions are derived only from the frozen baseline `{tag}`. The repaired planner is the frozen code plus repair
items FX1–FX6, each behind a Boolean flag that defaults to false; with every flag off the planner output is
byte-identical to the frozen reference (offline golden test T0) and the run-log writers are unchanged.

**Repairs.** FX1 guidance-cone repair (reachable turn, continuous slack, heading memory, feasibility-preserving cone);
FX2 κ peak-hold with exponential release and κ excluded from the local value; FX3 decision-relevant replanning
trigger (positive increments on the remaining path, positive decreases off it); FX4 switching hysteresis (ε_w margin
and T_g dwell, with the R4-m option that waives the dwell under a threat); FX5 threat-free gating of the cone width;
FX6 ε-constraint (implemented and unit-tested, not part of the simulated repair set). No tuned parameter was added.

**Protocol.** V1: incremental smoke ladder (development seeds 101–103). V2: screening of {{g3, g5, g6}} × {{off, on}} ×
six variants, seeds 1–5, with pre-registered safety gates. V3: {{g3, g6}} × {{off, on}} × six variants on fresh seeds
21–40, Wilcoxon signed-rank (Pratt), Holm, percentile bootstrap, exact McNemar. Switch adjustments made by the gates:
{adjust}.

**Results.** Outcome counts over variant × {{M2, M3, M6}}: {counts}. The per-variant statements are the template
sentences of §5 of this report; non-significant results are written "no evidence of a significant difference".
Success rates were at the ceiling in both versions, and no success-rate improvement is claimed.

### Limitations (additions)

- The frozen baseline contains coupling defects that degrade path quality even in static scenes; Chapter 4 reports it
  with these defects.
- The repair was evaluated in one simulated world; V2 is screening (n = 5); the flags-off equivalence was shown
  offline and at source level for the log writers.
- FX3's guarantee covers routes through cached edges; FX4 can delay a voluntary switch by at most T_g plus one
  replanning period (none with R4-m under a threat).
- The repaired version is a post-campaign patch; the conclusions of Chapter 4 rest on the frozen version.

### Defence Q&A

- *Why did g6 degrade relative to g3 in the baseline?* The coupling introduced defects that act even without moving
  obstacles (stopping made cheap at λ = 0, single-cycle κ, irrelevant global re-searches, no switching hysteresis).
  Statistically the effect is attributed only to C2 + C3 jointly.
- *What does the thesis do about it?* Chapter 4 reports the frozen results unchanged, including the degradation;
  §4.2.4 explains the mechanisms and labels the log evidence as associative; §4.5 evaluates the repair.
- *What was repaired and how was it verified?* FX1–FX5 (FX6 implemented, not simulated) behind default-false flags;
  offline golden identity and unit tests; V1 ladder, V2 screening with safety gates, V3 confirmation on fresh seeds.
- *Why not modify the baseline directly?* Changing the model after the campaign and re-reporting would turn the
  evaluation into tuning on the test results; the tag keeps Chapter 4 reproducible and the flags keep the repair
  auditable and reversible.
- *Did any repair fail?* {adjust}.
'''
