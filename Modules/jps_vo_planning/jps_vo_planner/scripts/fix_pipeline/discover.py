"""Stage 0b: introspective discovery of the project structure. Nothing here names a project file: every role is
found by a content signature (a definition, a launch include, a script reference) in the tracked tree."""
import glob
import os
import re

from fixlib import GateFailure, sh, log, save_json, PIPE_DIR

SRC_EXT = ('.h', '.hpp', '.cpp', '.cc')
PIPELINE_DIRNAME = 'fix_pipeline'  # where the pipeline keeps its own committed copy; never part of the project scan
SELF_DIR = os.path.dirname(os.path.abspath(__file__))


def _read(p):
    try:
        with open(p, errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def _unique(role, cands):
    if len(cands) != 1:
        raise GateFailure('Gate-0 discovery', 'role %s: expected exactly one match, found %d: %s' % (role, len(cands), cands))
    return cands[0]


def _shell_vars(text, env):
    """Resolve simple NAME=value / NAME=${NAME:-default} assignments of a bash script (top level, in order)."""
    out = dict(env)
    for m in re.finditer(r'^([A-Z_]+)=(\S+)', text, flags=re.M):
        name, val = m.group(1), m.group(2)
        d = re.match(r'\$\{%s:-(.*)\}$' % name, val)
        if d:
            val = d.group(1)
        if '$(' in val or '`' in val:
            continue
        for k in sorted(out, key=len, reverse=True):
            val = val.replace('${%s}' % k, out[k]).replace('$%s' % k, out[k])
        out[name] = val.strip('"\'')
    return out


def discover(repo):
    R = {'repo': repo}
    tracked = [os.path.join(repo, f) for f in sh(['git', 'ls-files'], cwd=repo).stdout.split('\n') if f]
    tracked = [f for f in tracked if not f.startswith(SELF_DIR + os.sep) and
               os.sep + PIPELINE_DIRNAME + os.sep not in f]
    src = [f for f in tracked if f.endswith(SRC_EXT)]

    # ---- core module: the header that defines the ablation switch struct, and its package ----
    R['switches_h'] = _unique('switches_h', [f for f in src if f.endswith(('.h', '.hpp')) and
                                             re.search(r'\bstruct\s+Switches\b', _read(f)) and
                                             re.search(r'\bstruct\s+ModelConstants\b', _read(f))])
    d = os.path.dirname(R['switches_h'])
    while d.startswith(repo) and not os.path.isfile(os.path.join(d, 'package.xml')):
        d = os.path.dirname(d)
    if not os.path.isfile(os.path.join(d, 'package.xml')):
        raise GateFailure('Gate-0 discovery', 'no package.xml above %s' % R['switches_h'])
    pkg = R['pkg_root'] = d
    R['pkg_name'] = re.search(r'<name>\s*([^<\s]+)\s*</name>', _read(os.path.join(pkg, 'package.xml'))).group(1)
    psrc = [f for f in src if f.startswith(pkg + os.sep)]
    hdr = [f for f in psrc if f.endswith(('.h', '.hpp'))]
    cpp = [f for f in psrc if f.endswith(('.cpp', '.cc'))]

    def by(role, files, *pats):
        return _unique(role, [f for f in files if all(re.search(p, _read(f)) for p in pats)])

    R['coupling_cpp'] = by('coupling_cpp', cpp, r'\ndouble\s+gammaV\(|\n\s+double\s+gammaV\(', r'double\s+thetaC\(')
    R['global_path_h'] = by('global_path_h', hdr, r'\bclass\s+GlobalPath\b')
    R['global_path_cpp'] = by('global_path_cpp', cpp, r'GlobalPath::costToGo\(')
    R['local_planner_h'] = by('local_planner_h', hdr, r'\bclass\s+LocalPlanner\b')
    R['local_planner_cpp'] = by('local_planner_cpp', cpp, r'LocalPlanner::solve\(')
    R['core_h'] = by('core_h', hdr, r'\bclass\s+PlannerCore\b')
    R['core_cpp'] = by('core_cpp', cpp, r'PlannerCore::replanGlobal\(')
    R['node_h'] = by('node_h', hdr, r'\bclass\s+PlannerNode\b', r'cycles_csv_')
    R['node_cpp'] = by('node_cpp', cpp, r'PlannerNode::globalThreadLoop', r'"switches/enable_gamma"')
    inc_root = os.path.join(pkg, 'include')
    R['inc_switches'] = os.path.relpath(R['switches_h'], inc_root)
    R['inc_core'] = os.path.relpath(R['core_h'], inc_root)

    # ---- CMake: package root list file, core library, offline test loop ----
    R['cmake'] = os.path.join(pkg, 'CMakeLists.txt')
    cm = _read(R['cmake'])
    lib = None
    for m in re.finditer(r'add_library\(\s*(\w+)\s+(?:STATIC\s+|SHARED\s+)?([^)]*)\)', cm):
        srcs = m.group(2).split()
        if os.path.relpath(R['coupling_cpp'], pkg) in srcs:
            lib = (m.group(1), srcs)
    if not lib:
        raise GateFailure('Gate-0 discovery', 'no add_library in %s contains the coupling source' % R['cmake'])
    R['core_lib'], R['core_lib_srcs'] = lib[0], [os.path.join(pkg, s) for s in lib[1]]
    for m in re.finditer(r'add_executable\(\s*(\w+)\s+([^)]*)\)', cm):
        srcs = m.group(2).split()
        if os.path.relpath(R['node_cpp'], pkg) in srcs:
            R['node_target'], R['node_srcs'] = m.group(1), [os.path.join(pkg, s) for s in srcs]
    if 'node_target' not in R:
        raise GateFailure('Gate-0 discovery', 'no add_executable in %s builds the node source' % R['cmake'])
    fm = re.search(r'foreach\(\s*t\s+([^)]*)\)\s*\n\s*add_executable\(\$\{t\}\s+([^)\s]+)\$\{t\}\.cpp\)', cm)
    if not fm:
        raise GateFailure('Gate-0 discovery', 'no offline-test foreach loop in %s' % R['cmake'])
    R['existing_tests'] = fm.group(1).split()
    R['test_dir'] = os.path.normpath(os.path.join(pkg, fm.group(2)))
    mm = re.search(r'target_compile_definitions\(\$\{t\}\s+PRIVATE\s+(\w+)=', cm)
    R['pkg_dir_macro'] = mm.group(1) if mm else ''
    tc = [f for f in glob.glob(os.path.join(R['test_dir'], '*.h')) if 'cylinderFile(' in _read(f) and 'finish()' in _read(f)]
    R['test_common_h'] = _unique('test_common_h', tc)

    # ---- launch files: the one loading the group yaml, and the SITL wrapper that includes it ----
    launches = [f for f in tracked if f.startswith(pkg + os.sep) and f.endswith('.launch')]
    R['launch_node'] = by('launch_node', launches, r'<rosparam[^>]*\$\(arg group\)\.yaml')
    gm = re.search(r'\$\(find\s+%s\)/([^"]*?)/groups/\$\(arg group\)\.yaml' % re.escape(R['pkg_name']), _read(R['launch_node']))
    if not gm:
        raise GateFailure('Gate-0 discovery', 'group rosparam path not parsable in %s' % R['launch_node'])
    R['config_dir'] = os.path.join(pkg, gm.group(1))
    R['groups'] = sorted(os.path.splitext(os.path.basename(g))[0] for g in glob.glob(os.path.join(R['config_dir'], 'groups', '*.yaml'))
                         if 'switches:' in _read(g))
    if not R['groups']:
        raise GateFailure('Gate-0 discovery', 'no group yaml with a switches block under %s' % R['config_dir'])
    R['fixes_dir'] = os.path.join(R['config_dir'], 'fixes')
    R['find_config'] = '$(find %s)/%s' % (R['pkg_name'], gm.group(1))
    node_launch = os.path.basename(R['launch_node'])
    R['launch_sitl'] = by('launch_sitl', launches, r'<include\s+file="[^"]*/%s"' % re.escape(node_launch))

    # ---- experiment drivers: start script (roslaunch of the SITL wrapper), batch driver (uses it, keeps an index) ----
    shs = [f for f in tracked if f.endswith('.sh')]
    sitl = os.path.basename(R['launch_sitl'])
    R['start_script'] = by('start_script', shs, r'roslaunch\s+%s\s+%s' % (re.escape(R['pkg_name']), re.escape(sitl)))
    R['batch_script'] = by('batch_script', shs, re.escape(os.path.basename(R['start_script'])), r'batch_index')
    bt = _read(R['batch_script'])
    V = _shell_vars(bt, {'HOME': os.path.expanduser('~')})
    R['frozen_out'] = os.path.expanduser(V.get('OUT', ''))
    R['frozen_index'] = os.path.join(R['frozen_out'], 'batch_index.csv')
    R['scenario_file'] = V.get('SC', '')
    exp = V.get('EXP', '')
    # the clean-up script is the one the driver runs last, after its main loop
    kills = [os.path.join(exp, n) for n in re.findall(r'"\$EXP/([\w.]+\.sh)"', bt)]
    R['kill_script'] = _unique('kill_script', [k for k in kills[-1:] if 'pkill' in _read(k)])
    pys = re.findall(r'"\$PKG/([\w/]+\.py)"', bt)
    pp = [os.path.join(V['PKG'], p) for p in pys if 'evaluate_run' in _read(os.path.join(V['PKG'], p))]
    R['post_process'] = _unique('post_process', pp)
    vr = [os.path.join(V['PKG'], p) for p in pys if os.path.join(V['PKG'], p) not in pp]
    R['verify_run'] = _unique('verify_run', vr)
    R['check_scenarios'] = by('check_scenarios', [f for f in tracked if f.startswith(pkg + os.sep) and f.endswith('.py')],
                              r'--seeds', r'--d_min')
    for k in ('scenario_file', 'frozen_index'):
        if not os.path.isfile(R[k]):
            raise GateFailure('Gate-0 discovery', '%s does not exist: %s' % (k, R[k]))

    # ---- read-only thesis assets (named by the task) ----
    docs = [os.path.join(d, f) for d, _, fs in os.walk(pkg) for f in fs if f == 'chapter4_ablation_results_phase6.md']
    R['chapter4_doc'] = _unique('chapter4_doc', docs)
    outs = sorted(set(re.findall(r'`(out_[\w]+)/`', _read(R['chapter4_doc']))))
    R['frozen_analysis_dirs'] = [os.path.join(pkg, o) for o in outs if os.path.isdir(os.path.join(pkg, o))]

    # ---- build: catkin workspace and the build space whose source dir holds the package ----
    ws = pkg
    while ws != '/' and not os.path.isfile(os.path.join(ws, '.catkin_workspace')):
        ws = os.path.dirname(ws)
    if ws == '/':
        raise GateFailure('Gate-0 discovery', 'no catkin workspace (.catkin_workspace) above %s' % pkg)
    R['ws'] = ws
    bsp = None
    for cc in glob.glob(os.path.join(ws, 'build', '*', 'CMakeCache.txt')) + glob.glob(os.path.join(ws, 'build', 'CMakeCache.txt')):
        m = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$', _read(cc), flags=re.M)
        if m and (pkg + os.sep).startswith(m.group(1).rstrip('/') + os.sep):
            if bsp is None or len(m.group(1)) > len(bsp[1]):
                bsp = (os.path.dirname(cc), m.group(1))
    if not bsp:
        raise GateFailure('Gate-0 discovery', 'no build space under %s/build has the package in its source dir' % ws)
    R['build_dir'], R['build_src'] = bsp
    R['build_cmd'] = ['catkin_make', '--source', R['build_src'], '--build', R['build_dir']]
    R['devel_setup'] = os.path.join(ws, 'devel', 'setup.bash')
    R['bin_dir'] = os.path.join(ws, 'devel', 'lib', R['pkg_name'])
    R['flags_make'] = os.path.join(R['build_dir'], os.path.relpath(pkg, R['build_src']), 'CMakeFiles',
                                   R['core_lib'] + '.dir', 'flags.make')
    R['node_flags_make'] = os.path.join(os.path.dirname(os.path.dirname(R['flags_make'])), R['node_target'] + '.dir', 'flags.make')
    for k in ('devel_setup', 'flags_make', 'node_flags_make'):
        if not os.path.isfile(R[k]):
            raise GateFailure('Gate-0 discovery', '%s missing: %s' % (k, R[k]))
    return R


def report(R):
    save_json('discovery.json', R)
    lines = ['# Stage 0 discovery report', '', 'Every path below was found at run time by a content signature.', '',
             '| Role | Path |', '|---|---|']
    for k in sorted(R):
        v = R[k]
        if isinstance(v, list):
            v = ', '.join(os.path.relpath(x, R['repo']) if isinstance(x, str) and x.startswith('/') else str(x) for x in v)
        lines.append('| %s | %s |' % (k, v))
    with open(os.path.join(PIPE_DIR, 'discovery.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    log('discovery: %d roles -> %s' % (len(R), os.path.join(PIPE_DIR, 'discovery.md')))
