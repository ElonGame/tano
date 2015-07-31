# shader compile script
# each shader's entry points are listed, and the script will try to compile
# debug and optimized version for each entry point.
# the script is on a loop, and constantly checks if shaders need to be
# recompiled.

import os
import time
import glob
import subprocess
import collections
from string import Template

SHADER_DIR = os.path.join('..', 'shaders')
OUT_DIR = os.path.join(SHADER_DIR, 'out')
ENTRY_POINT_TAG = 'entry-point'
SHADERS = {}
SHADER_FILES = set()


def get_shader_root(f):
    _, tail = os.path.split(f)
    root, _ = os.path.splitext(tail)
    return root


def entry_points_for_file(f):
    # parse all the hlsl files, and look for marked entry points
    global SHADERS
    root = get_shader_root(f)
    cur = None
    for r in open(f, 'rt').readlines():
        r = r.strip()
        if cur:
            # previous row was an entry point, so parse the entry point
            # name
            _, _, name = r.partition(' ')
            if name:
                name, _, _ = name.partition('(')
                if name:
                    SHADERS.setdefault(root, {}).setdefault(
                        cur, []).append(name)
            cur = None
        else:
            try:
                idx = r.index(ENTRY_POINT_TAG)
                # found entry point tag. check if vs, ps etc
                _, _, t = r[idx:].partition(':')
                if len(t) == 0:
                    print 'error parsing entry point tag'
                    continue
                t = t.strip()
                t = t[:2].lower()
                if t in ('vs', 'ps', 'gs', 'cs'):
                    # found correct entry point tag
                    cur = t
                else:
                    print 'Unknown tag type: %s' % t
            except ValueError:
                pass

shader_data = {
    'vs': {'profile': 'vs', 'obj_ext': 'vso', 'asm_ext': 'vsa'},
    'gs': {'profile': 'gs', 'obj_ext': 'gso', 'asm_ext': 'gsa'},
    'ps': {'profile': 'ps', 'obj_ext': 'pso', 'asm_ext': 'psa'},
    'cs': {'profile': 'cs', 'obj_ext': 'cso', 'asm_ext': 'csa'},
}

last_fail_time = {}


def safe_mkdir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def filetime_is_newer(time, filename):
    try:
        obj_time = os.path.getmtime(filename)
        return time > obj_time
    except:
        return True


def generate_files(base, entry_points, obj_ext, asm_ext):
    # returns the output files from the given base and entry points
    res = []
    for e in entry_points:
        res.append((base + '_' + e + '.' + obj_ext, e, False))
        res.append((base + '_' + e + '.' + asm_ext, e, False))
        res.append((base + '_' + e + 'D.' + obj_ext, e, True))
        res.append((base + '_' + e + 'D.' + asm_ext, e, True))
    return res

# conversion between HLSL and my types
known_types = {
    'float': {'type': 'float', 'alignment': 3},
    'float2': {'type': 'Vector2', 'alignment': 2},
    'float3': {'type': 'Vector3', 'alignment': 1},
    'float4': {'type': 'Vector4'},
    'float4x4': {'type': 'Matrix'},
}

buffer_template = Template("""#pragma once
namespace tano
{
  namespace cb
  {
$cbuffers
  }
}
""")


def dump_cbuffer(cbuffer_filename, cbuffers):

    if len(cbuffers) == 0:
        return

    bufs = []
    for c in cbuffers:
        name = c['name']
        vars = c['vars']

        # skip write the cbuffer if all the vars are unused
        if len(vars) == c['unused']:
            continue

        cur = '    struct %s\n    {\n' % name

        # calc max line length, to align the comments
        max_len = 0
        for n, (tt, comments) in vars.iteritems():
            t = tt['type']
            max_len = max(max_len, len(n) + len(t))

        padder = 0
        for n, (tt, comments) in vars.iteritems():
            t = tt['type']
            alignment = tt.get('alignment', 0)
            cur_len = len(n) + len(t)
            padding = (max_len - cur_len + 8) * ' '
            cur += '      %s %s;%s%s\n' % (t, n, padding, comments)
            if alignment:
                cur += '      float padding%s[%s];\n' % (padder, alignment)
                padder += 1
        cur += '    };'

        bufs.append(cur)

    res = buffer_template.substitute({'cbuffers': '\n'.join(bufs)})

    with open(cbuffer_filename, 'wt') as f:
        f.write(res)


def parse_cbuffer(basename, entry_point, out_name, ext):

    filename = out_name + '.' + ext
    cbuffer_filename = (out_name + '.cbuffers.hpp').lower()

    cbuffer_prefix = basename.title().replace('.', '')

    cbuffers = []
    cur_cbuffer = None
    try:
        with open(filename) as f:
            lines = f.readlines()
    except:
        return

    for line in lines:
        if not line.startswith('//'):
            continue
        line = line[3:]
        line = line.strip()

        if line.startswith('cbuffer'):
            name = line[len('cbuffer '):]
            cur_cbuffer = {
                'name': cbuffer_prefix + name,
                'vars': collections.OrderedDict(),
                'unused': 0,
            }
            continue
        elif line.startswith('}'):
            cbuffers.append(cur_cbuffer)
            cur_cbuffer = None
            continue

        if not cur_cbuffer:
            continue

        tmp, _, comments = line.partition(';')
        comments = comments.strip()
        if comments.find('[unused]') != -1:
            cur_cbuffer['unused'] += 1
        var_type, _, var_name = tmp.partition(' ')
        if not var_type or not var_name:
            continue
        if var_type not in known_types:
            continue
        cur_cbuffer['vars'][var_name] = (known_types[var_type], comments)

    dump_cbuffer(cbuffer_filename, cbuffers)


def compile():
    for basename, data in SHADERS.iteritems():
        for shader_type, entry_points in data.iteritems():
            profile = shader_data[shader_type]['profile']
            obj_ext = shader_data[shader_type]['obj_ext']
            asm_ext = shader_data[shader_type]['asm_ext']

            shader_file = os.path.join(SHADER_DIR, basename)
            hlsl_file_time = os.path.getmtime(shader_file + '.hlsl')

            # if the compilation has failed, don't try again if the hlsl file
            # hasn't updated
            if (
                shader_file in last_fail_time
                and last_fail_time[shader_file] == hlsl_file_time
            ):
                continue

            # check for old or missing files (each entry point gets its own
            # file)
            g = generate_files(basename, entry_points, obj_ext, asm_ext)
            for output, entry_point, is_debug in g:
                if (
                    filetime_is_newer(
                        hlsl_file_time, os.path.join(OUT_DIR, output))
                ):
                    out_name = os.path.join(
                        OUT_DIR, basename + '_' + entry_point)

                    if is_debug:
                        # create debug shader
                        # returns 0 on success, > 0 otherwise
                        res = subprocess.call([
                            'fxc',
                            '/nologo',
                            '/T%s_5_0' % profile,
                            '/Od',
                            '/Zi',
                            '/E%s' % entry_point,
                            '/Fo%sD.%s' % (out_name, obj_ext),
                            '/Fc%sD.%s' % (out_name, asm_ext),
                            '%s.hlsl' % shader_file])
                    else:
                        # create optimized shader
                        res = subprocess.call([
                            'fxc',
                            '/nologo',
                            '/T%s_5_0' % profile,
                            '/O3',
                            '/E%s' % entry_point,
                            '/Fo%s.%s' % (out_name, obj_ext),
                            '/Fc%s.%s' % (out_name, asm_ext),
                            '%s.hlsl' % shader_file])

                    if res:
                        # if compilation failed, don't try again until the
                        # .hlsl file has been updated
                        print '** FAILURE: %s, %s' % (shader_file, entry_point)
                        last_fail_time[shader_file] = hlsl_file_time
                    else:
                        parse_cbuffer(basename, entry_point, out_name, asm_ext)
                        if shader_file in last_fail_time:
                            del(last_fail_time[shader_file])

while True:
    safe_mkdir(OUT_DIR)
    cur_files = set()
    for f in glob.glob(os.path.join(SHADER_DIR, '*.hlsl')):
        if f not in SHADER_FILES:
            entry_points_for_file(f)
            SHADER_FILES.add(f)
        cur_files.add(f)

    # remove any files that no longer exist
    for f in SHADER_FILES.difference(cur_files):
        del SHADERS[get_shader_root(f)]
        SHADER_FILES.remove(f)

    compile()
    time.sleep(1)
