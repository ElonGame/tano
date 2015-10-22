# shader compile script
# each shader's entry points are listed, and the script will try to compile
# debug and optimized version for each entry point.
# the script is on a loop, and constantly checks if shaders need to be
# recompiled.

import os
import time
import glob
import subprocess
from collections import OrderedDict, defaultdict
from string import Template
import re

SHADER_DIR = os.path.join('..', 'shaders')
OUT_DIR = os.path.join(SHADER_DIR, 'out')
ENTRY_POINT_TAG = 'entry-point'
SHADERS = {}
SHADER_FILES = set()

SHADER_DECL_RE = re.compile('(.+)? (.+)\(.*')
ENTRY_POINT_RE = re.compile('// entry-point: (.+)')
DEPS_RE = re.compile('#include "(.+?)"')

DEPS = defaultdict(set)


def strip_dirs(f):
    return f.rpartition('\\')[2]


def get_shader_root(f):
    _, tail = os.path.split(f)
    root, _ = os.path.splitext(tail)
    return root


def entry_points_for_file(f):
    # parse all the hlsl files, and look for marked entry points
    global SHADERS
    root = get_shader_root(f)
    # nuke any old entries for the field
    SHADERS[root] = defaultdict(list)
    entry_point_type = None
    for r in open(f, 'rt').readlines():
        r = r.strip()
        if entry_point_type:
            # previous row was an entry point, so parse the entry point
            # name
            m = SHADER_DECL_RE.match(r)
            if m:
                name = m.groups()[1]
                SHADERS[root][entry_point_type].append(name)
            entry_point_type = None
        else:
            m = ENTRY_POINT_RE.match(r)
            if m:
                t = m.groups()[0]
                if t in ('vs', 'ps', 'gs', 'cs'):
                    # found correct entry point tag
                    entry_point_type = t
                else:
                    print 'Unknown tag type: %s' % t

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
    'float': {'type': 'float', 'size': 1},
    'float2': {'type': 'vec2', 'size': 2},
    'float3': {'type': 'vec3', 'size': 3},
    'float4': {'type': 'vec4', 'size': 4},
    'float4x4': {'type': 'Matrix', 'size': 16},
    'matrix': {'type': 'Matrix', 'size': 16},
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

    num_valid = 0
    bufs = []
    for c in cbuffers:
        name = c['name']
        vars = c['vars']

        # skip writing the cbuffer if all the vars are unused
        if len(vars) == c['unused']:
            continue
        num_valid += 1

        cur = '    struct %s\n    {\n' % name

        # calc max line length to align the comments
        max_len = 0
        for n, (var_data, comments) in vars.iteritems():
            t = var_data['type']
            max_len = max(max_len, len(n) + len(t))

        padder = 0
        slots_left = 4
        for n, (var_data, comments) in vars.iteritems():
            var_type = var_data['type']
            var_size = var_data.get('size', None)
            # if the current variable doesn't fit in the remaining
            # slots, align it
            if (
                slots_left != 4 and
                slots_left - var_size < 0
            ):
                cur += (
                    '      float padding%s[%s];\n' %
                    (padder, slots_left)
                )
                padder += 1
            cur_len = len(n) + len(var_type)
            padding = (max_len - cur_len + 8) * ' '
            cur += '      %s %s;%s%s\n' % (var_type, n, padding, comments)
            slots_left -= (var_size % 4)
            if slots_left == 0:
                slots_left = 4
        cur += '    };'

        bufs.append(cur)

    if num_valid:
        res = buffer_template.substitute({'cbuffers': '\n'.join(bufs)})

        # check if this is identical to the previous cbuffer
        identical = False
        try:
            with open(cbuffer_filename, 'rt') as f:
                identical = f.read() == res
        except IOError:
            pass

        if not identical:
            with open(cbuffer_filename, 'wt') as f:
                f.write(res)


def parse_cbuffer(basename, entry_point, out_name, ext):

    filename = out_name + '.' + ext
    cbuffer_filename = (out_name + '.cbuffers.hpp').lower()

    cbuffer_prefix = basename.title().replace('.', '')

    cbuffers = []
    cur_cbuffer = None
    cur_input_sig = None
    try:
        with open(filename) as f:
            lines = f.readlines()
    except:
        return

    skip_count = 0
    for line in lines:
        if skip_count:
            skip_count -= 1
            continue

        if not line.startswith('//'):
            continue
        line = line[3:]
        line = line.strip()

        if line.startswith('cbuffer'):
            name = line[len('cbuffer '):]
            cur_cbuffer = {
                'name': cbuffer_prefix + name,
                'vars': OrderedDict(),
                'unused': 0,
            }
            continue
        elif line.startswith('}'):
            cbuffers.append(cur_cbuffer)
            cur_cbuffer = None
            continue
        elif line.startswith('Input signature:'):
            cur_input_sig = True
            skip_count = 3
            continue

        if cur_input_sig:
            if not line:
                # done with current input sig
                cur_input_sig = None
                continue
            else:
                line = line.split()
                # the 'use' field isn't always set (for sys values?)
                used = None
                if len(line) == 6:
                    name, index, mask, register, sys_value, format = line
                elif len(line) == 7:
                    name, index, mask, register, sys_value, format, used = line

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
    first_tick = True
    for basename, data in SHADERS.iteritems():
        for shader_type, entry_points in data.iteritems():
            profile = shader_data[shader_type]['profile']
            obj_ext = shader_data[shader_type]['obj_ext']
            asm_ext = shader_data[shader_type]['asm_ext']

            # shader_file =  ..\shaders\landscape.landscape
            shader_file = os.path.join(SHADER_DIR, basename)

            # get the time for the file, and all files it depends on
            hlsl_file_name = shader_file + '.hlsl'
            dd = hlsl_file_name.rpartition('\\')[0]
            hlsl_file_time = os.path.getmtime(hlsl_file_name)
            for deps in DEPS[strip_dirs(hlsl_file_name)]:
                full_deps = os.path.join(dd, deps)
                hlsl_file_time = max(
                    hlsl_file_time, os.path.getmtime(full_deps))

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
                    if first_tick:
                        ll = time.localtime()
                        print '==> COMPILE STARTED AT [%.2d:%.2d:%.2d]' % (
                            ll.tm_hour, ll.tm_min, ll.tm_sec)
                        first_tick = False
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

# find any deps
for f in glob.glob(os.path.join(SHADER_DIR, '*.hlsl')):
    for row in open(f).readlines():
        m = DEPS_RE.match(row)
        if m:
            dependant = m.groups()[0]
            DEPS[strip_dirs(f)].add(dependant)

try:
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
except KeyboardInterrupt:
    print 'Exiting'
    exit(1)
