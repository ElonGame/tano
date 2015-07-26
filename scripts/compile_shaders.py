# shader compile script
# each shader's entry points are listed, and the script will try to compile debug
# and optimized version for each entry point.
# the script is on a loop, and constantly checks if shaders need to be recompiled.

import os
import time
import glob
import subprocess
import collections
from string import Template

SHADER_DIR = os.path.join('..', 'shaders')
OUT_DIR = os.path.join(SHADER_DIR, 'out' )

## shaders and entry points
shaders = {
    'basic' : {
        'vs' : ['VsPos', 'VsPosNormal', 'VsPosColor'],
        'ps' : ['PsPos', 'PsPosNormal', 'PsPosColor'],
    },
    'blob' : {
        'vs' : ['VsMesh'],
        'ps' : ['PsMesh'],
    },
    'cluster' : {
        'vs' : ['VsMesh'],
        'ps' : ['PsMesh'],
    },
    'common' : {
        'vs' : ['VsQuad'],
        'ps' : ['PsCopy', 'PsAdd'],
    },
    'common.scale' : {
        'ps' : ['PsScaleBias', 'PsScaleBiasSecondary'],
    },
    'fluid.texture' : {
        'vs' : ['VsMain'],
        'ps' : ['PsMain'],
    },
    'imgui' : {
        'vs' : ['VsMain'],
        'ps' : ['PsMain'],
    },
    'intro.background' : {
        'ps' : ['PsBackground'],
    },
    'intro.composite' : {
        'ps' : ['PsComposite'],
    },
    'intro.fracture' : {
        'vs' : ['VsFracture'],
        'ps' : ['PsFracture'],
    },
    'intro.particle' : {
        'vs' : ['VsParticle'],
        'ps' : ['PsParticle'],
        'gs' : ['GsParticle'],
    },
    'intro.plexus' : {
        'vs' : ['VsLines'],
        'ps' : ['PsLines'],
        'gs' : ['GsLines'],
    },
    'landscape.lensflare' : {
        'ps' : ['PsLensFlare'],
    },
    'landscape.composite' : {
        'ps' : ['PsComposite'],
    },
    'landscape.particle' : {
        'vs' : ['VsParticle'],
        'ps' : ['PsParticle'],
        'gs' : ['GsParticle'],
    },
    'landscape.landscape' : {
        'vs' : ['VsLandscape'],
        'ps' : ['PsLandscape'],
        'gs' : ['GsLandscape'],
    },
    'landscape.sky' : {
        'ps' : ['PsSky'],
    },
    'lines' : {
        'vs' : ['VsMain'],
        'ps' : ['PsMain'],
    },
    'plexus' : {
        'vs' : ['VsLines'],
        'ps' : ['PsLines'],
        'gs' : ['GsLines'],
    },
    'tunnel.lines' : {
        'vs' : ['VsTunnelLines'],
        'ps' : ['PsTunnelLines'],
        'gs' : ['GsTunnelLines'],
    },
    'tunnel.composite' : {
        'ps' : ['PsComposite'],
    },
    'tunnel.mesh' : {
        'vs' : ['VsMesh'],
        'ps' : ['PsMesh'],
    },
    'quad' : {
        'vs' : ['VsMain'],
    },
    'imgui' : {
        'vs' : ['VsMain'],
        'ps' : ['PsMain'],
    },
    'raymarcher' : {
        'ps' : ['PsRaymarcher'],
    },
    'blur' : {
        'cs' : ['CopyTranspose', 'BlurTranspose', 'BoxBlurX'],
    }
}

shader_data = {
    'vs' : { 'profile': 'vs', 'obj_ext': 'vso', 'asm_ext': 'vsa' },
    'gs' : { 'profile': 'gs', 'obj_ext': 'gso', 'asm_ext': 'gsa' },
    'ps' : { 'profile': 'ps', 'obj_ext': 'pso', 'asm_ext': 'psa' },
    'cs' : { 'profile': 'cs', 'obj_ext': 'cso', 'asm_ext': 'csa' },
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
    'float2' : { 'type': 'Vector2', 'alignment': 2 },
    'float3' : { 'type': 'Vector3', 'alignment': 1 },
    'float4' : { 'type': 'Vector4' },
    'float4x4' : { 'type': 'Matrix' },
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
    for name, cbuffer_vars in cbuffers:
        cur =  '    struct %s\n    {\n' % name

        # calc max line length, to align the comments
        max_len = 0
        for n, (tt, comments) in cbuffer_vars.iteritems():
            t = tt['type']
            max_len = max(max_len, len(n) + len(t))

        padder = 0
        for n, (tt, comments) in cbuffer_vars.iteritems():
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

    cbuffer_name = None
    cbuffer_vars = collections.OrderedDict()
    cbuffers = []
    for line in open(filename).readlines():
        if not line.startswith('//'):
            continue
        line = line[3:]
        line = line.strip()

        if line.startswith('cbuffer'):
            cbuffer_name = line[len('cbuffer '):]
            continue
        elif line.startswith('}'):
            cbuffers.append((cbuffer_prefix + cbuffer_name, cbuffer_vars))
            cbuffer_name = None
            cbuffer_vars = collections.OrderedDict()
            continue

        if not cbuffer_name:
            continue

        tmp, _, comments = line.partition(';')
        comments = comments.strip()
        tmp = tmp.strip()
        if len(tmp) == 0:
            continue
        t, _, n = tmp.partition(' ')
        if len(t) == 0 or len(n) == 0:
            continue
        if t not in known_types:
            continue
        cbuffer_vars[n] = (known_types[t], comments)

    dump_cbuffer(cbuffer_filename, cbuffers)


def compile():
    for basename, data in shaders.iteritems():
        for shader_type, entry_points in data.iteritems():
            profile = shader_data[shader_type]['profile']
            obj_ext = shader_data[shader_type]['obj_ext']
            asm_ext = shader_data[shader_type]['asm_ext']

            shader_file = os.path.join(SHADER_DIR, basename)
            hlsl_file_time = os.path.getmtime(shader_file + '.hlsl')

            # if the compilation has failed, don't try again if the hlsl file hasn't updated
            if shader_file in last_fail_time and last_fail_time[shader_file] == hlsl_file_time:
                continue

            # check for old or missing files (each entry point gets its own file)
            for output, entry_point, is_debug in generate_files(basename, entry_points, obj_ext, asm_ext):
                if filetime_is_newer(hlsl_file_time, os.path.join(OUT_DIR, output)):
                    out_name = os.path.join(OUT_DIR, basename + '_' + entry_point)

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
                        # if compilation failed, don't try again until the .hlsl file has been updated
                        print '** FAILURE: %s, %s' % (shader_file, entry_point)
                        last_fail_time[shader_file] = hlsl_file_time
                    else:
                        parse_cbuffer(basename, entry_point, out_name, asm_ext)
                        if shader_file in last_fail_time: 
                            del(last_fail_time[shader_file])

safe_mkdir(OUT_DIR)

while True:
    compile()
    time.sleep(1)