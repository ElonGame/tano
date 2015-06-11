# shader compile script
# each shader's entry points are listed, and the script will try to compile debug
# and optimized version for each entry point.
# the script is on a loop, and constantly checks if shaders need to be recompiled.

import os
import time
import glob
import subprocess
import collections

SHADER_DIR = os.path.join('..', 'shaders')
OUT_DIR = os.path.join(SHADER_DIR, 'out' )

## shaders and entry points
vs = {
    'particle_tunnel' : ['VsParticle', 'VsQuad', 'VsText', 'VsLines'],
    'basic' : ['VsPos', 'VsPosNormal', 'VsPosColor'],
    'raymarcher' : ['VsQuad'],
    'imgui' : ['VsMain'],
    'quad' : ['VsMain'],
    'cluster' : ['VsMesh'],
    'blob' : ['VsMesh'],
    'landscape' : ['VsQuad', 'VsLandscape', 'VsLandscape2', 'VsBoids', 'VsParticle'],
    'lines': ['VsMain']
}

ps = {
    'particle_tunnel' : ['PsParticle', 'PsBackground', 'PsText', 'PsComposite', 'PsLines'],
    'basic' : ['PsPos', 'PsPosNormal', 'PsPosColor'],
    'raymarcher' : ['PsRaymarcher'],
    'imgui' : ['PsMain'],
    'cluster' : ['PsMesh'],
    'blob' : ['PsMesh'],
    'landscape' : ['PsLandscape', 'PsEdgeDetect', 'PsComposite', 'PsSky', 'PsBoids', 'PsSolidWire', 'PsParticle'],
    'lines': ['PsMain']
}

cs = {
    'blur' : ['CopyTranspose', 'BlurTranspose', 'BoxBlurX', 'BoxBlurY'],
}

gs = {
    'particle_tunnel' : ['GsLines'],
    'landscape' : ['GsSolidWire'],
}

vs_data = { 'shaders': vs, 'profile': 'vs', 'obj_ext': 'vso', 'asm_ext': 'vsa' }
gs_data = { 'shaders': gs, 'profile': 'gs', 'obj_ext': 'gso', 'asm_ext': 'gsa' }
ps_data = { 'shaders': ps, 'profile': 'ps', 'obj_ext': 'pso', 'asm_ext': 'psa' }
cs_data = { 'shaders': cs, 'profile': 'cs', 'obj_ext': 'cso', 'asm_ext': 'csa' }

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

def compile(data):
    profile = data['profile']
    obj_ext = data['obj_ext']
    asm_ext = data['asm_ext']

    for basename, entry_points in data['shaders'].iteritems():
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
                    last_fail_time[shader_file] = hlsl_file_time
                elif shader_file in last_fail_time: 
                    del(last_fail_time[shader_file])

safe_mkdir(OUT_DIR)

while True:
    compile(vs_data)
    compile(gs_data)
    compile(ps_data)
    compile(cs_data)
    time.sleep(1)