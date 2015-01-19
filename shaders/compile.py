# shader compile script
# each shader's entry points are listed, and the script will try to compile debug
# and optimized version for each entry point.
# the script is on a loop, and constantly checks if shaders need to be recompiled.

import os
import time
import glob
import subprocess
import collections

vs = collections.defaultdict(list)
ps = collections.defaultdict(list)
cs = collections.defaultdict(list)

vs_data = { 'shaders': vs, 'profile': 'vs', 'obj_ext': 'vso', 'asm_ext': 'vsa' }
ps_data = { 'shaders': ps, 'profile': 'ps', 'obj_ext': 'pso', 'asm_ext': 'psa' }
cs_data = { 'shaders': cs, 'profile': 'cs', 'obj_ext': 'cso', 'asm_ext': 'csa' }

## shaders 
vs['particle_tunnel'] = ['VsMain']
ps['particle_tunnel'] = ['PsMain']

vs['imgui'] = ['VsMain']
ps['imgui'] = ['PsMain']


last_fail_time = {}

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

    for filename, entry_points in data['shaders'].iteritems():
        hlsl_file_time = os.path.getmtime(filename + '.hlsl')

        # if the compilation has failed, don't try again if the hlsl file hasn't updated
        if filename in last_fail_time and last_fail_time[filename] == hlsl_file_time:
            continue

        # check for old or missing files (each entry point gets its own file)
        for output, entry_point, is_debug in generate_files(filename, entry_points, obj_ext, asm_ext):
            if filetime_is_newer(hlsl_file_time, output):
                out_name = filename + '_' + entry_point

                if is_debug:
                    # create debug shader
                    # returns 0 on success, > 0 otherwise
                    res = subprocess.call([
                        'fxc', 
                        '/T%s_5_0' % profile,
                        '/Od', 
                        '/Zi', 
                        '/E%s' % entry_point, 
                        '/Fo%sD.%s' % (out_name, obj_ext),
                        '/Fc%sD.%s' % (out_name, asm_ext),
                        '%s.hlsl' % filename])
                else:
                    # create optimized shader
                    res = subprocess.call([
                        'fxc', 
                        '/T%s_5_0' % profile,
                        '/O3',
                        '/E%s' % entry_point, 
                        '/Fo%s.%s' % (out_name, obj_ext),
                        '/Fc%s.%s' % (out_name, asm_ext),
                        '%s.hlsl' % filename])

                if res:
                    # if compilation failed, don't try again until the .hlsl file has been updated
                    last_fail_time[filename] = hlsl_file_time
                elif filename in last_fail_time: 
                    del(last_fail_time[filename])

while True:
    compile(vs_data)
    compile(ps_data)
    compile(cs_data)
    time.sleep(1)