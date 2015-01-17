# shader compile script

import os
import time
import glob
import subprocess
import collections

vs = collections.defaultdict(list)
ps = collections.defaultdict(list)
cs = collections.defaultdict(list)

default_vs = ['particle_tunnel']
default_ps = ['particle_tunnel']
default_cs = []

def old_setup():
    # list all shaders that have non-default entry points
    ps['text_shader'] = ['EdgeDetect']
    ps['tonemap'] = ['LuminanceMap', 'Composite', 'AdaptLuminance', 'BloomThreshold']

    cs['blur'] = ['CopyTranspose', 'BlurTranspose']

    default_vs = ['quad', 'debug_draw', 'fullscreen', 'generator', 'text_shader', 'particle']
    default_ps = ['debug_draw', 'fullscreen', 'generator', 'text_shader', 'copy', 'particle']

for x in default_vs:
    vs[x].append('VsMain')

for x in default_ps:
    ps[x].append('PsMain')

first_run = True

def filetime_is_newer(time, filename):
    global first_run
    if first_run:
        return True
    try:
        obj_time = os.path.getmtime(filename)
        return time > obj_time
    except:
        return True

while True:
    #import pdb; pdb.set_trace()
    for filename in glob.glob('*.hlsl'):
        (cur, e) = os.path.splitext(filename)
        file_time = os.path.getmtime(filename)

        # vertex shaders
        if cur in vs:
            entry_points = vs[cur]
            if filetime_is_newer(file_time, cur + '_' + entry_points[0] + '.vso'):
                for v in entry_points:
                    out_name = cur + '_' + v
                    subprocess.call(['fxc', '/Tvs_5_0', '/Od', '/Zi', ('/E%s' % v), ('/Fo%sD.vso' % out_name), ('/Fc%sD.vsa' % out_name), ('%s.hlsl' % cur)])
                    subprocess.call(['fxc', '/Tvs_5_0', '/O3',        ('/E%s' % v), ('/Fo%s.vso' % out_name),  ('/Fc%s.vsa' % out_name),  ('%s.hlsl' % cur)])

        # pixel shaders
        if cur in ps:
            entry_points = ps[cur]
            if filetime_is_newer(file_time, cur + '_' + entry_points[0] + '.pso'):
                for p in entry_points:
                    out_name = cur + '_' + p
                    subprocess.call(['fxc', '/Tps_5_0', '/Od', '/Zi', ('/E%s' % p), ('/Fo%sD.pso' % out_name), ('/Fc%sD.psa' % out_name), ('%s.hlsl' % cur)])
                    subprocess.call(['fxc', '/Tps_5_0', '/O3',        ('/E%s' % p), ('/Fo%s.pso' % out_name),  ('/Fc%s.psa' % out_name),  ('%s.hlsl' % cur)])

        # compute shaders
        if cur in cs:
            entry_points = cs[cur]
            if filetime_is_newer(file_time, cur + '_' + entry_points[0] + '.cso'):
                for c in entry_points:
                    out_name = cur + '_' + c
                    subprocess.call(['fxc', '/Tcs_5_0', '/Od', '/Zi', ('/E%s' % c), ('/Fo%sD.cso' % out_name), ('/Fc%sD.csa' % out_name), ('%s.hlsl' % cur)])
                    subprocess.call(['fxc', '/Tcs_5_0', '/O3',        ('/E%s' % c), ('/Fo%s.cso' % out_name),  ('/Fc%s.csa' % out_name),  ('%s.hlsl' % cur)])

    first_run = False
    time.sleep(1)
