#! /usr/bin/env python3

from collections import defaultdict
import sys
import os

homework_number = sys.argv[1]
base_file = sys.argv[2]

excluded = set()
if len(sys.argv) > 2:
    excluded.update(sys.argv[3:])

homework_title = 'hw%s' % homework_number

base_fname = base_file.split('.')[0]
hw_name = '%s_team_22' % homework_title

hw_func = '''
%% This function is generated automatically by <concat.py>.
function %s(r)
    global simulator
    simulator = 1;

    %s(r);
end
'''

visited = defaultdict(bool)
visited['%s.m' % hw_name] = True

def pour(infp, infn, pf):
    hwfp('%% <concat.py>: concatenating %s ------->\n' % infn)
    for line in infp:
        if 'trplot2' in line:
            pass
        elif 'hold on' in line:
            pass
        elif line.lstrip().startswith('simulator ='):
            pass
        elif line.lstrip().startswith('simulator='):
            pass
        else:
            pf(line.rstrip() + '\n')
    pf('\n')


header = '''%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework {0}
%
% Team number: 22
% Team leader: Zihang Chen (zc2324)
% Team members: Yixing Chen (yc3094), Xin Yang (xy2290)
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%'''.format(
    homework_number
)

with open('%s.m' % hw_name, 'w') as hwf:
    hwfpln = lambda x: print(x, file=hwf)
    hwfp = lambda x: print(x, file=hwf, end='')

    hwfpln(header)

    hwfpln('\n%% This file is concatenated automatically by <concat.py>.')
    hwfp(hw_func % (hw_name, base_fname))
    hwfpln('')

    with open(base_file, 'r') as rf:
        pour(rf, base_file, hwfp)
        visited[base_file] = True

    for r, _, fs in os.walk('.'):
        for f in fs:
            if not visited[f] and f not in excluded:
                visited[f] = True

                if f.endswith('.m'):
                    with open(f, 'r') as fp:
                        pour(fp, f, hwfp)
        break

