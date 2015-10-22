import random
from string import Template

count = 4096
random_floats = [str(random.random()) + 'f' for _ in range(count)]
random_ints = [str(random.randint(0, 32767)) for _ in range(count)]
random_gauss_1_01 = [str(random.gauss(1, 0.01)) + 'f' for _ in range(count)]
random_gauss_1_10 = [str(random.gauss(1, 0.1)) + 'f' for _ in range(count)]
random_gauss_1_50 = [str(random.gauss(1, 0.5)) + 'f' for _ in range(count)]

template = """\
static int NUM_VALUES = $count;
static float FLOAT_TABLE[$count] = {$float_table};
static float GAUSS_1_01_TABLE[$count] = {$random_gauss_1_01};
static float GAUSS_1_10_TABLE[$count] = {$random_gauss_1_10};
static float GAUSS_1_50_TABLE[$count] = {$random_gauss_1_50};
static int INT_TABLE[$count] = {$int_table};
"""

t = Template(template)
d = {
    'count': count,
    'float_table': ", ".join(random_floats),
    'int_table': ", ".join(random_ints),
    'random_gauss_1_01': ", ".join(random_gauss_1_01),
    'random_gauss_1_10': ", ".join(random_gauss_1_10),
    'random_gauss_1_50': ", ".join(random_gauss_1_50),
}

print t.substitute(d)
