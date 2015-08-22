# generate the coeffs for a gaussian blur filter
# this is very much based on the c++ code from the article:
# https://software.intel.com/en-us/blogs/2014/07/15/an-investigation-of-fast-real-time-gpu-based-image-blur-algorithms

import math
import sys


def gen(sigma, kernel_size):
    # generate gaussian curve for the given sigma and kernel size
    kernel = []
    mean = kernel_size / 2
    s2 = sigma * sigma
    for i in range(kernel_size):
        a = pow((i - mean) / sigma, 2)
        b = pow(mean / sigma, 2)
        kernel.append(math.sqrt(math.exp(-0.5 * (a + b)) / (2 * math.pi * s2)))

    s = sum(kernel)
    return [x / s for x in kernel]


def gen_approx(kernel_size):
    # binary search to find a sigma that gives a gaussian curve that
    # just fits the given kernel
    target = 2e-3 / kernel_size
    min_value = 0
    max_value = 1000

    while True:
        sigma = 0.5 * (min_value + max_value)
        attempt = gen(sigma, kernel_size)
        x = attempt[0]

        if abs(x - target) < 0.00001:
            return attempt
        elif x > target:
            max_value = sigma
        else:
            min_value = sigma


def gen_coeffs(kernel_size):
    # generate the shader coeffecients for the given kernel
    assert((kernel_size + 1) % 4 == 0)

    kernel = gen_approx(kernel_size)

    print sum(kernel), kernel
    exit(1)
    half_size = kernel_size / 2

    one_side_inputs = kernel[half_size::-1]
    one_side_inputs[0] *= 0.5

    num_samples = len(one_side_inputs) / 2
    weights = []
    offsets = []
    for i in range(num_samples):
        s = one_side_inputs[i * 2 + 0] + one_side_inputs[i * 2 + 1]
        weights.append(s)
        offsets.append(i * 2 + one_side_inputs[i * 2 + 1] / s)

    return weights, offsets

kernel_size = int(sys.argv[1])

weights, offsets = gen_coeffs(kernel_size)
num_samples = len(weights)
weights = ', '.join([str(f) for f in weights])
offsets = ', '.join([str(f) for f in offsets])

print 'static int num_samples%d = %d;' % (kernel_size, num_samples)
print 'static float weights%d[%d] = {%s};' % (
    kernel_size, num_samples, weights)
print 'static float offsets%d[%d] = {%s};' % (
    kernel_size, num_samples, offsets)
