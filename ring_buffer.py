import numpy as np
from bisect import bisect_left, bisect_right

class RingBuffer(object):
    '''
    Implements a ring buffer that stores only the last CAPACITY added elements.
    '''
    def __init__(self, capacity, default_value=0):
        self.capacity = capacity
        self.arr = [default_value for i in xrange(capacity)]
        self.start_index = 0
        self.cur_size = 0

    def append(self, element):
        self.arr[(self.start_index + self.cur_size) % self.capacity] = element
        if self.cur_size == self.capacity:
            self.start_index += 1
        else:
            self.cur_size += 1

    def __len__(self):
        return self.cur_size

    def index(self, key):
        return (self.start_index + key) % self.capacity

    def __getitem__(self, key):
        assert isinstance(key, int), "Slices are not supported"
        if self.index(key) >= self.cur_size:
            raise IndexError#, "RingBuffer key out of range"
        return self.arr[self.index(key)]

    def __iter__(self):
        for i in xrange(len(self)):
            yield self[i]


def my_bisect(buf, time):
    l = 0
    r = len(buf) - 1 
    while r - l > 1:
        m = (l + r) / 2
        if buf[m].time <= time:
            l = m
        else:
            r = m - 1
    if buf[r] <= time:
        return r
    else:
        return l


def get_interval(buf, time, size):
    if len(buf) == 0:
        return []
    if time < buf[0].time:# or time > buf[len(buf) - 1].time:
        return []
    #print "times: t =", time, "buf:", buf[0].time, "->", buf[len(buf) - 1].time
    x = my_bisect(buf, time)
    l = max(0, x - (size//2))
    r = min(len(buf) - 1, x + (size//2))
    #print "found:", buf[l].time, '->', buf[x].time, '->', buf[r].time
    return [buf[i] for i in xrange(l, r + 1)]
