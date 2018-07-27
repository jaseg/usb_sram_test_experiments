#!/usr/bin/env python3

import bluepill
import sqlite3
import hashlib
import binascii
import time
import re

PATTERNS = [[0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff],
            [0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff],
            [0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff],
            [0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f],
            [0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33],
            [0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55]]

class Pattern:
    name = 'pattern'

    def __init__(self, pattern):
        self.pattern = pattern
        self.settings = binascii.hexlify(self.pattern).decode()

    def write(self, fix):
        fix.write(pattern=self.pattern)

    def xor(self, fix):
        fix.xor(pattern=self.pattern)

    def __str__(self):
        return f'{self.name}={self.settings}'

class Random:
    name = 'random'

    def __init__(self, seed):
        self.seed = hashlib.sha256(seed.encode()).digest()[:16]
        self.settings = binascii.hexlify(self.seed).decode()

    def write(self, fix):
        fix.write(seed=self.seed)

    def xor(self, fix):
        fix.xor(seed=self.seed)

    def __str__(self):
        return f'{self.name}={self.settings}'

inv = lambda l: [(~e)&0xff for e in l]

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--database', default='results.sqlite3', help='result sqlite3 database file')
    parser.add_argument('-s', '--serial', default=None, help='bluepill USB serial integer part, i.e. 23 for bp1-0023. Only necessary when using multiple devices at once.')
    parser.add_argument('-i', '--intervals', default='1000@20ms,25ms,30ms,40ms,50ms,70ms,100ms;750@150ms,200ms,250ms,300ms;500@400ms,500ms,750ms,1000ms;250@1200ms,1500ms,2000ms,2500ms,3000ms,4000ms,5000ms', help='Interval schedule to perform')
    parser.add_argument('-m', '--mode', default='blank,pattern,random', help='Modes to use')
    parser.add_argument('-r', '--random_seeds', default='0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15', help='Comma-separated string seeds to use for random mode')
    args = parser.parse_args()

    runner_map = {
        'blank': [Pattern(b'\x00\x00\x00\x00\x00\x00\x00\x00'), Pattern(b'\xff\xff\xff\xff\xff\xff\xff\xff')],
        'pattern': [Pattern(bytes(pattern)) for pattern in PATTERNS] + [Pattern(bytes(inv(pattern))) for pattern in PATTERNS],
        'random': [Random(seed) for seed in args.random_seeds.split(',')]
        }

    mode_schedule = args.mode.split(',')
    if len(set(mode_schedule)) != len(mode_schedule) or len(set(mode_schedule + ['blank', 'pattern', 'random'])) != 3:
        raise ValueError(f'Invalid mode schedule {args.mode}')
    mode_schedule = [ runner for mode in mode_schedule for runner in runner_map[mode] ]
    print(f'{len(mode_schedule)} modes selected:')
    for mode in mode_schedule:
        print(f'    {mode}')

    if not re.match('^[0-9]+@[0-9]+ms(,[0-9]+ms)*(;[0-9]+@[0-9]+ms(,[0-9]+ms)*)*$', args.intervals):
        raise ValueError(f'Invalid interval schedule {args.intervals}')
    blocks = [ block.partition('@') for block in args.intervals.split(';') ]
    interval_schedule = [ (int(num), int(ivl[:-2])) for num, _, ivls in blocks for ivl in ivls.split(',') ]
    tot = 0
    print(f'{len(interval_schedule)} intervals selected:')
    for num, ivl in interval_schedule:
        run_ivl_offx = 500 # ms
        dur = (ivl + run_ivl_offx)*num*len(mode_schedule)/1000
        tot += dur
        print(f'{ivl: 5d}ms × {num: 7d} = {dur/3600: 2.0f}:{dur/60%60:02.0f}:{dur%60:02.0f}')
    print(f'Total {tot/3600: 2.0f}:{tot/60%60:02.0f}:{tot%60:02.0f}')

    db = sqlite3.connect(args.database)
    db.execute('''CREATE TABLE IF NOT EXISTS measurements(
                    oid INTEGER PRIMARY KEY,
                    mode TEXT,
                    mode_settings TEXT,
                    interval_ms INTEGER,
                    timestamp INTEGER,
                    temperature REAL,
                    result BLOB)''')

    fix = bluepill.BluepillFixture(args.serial)

    for num, ivl in interval_schedule:
        for i in range(num):
            print(f'Interval {ivl} run {i}')
            for mode in mode_schedule:
                mode.write(fix)
                fix.power_cycle(ivl, wait=True)
                time.sleep(0.5)
                mode.xor(fix)
                data = fix.read()
                flips = sum(bin(x).count('1') for x in data)
                print(f'{flips: 9d} {mode}')
                db.execute('INSERT INTO measurements(mode, mode_settings, interval_ms, timestamp, temperature result) VALUES (?,?,?,?,?,?)',
                        (mode.name, mode.settings, ivl, time.time()*1000, -273, data))

