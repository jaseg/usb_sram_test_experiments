#!/usr/bin/env python3

import re
import binascii
import sqlite3

import modes

PATTERNS = [[0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff],
            [0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff],
            [0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff],
            [0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f],
            [0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33],
            [0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55]]

inv = lambda l: [(~e)&0xff for e in l]
do_hex = lambda b: binascii.hexlify(b).decode()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--database', default='results.sqlite3', help='result sqlite3 database file')
    parser.add_argument('-i', '--intervals', default='400@5ms,10ms,15ms,20ms,25ms,30ms,40ms,50ms,70ms,100ms;750@150ms,200ms,250ms,300ms;200@400ms,500ms,750ms,1000ms;100@1200ms,1500ms,2000ms,2500ms,3000ms,4000ms,5000ms', help='Interval schedule to perform')
    parser.add_argument('-m', '--mode', default='blank,pattern,random', help='Modes to use')
    parser.add_argument('-r', '--random_seeds', default='0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15', help='Comma-separated string seeds to use for random mode')
    args = parser.parse_args()

    runner_map = {
        'blank': [('pattern', '0000000000000000'), ('pattern', 'ffffffffffffffff')],
        'pattern': [('pattern', do_hex(bytes(pattern))) for pattern in PATTERNS] + [('pattern', do_hex(bytes(inv(pattern)))) for pattern in PATTERNS],
        'random': [('random', seed) for seed in args.random_seeds.split(',')]
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

    print('Writing jobs')
    db = sqlite3.connect(args.database)
    with db:
        db.execute('''CREATE TABLE IF NOT EXISTS jobs(
                        oid INTEGER PRIMARY KEY,
                        mode TEXT,
                        mode_settings TEXT,
                        interval_ms INTEGER)''')
        db.execute('''CREATE TABLE IF NOT EXISTS measurements(
                        oid INTEGER PRIMARY KEY,
                        mode TEXT,
                        mode_settings TEXT,
                        interval_ms INTEGER,
                        timestamp INTEGER,
                        temperature REAL,
                        result BLOB)''')
        for num, ivl in interval_schedule:
            for i in range(num):
                for mode, params in mode_schedule:
                    db.execute('INSERT INTO jobs(mode, mode_settings, interval_ms) VALUES (?,?,?)', (mode, params, ivl))
