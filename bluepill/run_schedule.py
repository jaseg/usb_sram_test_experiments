#!/usr/bin/env python3

import sqlite3
import subprocess
import statistics
import time

def run_schedule(database, serial, batchsize=16):
    db = sqlite3.connect(database)
    i = 0
    overheads = []
    t1, t2 = None, time.time()
    while True:
        jobs = db.execute('SELECT oid, interval_ms FROM jobs LIMIT ?', (batchsize,)).fetchall()
        if not jobs:
            return
        jobspec = ','.join(str(job_id) for job_id, ivl in jobs)
        print(f'\033[90mSpawning worker {i} for jobs {jobspec}\033[0m')
        subprocess.check_call(['python3', 'worker.py', database, serial or '', jobspec])
        t1, t2 = t2, time.time()
        overheads.append(((t2-t1) - sum(ivl/1000 for _, ivl in jobs))/batchsize)
        processed, remaining = db.execute('SELECT (SELECT COUNT(*) FROM measurements), (SELECT COUNT(*) FROM jobs)').fetchone()
        avg_overhead = statistics.mean(overheads)
        rem, = db.execute('SELECT SUM(?+interval_ms/1000) FROM jobs', (avg_overhead,)).fetchone()
        print(f'Worker {i} done. \033[92m{processed}\033[0m jobs processed, \033[91m{remaining}\033[0m jobs to go (\033[93m{100*processed/(processed+remaining):>5.3f}% ETA+{rem//3600:2.0f}:{rem//60%60:02.0f}:{rem%60:02.0f}\033[0m)')
        i += 1


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--database', default='results.sqlite3', nargs='?', help='SQLite3 database file containing jobs generated with plan_schedule.py')
    parser.add_argument('-s', '--serial', default=None, help='bluepill USB serial integer part, i.e. 23 for bp1-0023. Only necessary when using multiple devices at once.')
    parser.add_argument('-b', '--batch-size', type=int, default=16, help='Number of jobs to perform in a single worker invocation')
    args = parser.parse_args()

    run_schedule(args.database, args.serial, args.batch_size)
