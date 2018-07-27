#!/usr/bin/env python3

import sqlite3
import subprocess

def run_schedule(database, serial, batchsize=16):
    db = sqlite3.connect(database)
    i = 0
    while True:
        jobs = db.execute('SELECT oid FROM jobs LIMIT ?', (batchsize,)).fetchall()
        if not jobs:
            return
        jobspec = ','.join(str(job_id) for job_id, in jobs)
        print(f'Spawning worker {i} for jobs {jobspec}')
        subprocess.check_call(['python3', 'worker.py', database, serial or '', jobspec])
        processed, remaining = db.execute('SELECT (SELECT COUNT(*) FROM measurements), (SELECT COUNT(*) FROM jobs)').fetchone()
        print(f'Worker {i} done. {processed} jobs processed, {remaining} jobs to go')
        i += 1


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--database', default='results.sqlite3', nargs='?', help='SQLite3 database file containing jobs generated with plan_schedule.py')
    parser.add_argument('-s', '--serial', default=None, help='bluepill USB serial integer part, i.e. 23 for bp1-0023. Only necessary when using multiple devices at once.')
    parser.add_argument('-b', '--batch-size', type=int, default=16, help='Number of jobs to perform in a single worker invocation')
    args = parser.parse_args()

    run_schedule(args.database, args.serial, args.batch_size)
