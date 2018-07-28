
import time
import modes
import sqlite3
import bluepill

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('database')
    parser.add_argument('serial')
    parser.add_argument('jobs')
    args = parser.parse_args()

    db = sqlite3.connect(args.database)
    fix = bluepill.BluepillFixture(args.serial or None)

    for job in args.jobs.split(','):
        mode_name, params, ivl = db.execute('SELECT mode, mode_settings, interval_ms FROM jobs WHERE oid=?', (int(job),)).fetchone()
        ivl = int(ivl)
        mode = modes.MODES[mode_name](params)

        mode.write(fix)
        fix.power_cycle(ivl, wait=True, interval=0.03)
        mode.xor(fix)
        data = fix.read()

        flips = sum(bin(x).count('1') for x in data)
        print(f'job {int(job): 9d} {flips-len(data)*8//2: 9d}@{ivl}ms {mode_name:>8}:{params}', flush=True)
        with db:
            db.execute('INSERT INTO measurements(mode, mode_settings, interval_ms, timestamp, temperature, result) VALUES (?,?,?,?,?,?)',
                (mode_name, params, ivl, time.time()*1000, -273, data))
            db.execute('DELETE FROM jobs WHERE oid=?', (int(job),))

