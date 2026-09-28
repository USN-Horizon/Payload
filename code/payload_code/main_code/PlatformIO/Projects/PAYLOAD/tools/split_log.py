#!/usr/bin/env python3
"""
split_log.py  -  split a Stack_without_LoRa run file into one CSV per record type.

The firmware writes ONE file per run (/logs/run_NNNNNN.csv on the SD card)
with every sensor mixed together; column 1 is the record type:

    IMU,BARO,MAG,GEIG,RATE,TEMP,STAT,EVT

Lines starting with '#' are the header: one line per record type giving the
column names, which this script reuses.  Rows are sorted by t_us per type
(the pre-trigger dump at launch writes IMU rows that are older than rows
already on the card).

usage:
    python3 tools/split_log.py run_000012.csv            -> run_000012_IMU.csv, ...
    python3 tools/split_log.py run_000012.csv --outdir out/
    python3 tools/split_log.py run_000012.csv --summary   (just print counts)
"""
import argparse, collections, csv, os, re, sys


def parse(path):
    headers = {}            # type -> [column names]
    rows = collections.defaultdict(list)
    meta = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if not line:
                continue
            if line.startswith("#"):
                body = line[1:].strip()
                m = re.match(r"^([A-Z]+),(.+)$", body)
                if m and m.group(1) not in headers:
                    cols = [c.strip() for c in m.group(2).split("(")[0].split(",") if c.strip()]
                    headers[m.group(1)] = cols
                else:
                    meta.append(body)
                continue
            parts = line.split(",")
            t = parts[0]
            if t == "EVT":
                # text may itself contain commas
                parts = [parts[0], parts[1], ",".join(parts[2:])]
            rows[t].append(parts[1:])
    return meta, headers, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--summary", action="store_true")
    a = ap.parse_args()

    meta, headers, rows = parse(a.logfile)
    base = os.path.splitext(os.path.basename(a.logfile))[0]
    outdir = a.outdir or os.path.dirname(os.path.abspath(a.logfile))
    os.makedirs(outdir, exist_ok=True)

    for m in meta:
        print("#", m)
    for t in sorted(rows):
        r = rows[t]
        try:
            r.sort(key=lambda p: int(p[0]))
        except ValueError:
            pass
        t0 = int(r[0][0]) if r and r[0][0].isdigit() else 0
        t1 = int(r[-1][0]) if r and r[-1][0].isdigit() else 0
        span = (t1 - t0) / 1e6
        rate = (len(r) / span) if span > 0 else 0.0
        print(f"{t:5s} {len(r):8d} rows  {span:9.1f} s  {rate:7.1f} /s")
        if a.summary:
            continue
        cols = headers.get(t, [t] + [f"c{i}" for i in range(len(r[0]) - 1)])
        out = os.path.join(outdir, f"{base}_{t}.csv")
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            w.writerows(r)
        print(f"      -> {out}")


if __name__ == "__main__":
    main()
