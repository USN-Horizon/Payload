#!/usr/bin/env python3
"""
gs_capture.py - watch / record what the ground-station adapter would forward,
and optionally decode it live exactly like the Qt ground station does.

Needs the Nano 33 IoT running ground_station env:adapter_sim.  That firmware
prints, per received LoRa packet:

    #PKT len=<n> rssi=<dBm> snr=<dB>\\n  <128 raw bytes>  \\n#END\\n

which is the adapter's `Serial.write(received, 128)` wrapped in markers.
This script prints the RSSI/SNR of every packet, stores records of
[len][128-byte blob] in a capture file, and with --decode pipes each record
into the C++ simulator (same mavlink_frame_char parser as Nidhoggr) so the
decoded messages appear in the same terminal.

usage:
    python3 tools/gs_capture.py PORT SECONDS capture.bin [--decode /tmp/gs_decode_sim]

    # build the decoder once:
    g++ -std=c++17 -O1 -Wno-address-of-packed-member \\
        -I include/mavlink/library -I include/Stack_with_LoRa \\
        tools/gs_decode_sim.cpp -o /tmp/gs_decode_sim
    # decode a recording afterwards:
    /tmp/gs_decode_sim --capture capture.bin
"""
import subprocess, sys, time, serial

args = sys.argv[1:]
decoder = None
if "--decode" in args:
    i = args.index("--decode")
    decoder = args[i + 1]
    del args[i:i + 2]
port, seconds, out = args[0], float(args[1]), args[2]

proc = None
if decoder:
    proc = subprocess.Popen([decoder, "--stream"], stdin=subprocess.PIPE)

s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0.05
s.dtr = True; s.rts = True
s.open()
buf = b""
t0 = time.time()
packets = 0
rssi = []
snr = []
try:
    with open(out, "wb") as f:
        while time.time() - t0 < seconds:
            buf += s.read(4096)
            while True:
                i = buf.find(b"#PKT ")
                if i < 0:
                    # print status / error lines, drop the rest
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if line.startswith(b"#"):
                            print(line.decode("ascii", "replace"), flush=True)
                    break
                j = buf.find(b"\n", i)
                if j < 0:
                    break
                header = buf[i:j].decode("ascii", "replace")
                need = j + 1 + 128 + len(b"\n#END\n")
                if len(buf) < need:
                    break
                blob = buf[j + 1:j + 1 + 128]
                trailer = buf[j + 1 + 128:need]
                if trailer != b"\n#END\n":
                    print("framing error, resyncing", flush=True)
                    buf = buf[i + 5:]
                    continue
                try:
                    parts = dict(p.split("=") for p in header.split()[1:])
                    plen = int(parts["len"])
                    rssi.append(float(parts["rssi"])); snr.append(float(parts["snr"]))
                except Exception:
                    plen = 128
                record = bytes([min(plen, 255)]) + blob   # [packet length][adapter buffer]
                f.write(record)
                f.flush()
                packets += 1
                print(f"[{time.time()-t0:6.1f}s] {header}", flush=True)
                if proc:
                    proc.stdin.write(record)
                    proc.stdin.flush()
                buf = buf[need:]
except KeyboardInterrupt:
    pass
finally:
    s.close()
    if proc:
        proc.stdin.close()
        proc.wait()
print(f"--- {packets} packets saved to {out}")
if rssi:
    print(f"    RSSI min/avg/max = {min(rssi):.1f} / {sum(rssi)/len(rssi):.1f} / {max(rssi):.1f} dBm   "
          f"SNR min/avg/max = {min(snr):.1f} / {sum(snr)/len(snr):.1f} / {max(snr):.1f} dB")
