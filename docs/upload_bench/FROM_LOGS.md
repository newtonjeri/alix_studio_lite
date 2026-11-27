# Uploads in Lite's logs (2026-10-14 to 2026-11-13)

Written by `tools/upload_bench.py logs` from `Resources/Logs/*/hardware.jsonl` on 2026-11-27.

- Uploads recorded: 179, stored and verified: 179. With the send timed apart from the program-list reads: 176 (files over 5 KB).
- Send rate: 46.3 KB/s with 0.369 s fixed per file (least squares over 176 sends); 747 lines/s at 57.5 bytes a line.
- LEFT: median 42.2 KB/s (33.4 to 45.9), 103 sends.
- RIGHT: median 41.7 KB/s (34.4 to 45.4), 73 sends.
- Biggest file: 898 KB, 16159 lines, send 19.58 s, 19.801 s with the list reads (2026-11-12, left).
- Program-list read before the send: median 0.13 s; read-back after it: median 0.17 s.
- Programs deleted: 15, median 0.41 s each.
- Uploads that overlapped a motion command on the same arm: 0; on the other arm: 0; uploads that overlapped another upload: 0.

The logs answer the rate only. Every upload ran with both arms idle and alone, so whether a controller takes an upload while it runs a Program, or two uploads at once, needs `live`.
