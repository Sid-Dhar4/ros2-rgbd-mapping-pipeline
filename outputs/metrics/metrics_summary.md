# Metrics Summary

Sequence: `rgbd_dataset_freiburg1_xyz`

| Metric | Observed value |
|---|---:|
| RGB-D replay rate | ~10 Hz |
| Raw cloud rate | ~10 Hz |
| Filtered cloud rate | ~10 Hz |
| Map cloud publish rate | ~1.66 Hz |
| Raw points per frame | ~14k–15.5k |
| Filtered points per frame | ~1.5k–6k |
| Typical point reduction | ~60–90% |
| Final accumulated map points | ~17.8k |
| Final PCD size | ~279 KB |
| Final PLY size | ~262 KB |

Notes:
- The map publish rate is lower than the frame rate by design.
- The C++ map accumulator processes every 3rd filtered cloud and publishes every 2 processed frames.
- The final map uses TUM ground-truth poses for world-frame accumulation.
- These are first-pass Version 1 metrics from one TUM RGB-D sequence.
