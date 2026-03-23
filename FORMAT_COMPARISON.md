# GPS Log Format Comparison

This document compares the GPS log formats currently supported by the `gps_log` component.

The intent is practical: which format should be enabled for exact speed analytics, compact storage,
tool interoperability, or user-facing export.

## Summary Table

| Format | Current write shape | Time stored | Fidelity | Typical use | Main limitation |
| --- | --- | --- | --- | --- | --- |
| `UBX` | Raw receiver messages, variable size | Receiver-native UBX NAV-PVT time fields | Highest | Ground truth, debugging, future-proof reprocessing | Largest files, parser needed |
| `GPY` | `36` byte full frame + `20` byte compressed delta frame | UTC epoch milliseconds from GNSS | High for speed/time/position | Best compact derivative format used by this project | No altitude field in frame |
| `SBP` | `64` byte header + `32` byte fixed frames | Packed UTC date/time + ms in `UtcSec` | Medium | Compatibility with older GPS speed tools | Quantized fields and reduced metadata |
| `OAO` | `52` byte GNSS frame, optional `512` byte header format exists | UTC epoch milliseconds from GNSS | High for accuracy metadata | Rich derived GNSS archive | Larger than GPY and ecosystem value is narrower |
| `GPX` | XML trackpoints, currently only emitted on normalized full-second samples | UTC text timestamps | Low to medium | Mapping, route sharing, visual inspection | Current writer is effectively `1 Hz` |
| `TXT` | Human-readable text rows | Human-readable timestamp text | Low | Manual inspection, quick logging | Inefficient and not analysis-grade |

## Per-Format Notes

### UBX

- Stores the original u-blox messages with minimal project-side interpretation.
- Best choice when exact replay, protocol debugging, or future parser improvements matter more than file size.
- If storage budget allows, this is the safest archival format.

### GPY

- Binary format originating from RP6conrad's ESP-GPS-Logger and adapted here in a derivative implementation.
- Full frame stores `Unix_time`, `Speed`, `Speed_error`, `Latitude`, `Longitude`, `COG`, `Sat`, `fix`, and `HDOP`.
- Compressed frames store signed deltas against the last full frame and fall back to a full frame after large jumps or a lost NAV-PVT frame.
- Current firmware stores GNSS-derived UTC milliseconds through shared UTC conversion logic, so it is independent of ESP32 local time drift and process timezone.
- Best fit when you want exact speed metrics but do not want UBX-sized logs.

### SBP

- Compact fixed `32` byte frame format with wide compatibility in older GPS speed ecosystems.
- Several fields are deliberately quantized or reduced:
- speed is stored in `0.01 m/s`
- altitude is stored in centimeters
- heading is stored in `0.01 degree`
- HDOP is reduced to `0.2` resolution and limited to `8` bits
- satellite membership is stored as a bitmask, not full per-satellite metadata
- Good when external SBP-compatible tooling is the priority.
- Less suitable when you want to preserve as much GNSS detail as possible.

### OAO

- Fixed `52` byte GNSS frame stores latitude, longitude, altitude, speed, heading, UTC GNSS milliseconds, fix, satellite count, and multiple accuracy terms.
- The format definition also includes a rich `512` byte session header with best-of metrics and signature fields, but the current writer does not emit that header yet.
- In this project OAO is technically richer than SBP, but it is not more compact than GPY and does not preserve raw receiver messages like UBX.
- Best if you specifically want one frame to carry both kinematics and accuracy metadata.

### GPX

- Uses standard XML track points for broad interoperability.
- The current writer logs only when normalized GNSS milliseconds equal zero, so it effectively outputs one point per second even if the receiver runs faster.
- Useful for map tools, route export, and long-duration track viewing.
- Not suitable as the primary format for exact high-rate speed analytics in the current implementation.

### TXT

- Simple human-readable log for manual inspection.
- Helpful for quick debugging on-device or checking that logging works.
- Not intended as a compact or lossless analytics format.

## Recommendations

### If you want the most exact archive

- Use `UBX`.
- It preserves the receiver output with the least project-side transformation.

### If you want compact files but still exact speed/time metrics

- Use `GPY`.
- It is the best balance in this codebase between storage efficiency and analysis fidelity.

### If you need compatibility with older GPS speed tooling

- Use `SBP`.
- Accept the field quantization as the compatibility cost.

### If you need rich per-frame accuracy metadata in a single compact frame

- Use `OAO`.
- This is stronger than SBP on metadata richness, but weaker than UBX on raw completeness and weaker than GPY on compactness.

### If you want route sharing or mapping export

- Use `GPX`.
- Treat it as a presentation/export format, not the canonical analytics format.

## Recommended Logging Sets

| Goal | Recommended formats |
| --- | --- |
| Development and parser work | `UBX` + `GPY` |
| Storage-efficient performance logging | `GPY` |
| Legacy tool interoperability | `SBP` + `GPY` |
| Mapping and user-visible exports | `GPY` + `GPX` |
| Rich accuracy archive experiments | `UBX` + `OAO` |

## Windsurf Recommendation

For this project, windsurf speed logging should default to `GPY` unless there is a specific reason to prefer another format.

Why:

- It preserves millisecond GNSS-derived UTC time.
- It preserves exact speed and position fields needed by the offline metric tools.
- It stays far smaller than raw `UBX` logging.
- It avoids the precision and metadata tradeoffs of `SBP`.
- It is more compact and more directly useful to this project than `OAO` for everyday performance sessions.

The conservative development setup is `UBX` + `GPY`.

That gives:

- raw receiver truth for debugging
- a compact derivative GPY file for routine analysis
- a direct way to compare GPY-derived results against UBX during parser or firmware changes

## Offline Tooling Support

The repository Python tooling under `scripts/` supports all current binary analysis paths:

- `gps_log_analyzer.py`: `SBP`, `UBX`, `GPX`, `OAO`, `GPY`
- `gps_session_metrics.py`: `SBP`, `UBX`, `GPX`, `OAO`, `GPY`

That means `GPY` can participate in the same gap, spike, and session-metric workflows as the other supported formats.