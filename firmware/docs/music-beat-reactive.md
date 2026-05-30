# Music beat-reactive Stack-chan (idle "dance" mode)

While Stack-chan is in **wake-word standby (idle)**, it listens for music on the mic,
locks onto the kick-drum tempo, and reacts: the neon LEDs pulse on the beat (colour =
tempo, brightness = confidence) and the face "headbangs" (nods to the beat) while
randomly switching expression. It never runs during a conversation (the mic is busy),
and it does not move the servos (that would add noise and wear).

## Signal path

```
mic (16 kHz, AFE-raw PCM, idle only)
  -> band-pass biquad 40-75 Hz            (esp-dsp dsps_biquad_*, AudioService)
  -> per-frame energy, ~100 Hz            "kick energy"
  -> decimate x3 -> ~33 Hz                (smoother single peaks, easier onset)
  -> onset = flux > max(floor, K*noise),  with a refractory gap
     (frames while a servo is moving are SKIPPED -- known state, not acoustic)
  -> inter-onset intervals
  -> octave-fold toward their median + median   -> tempo period (fixes half/double)
  -> confidence = how tight the intervals are
  -> PLL: phase anchor nudged toward each onset
  -> hal_bridge::beat_publish(anchor_us, period_us, locked, confidence)
        (consumed by the 50 Hz stackchan update task for LEDs + face)
```

Detection lives in `AudioService::BeatDetectStep()` /
`AudioService::MaybeEmitSpectrum()` (`xiaozhi-esp32/main/audio/audio_service.cc`), runs
on the audio input task (the mic clock), and is gated to the wake-word-running (idle)
state. Servo-noise frames are excluded via `hal_bridge::servo_is_moving()`.

The LED pulse and the face "headbang" are driven from the **stackchan update task**
(`main/hal/hal.cpp`, 50 Hz, owns the neon LEDs + avatar), which reads the shared beat
state and derives beats from `anchor + k*period`.

## What it does

- **Neon LEDs**: flash on each beat, decaying between beats.
  - Colour = tempo (HSV hue): slow ~60 bpm = blue, fast ~180 bpm = red.
  - Peak brightness = confidence, quantized to **4 steps over 60..100** (uncertain /
    wrong beats are dim, so mistakes are barely noticeable).
- **Face "headbang"**: eyes + mouth dip **down** on each beat then ease back up
  (vertical only -- the earlier left/right tilt was removed as it looked odd).
- **Expression**: on ~15 % of beats, switch to a weighted-random emotion
  (Happy 55 % / Neutral 30 % / Doubt 15 %); reset to Neutral on unlock.
- Unlocks ~2.5 s after the music stops (no more kicks) and returns to normal idle.

## Tuning knobs

> The beat detection threshold is expected to be re-tuned. All the knobs are plain
> constants; change + rebuild + flash.

Detection -- `xiaozhi-esp32/main/audio/audio_service.cc` (`kBeat*` constants):

| Constant | Default | Effect |
|---|---|---|
| `kBeatThreshK` | `1.0` | onset threshold = `max(floor, K * noise)`; lower = more sensitive |
| `kBeatThreshFloor` | `2.5` | absolute floor on the flux; lower = more sensitive |
| `kBeatLockSpread` | `0.35` | how loose the intervals may be and still lock; also the confidence denominator |
| `kBeatDecimate` | `3` | 100 Hz -> ~33 Hz detection rate |
| `kBeatRefractoryUs` | `180000` | min spacing between onsets (<= ~330 bpm) |
| `kBeatTimeoutUs` | `2500000` | unlock after this long with no kick |

LED + face -- `main/hal/hal.cpp` (in the `_stackchan_update_task` beat blocks):
- LED hue map (`bpm -> hue`), brightness levels (`60 + lvl*(40/3)`, 4 steps), decay `*0.78`.
- Headbang dip amount `s_hb * 38.0f`, decay `*0.82`.
- Expression switch probability (`esp_random()%100 < 15`) and the weighted choice.

Idle head-movement frequency (separate, also reduces mic servo-noise): `idle_lv` NVS /
`CreateIdleMotionModifier()` in `stackchan_display.cc` (default raised to 10-20 s).

## Dev tool

`tools/spec_viewer.py` (run with `uv run firmware/tools/spec_viewer.py --port COM3`) is a
live PC viewer used during bring-up to find the kick band and validate onset/BPM. It
reads a `[KICK]<m><hh>` serial stream that is **currently disabled** (detection moved
on-device). To use it again, re-enable the `printf("[KICK]...)` line in
`MaybeEmitSpectrum()`.

## Temporary diagnostics still in the tree (remove later)

- Camera frame `[DIAG]` log in `StackChanCamera::Capture()` (`stackchan_camera.cc`) --
  was for the "green frame" survey; logs on every photo.
- `[BEAT] LOCK/UNLOCK` logs in `audio_service.cc` -- handy while tuning; cheap to keep.
- Dead code after the early `return;` in `Application::CheckNewVersion()` (the old
  version-check body, intentionally bypassed -- see below).

## Other changes merged alongside

- **MCP tool unification**: the Grok/OpenAI realtime agent now generates its function
  tools from the MCP registry and dispatches through `McpServer::CallToolSync()` instead
  of a duplicate list; camera vision is injected via `Camera::SetExplainDelegate()`
  (OpenAI = attach image, Grok = `DescribeImageWithGrok`). See [[grok-vision-camera]].
- **Camera head-return**: `look_and_take_photo` returns the head to its pre-photo pose
  before showing the review, disabling `auto_angle_sync` for that move (the live serial
  `getCurrentAngle()` read is unreliable right after capture).
- **Turn-taking sound cues** (esp-dsp synthesized, no WAV assets): a walkie-talkie
  "squelch" cue when the assistant finishes its turn (you/your-turn), plus OK/NG photo
  follow-up wording. Toggle: NVS `agent/cues`.
- **Servo runaway guard**: a failed serial `ReadPos()` (-1) no longer maps to the angle
  limit and teleports the head there. `getCurrentAngle()` returns the last good value on
  a read failure. (Was the cause of sudden big head rotations + USB brown-out.)
- **Startup OTA version check skipped** (`CheckNewVersion()` only marks the partition
  valid) -- faster, network-independent boot. Manual upgrade still works.
- **Camera button lifecycle**: created on conversation enter, destroyed in standby (not
  just hidden), so it frees LVGL resources while idle and never shows during boot.
