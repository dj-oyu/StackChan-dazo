# Camera (GC0308) notes — color, resolution, AI vision

Hardware: M5Stack CoreS3, GC0308 VGA DVP sensor, ESP-IDF v6 `esp_video` (V4L2) +
`esp_cam_sensor`. Code: `main/hal/board/stackchan_camera.cc`,
`main/hal/board/stackchan_display.cc`,
`xiaozhi-esp32/main/protocols/gpt_realtime_protocol.cc`.

## Pixel format: the buffer is big-endian RGB565 — do NOT trust the FourCC

The sensor's reported format is a lie on this module. Three sources disagree:
the V4L2 query reports `0x50323234` ("422P"), the raw `dump_raw_frame` buffer
reports the same, yet **the actual bytes only decode correctly as big-endian
RGB565** (verified by dumping a frame and rendering all six candidate
interpretations — only `rgb565_be` produced a real photo; every YUV ordering was
garbage). Treating the bytes as YUYV was the long-standing cause of the
magenta/green cast (Grok literally described a "green-purple marble").

Counter-intuitively, the sensor's nominal **"RGB565" mode (reg `0x24`=`0xa6`,
GC0308 Kconfig index 5) produces corrupt frames** (green / green-magenta speckle),
while its **"YUV422" mode (reg `0x24`=`0xa2`, index 3) streams clean RGB565-BE**.
Register writes do take effect (geometry/subsample come out correct, so SCCB
works) — this module's `0x24` format encoding simply does not match the stock
GC0308 driver's model. So: **keep Kconfig on index 3 (YUV422) for clean data.**

`StackChanCamera::Capture()` then relabels it. In the copy `switch`, the
`V4L2_PIX_FMT_YUV422P` case is handled **exactly like `RGB565X`**: byte-swap each
pixel `BE -> LE` and set `frame_.format = V4L2_PIX_FMT_RGB565`. After that one
relabel, every downstream path is correct and consistent (preview/rotate use
`RGB565_LE`, JPEG/vision encode RGB565), with no per-path reinterpretation.

## Intermittent green frames: lightweight validate-and-retry

This sensor intermittently emits corrupt frames (all-green, or a green-magenta
speckle). `Capture()` validates each frame (after the RGB565 relabel) and retries
on failure — up to 12 validated attempts (`kMaxCaptureAttempts = kWarmupFrames +
12`), ~80–130 ms each. Only a validated frame reaches the user / vision backend.

The detector (`analyze_rgb565_frame` + `looks_like_bad_green_frame`) is built to
be cheap. Corruption decorrelates the green channel from luma while R and B keep
the scene, so the verdict is the **sign of `cov(R,G)`**:

```
C = n*Σ(r*g) − Σr*Σg     // ∝ cov(R,G); clean ⇒ C>0, green-corrupt ⇒ C<0
bad = (C < 0) || (Σg*2 > 3*(Σr+Σb))   // OR extreme green-dominance (all-green has C≈0)
```

This is scale-invariant (no divide / sqrt / variance). Sampling is **5 contiguous
rows spread vertically**: row-contiguous keeps memory access sequential, while
spreading the rows catches partially-updated/torn frames that a head-only block
misses. The inner loop is branch-free (pure accumulation); all comparisons happen
once at the end. It reads `frame_.data` little-endian (post byte-swap).

### Dead ends (do NOT repeat)

- The data is **not** YUV. Cb/Cr swap, hue 180°, planar 422P, YUV matrix/AWB —
  all tried, all wrong. A full-screen R/G/B/white test pattern displayed
  correctly, proving the LVGL/panel path is fine; the bug was 100% in
  interpreting the source bytes.
- Do **not** switch Kconfig to the GC0308 "RGB565" mode (index 5 / reg `0xa6`)
  expecting cleaner data — on this module that mode is the one that corrupts.
- Do **not** count "green pixels" to detect bad frames: a genuinely green subject
  keeps R/G/B correlated (`cov(R,G) > 0`) and must not be rejected; corruption is
  what makes `cov(R,G)` go negative.

## Resolution: use 320×240, not 640×480

640×480 capture is **garbled on this hardware** (DMA/bus can't sustain the
frame) regardless of how the bytes are decoded. Run 320×240:

```
CONFIG_CAMERA_GC0308_DVP_YUV422_320X240_20FPS=y
CONFIG_CAMERA_GC0308_DVP_IF_FORMAT_INDEX_DEFAULT=3
```

The preview path still supports a nearest-neighbour 2:1 downscale for frames
wider than the 320×240 display, but the sensor itself must stay at 320×240.

## Saturation

GC0308 RGB is a little washed out. A software saturation boost (push each
channel away from luma) is easy to add in the RGB565 loop but was intentionally
left **off** — raw colors looked more natural than a 1.5× boost.

## Shutter / camera button touch area

The on-screen camera button (`CreateCameraButton` in `stackchan_display.cc`) is a
48×48 icon. Its hit area is enlarged with `lv_obj_set_ext_click_area(btn, 40)`
so it is easy to tap without changing the visual size.

## Look-and-take: wait for the head to actually stop

`look_and_take_photo` issues a head move (a spring animation ticked by the
`_stackchan_update_task`), then captures. A fixed `vTaskDelay(settle_ms)` was not
enough for large/slow turns, so the shutter fired **mid-rotation** (motion blur /
wrong direction). Both photo paths (`hal_mcp.cpp` and
`CaptureImageForFunctionTool` in `gpt_realtime_protocol.cc`) now **poll
`motion.isMoving()` until the head reaches target** (capped at 4 s so a stalled
servo can't hang the call), then apply a short post-motion `settle_ms` (default
800 ms) for vibration / auto-exposure to settle before capturing.

## AI vision (Grok / xAI) — how it works

Grok realtime voice (`wss`) can't take images, so a function tool
(`stackchan_take_photo` / `stackchan_look_and_take_photo`) captures a photo and
sends it to a **separate** xAI vision HTTP request, then feeds the text
description back as the `function_call_output`.

Key requirements (see `gpt_realtime_protocol.cc`):

- Run the capture on the **main loop** (`Application::Schedule()`), never on the
  WebSocket receive task (blocks realtime audio) and never on a new
  `std::thread` (pthread-create abort under memory pressure).
- **Dedup function calls by `call_id`** — Grok fires both
  `response.function_call_arguments.done` and a `function_call` in
  `response.done` for the same id.
- Vision endpoint = **`/v1/chat/completions`** (OpenAI-compatible,
  `image_url` data URL), model `grok-4.3`, parse `choices[0].message.content`.
- **`reasoning_effort:"none"` is required** — grok-4.3 defaults to "low"
  reasoning which is not capped by `max_completion_tokens`, blowing past the
  60 s timeout.
- Use **`esp_http_client`** (cert bundle) for the POST, not the board's custom
  `HttpClient` (its TLS receive task is starved during realtime audio).
- Encode the JPEG using `frame_.format` directly. After the capture relabel that
  is `V4L2_PIX_FMT_RGB565` (see the pixel-format section), so the vision JPEG is
  RGB565 — verified on device: Grok now reports real colors (e.g. "black bar with
  white and yellow characters") instead of the old magenta/green misread.

### Grok will NOT auto-speak the photo result — by design

Grok's Voice Agent **does not stream audio for a client-issued
`response.create`**. Verified on device across four approaches, all identical:
bare `response.create`, one with `instructions`, an injected user-text turn, and
one with explicit `output_modalities:["audio"]` — each opened a response
(`response.created` → `response.content_part.added`) then went **silent** (no
audio delta, no `response.done`) until the user spoke. Only **server-VAD turns
(real user speech)** produce audio. Muting the mic during/after the tool did not
help, so it is not a mic-streaming race.

**Fallback (current behavior):** after the tool finishes we do NOT send
`response.create`. The visual description is in the conversation via
`function_call_output`, so the user's **next spoken turn** ("何が見えた？") makes
Grok describe the photo from context (correct colors). To prompt the user we
play a chime (`OGG_NEW_NOTIFICATION`) and show a short bubble hint
("「何が見えた？」と聞いてね"). Keep the hint ≤2 lines: the avatar speech bubble
caps at 2 lines and only auto-scrolls overflow for *streamed* replies
(`onSpeechComplete`), not one-shot system text, so a longer hint clips.

Mic suppression (`function_active_`) is kept only **during** the tool run (so
server-VAD can't open a competing turn mid-capture) with a 30 s failsafe in
`SendAudio` in case the touch confirmation is never answered.

Watch: free SRAM dips to ~4–7 KB during the vision call (near OOM).
