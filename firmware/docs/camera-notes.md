# Camera (GC0308) notes — color, resolution, AI vision

Hardware: M5Stack CoreS3, GC0308 VGA DVP sensor, ESP-IDF v6 `esp_video` (V4L2) +
`esp_cam_sensor`. Code: `main/hal/board/stackchan_camera.cc`,
`main/hal/board/stackchan_display.cc`,
`xiaozhi-esp32/main/protocols/gpt_realtime_protocol.cc`.

## Preview color: the buffer is RGB565, not YUV422 (root cause of the magenta/green cast)

The captured frame is reported as `V4L2_PIX_FMT_YUYV` (0x56595559) /
sensor format `0x50323234` ("422P"), **but the bytes are actually big-endian
RGB565**. Decoding them as YUYV is what produced the long-standing
magenta/green preview.

Fix (in the `V4L2_PIX_FMT_YUYV` case of `StackChanCamera::Capture()`): reinterpret
each pixel as RGB565 big-endian:

```c
uint16_t v = (src[2*i] << 8) | src[2*i + 1];   // R5 G6 B5
```

Verified on device with uniform color cards:

| Subject | raw bytes | as RGB565-BE | result  |
|---------|-----------|--------------|---------|
| red     | `c1 a5`   | `0xc1a5`     | R=24 G=13 B=5  (red)   |
| green   | `16 a2`   | `0x16a2`     | R=2  G=53 B=2  (green) |

### Dead ends (do NOT repeat)

All of these were tried and are **wrong** — the data was never YUV:
Cb/Cr swap, chroma inversion (hue 180°), `average chroma` reg `0x24` `0x82` vs
`0xa2`, planar 422P layout, the YUV color matrix / AWB gains. A full-screen
R/G/B/white **test pattern displayed correctly**, which proved the LVGL/panel
color path is fine and the bug was 100% in interpreting the source bytes.
GC0308 reg `0x24` is left at the factory value `0xa2`.

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

### Still open (polish)

Grok doesn't auto-speak the result immediately after the photo; the function
output lands in context but the answer comes on the next user turn. Free SRAM
dips to ~4.6 KB during the vision call (near OOM).
