
## Build

### Fetch Dependencies

```bash
python3 ./fetch_repos.py
```

### Tool Chains

[ESP-IDF v5.5.4](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32s3/index.html)

### Build

```bash
idf.py build
```

### Flash

```bash
idf.py flash
```

## GPT (OpenAI Realtime) provider

In addition to the default Xiaozhi server, this firmware can talk directly to
OpenAI's **Realtime API** (speech-to-speech). Get an API key from
<https://platform.openai.com/>.

### 1. Provide the OpenAI API key

There is **no on-device UI for the API key**; set it one of two ways:

- **Build time (menuconfig):**
  `idf.py menuconfig` → **Xiaozhi Assistant** →
  - `OpenAI API key` (`CONFIG_OPENAI_API_KEY`)
  - `OpenAI Realtime model` (default `gpt-realtime-2`)
  - `OpenAI Realtime voice` (default `alloy`)
  - `OpenAI Realtime instructions` (system prompt)

  ⚠️ The key is then baked into the firmware image. **Do not commit `sdkconfig`
  with a real key** (it would leak it). Prefer NVS below for real keys.

- **Runtime (NVS, no rebuild):** values in the NVS namespace **`openai`**
  override the build-time defaults. Keys: `api_key`, `model`, `voice`,
  `instructions`, `url`. NVS takes precedence; the `CONFIG_*` values are only
  fallbacks. This lets you ship firmware without an embedded key and provision
  per device.

### 2. Select the GPT provider on the device

Open the **Setup** app → **AI provider** → choose **GPT** → **Confirm**
(this writes NVS `agent/provider = "gpt"`; choose **Xiaozhi** to switch back).

Once selected and a key is present, say the wake word ("Hi, Stack-Chan") and talk;
the reply is streamed as speech and shown in the speech bubble. The model, voice
and instructions can be changed without reflashing by editing the `openai` NVS
namespace.

## Grok (xAI Voice Agent) provider

xAI's **Grok Voice Agent API** is OpenAI-Realtime-compatible and uses the same
speech-to-speech path as the GPT provider — only the endpoint
(`wss://api.x.ai/v1/realtime`), the API key and the session config differ. Get an
API key from <https://console.x.ai/>.

### 1. Provide the Grok API key

Same two options as the GPT provider, but in their own namespace:

- **Build time (menuconfig):**
  `idf.py menuconfig` → **Xiaozhi Assistant** →
  - `Grok (xAI) API key` (`CONFIG_GROK_API_KEY`)
  - `Grok voice model` (default `grok-voice-think-fast-1.0`)
  - `Grok voice` (default `eve`; also `ara`, `rex`, `sal`, `leo`, or a custom voice ID)
  - `Grok instructions` (system prompt)

  ⚠️ Same warning as above — the key is baked into the image; prefer NVS for real keys.

- **Runtime (NVS, no rebuild):** values in the NVS namespace **`grok`** override the
  build-time defaults. Keys: `api_key`, `model`, `voice`, `instructions`, `url`.

### 2. Select the Grok provider on the device

Open the **Setup** app → **AI provider** → choose **Grok** → **Confirm**
(this writes NVS `agent/provider = "grok"`).

The model is pinned via the URL query parameter; change it without reflashing by
editing `grok/model` (or `grok/url`) in NVS.
