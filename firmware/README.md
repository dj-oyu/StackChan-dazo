
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
