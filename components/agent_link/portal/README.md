# Provisioning web UI (`portal/`)

These are the WiFi captive-portal pages, embedded into the firmware at build time
(`EMBED_TXTFILES` in [`../CMakeLists.txt`](../CMakeLists.txt)). Edit them to restyle or re-brand
the provisioning experience — **no C changes needed**. Rebuild (`idf.py build`) to pick up edits.

| File | Purpose |
|---|---|
| `portal.html`     | The config page: pick/enter WiFi SSID + password. |
| `connecting.html` | Shown after submit: polls join progress and reports success/failure. |

## Placeholder (filled by the firmware before the page is sent)

- `{{SSIDS}}` — *(portal.html only)* the scanned networks as `<option>` elements, injected into the
  `<select>`. Keep this marker if you replace the file.

Everything else is plain static HTML — the firmware does not template it.

## Bilingual, entirely in the page (no JavaScript, no server round-trip)

Each page ships both languages inline and the toggle is pure CSS — a hidden checkbox flipped by the
button label:

```html
<input type="checkbox" id="lang">           <!-- unchecked = 中文, checked = English -->
...
<span class="zh">中文文案</span><span class="en">English text</span>
```

```css
.en{display:none}                        /* default (中文): hide English */
#lang:checked~.card .en{display:inline}  /* checked (English): show English, */
#lang:checked~.card .zh{display:none}     /*                   hide Chinese  */
```

Because it is a local CSS state change (no navigation, no `fetch`), switching works even in the
phone's captive-portal mini-browser, which often blocks JavaScript and link navigation. To add a
third language, add another class + matching `:checked` rules (or swap the checkbox for radios).

## Constraints

- **Self-contained only** — the phone has no internet while on the SoftAP. Inline all CSS/JS; no
  external fonts, images, or CDNs.
- Keep the form field names `ssid` and `password` — the firmware reads them.
- `connecting.html` polls `GET /status` (JSON: `state`, `ssid`, `ip`, `reason`). `reason` is a short
  code (`badpass` / `notfound` / `fail`) that the page localizes — see the `R` map in its script.
  This live progress is best-effort (needs JS); the device connects regardless of whether it updates.
