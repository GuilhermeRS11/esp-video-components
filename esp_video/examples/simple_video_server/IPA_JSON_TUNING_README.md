# 📷 IPA JSON Tuning Guide (IMX708 + ISP)

## 🎯 Overview

**Purpose:**  
The **IPA JSON** describes image-processing and auto-control behavior (exposure/gain, white balance, sharpening/contrast, color) used by the ISP pipeline.  
It lets you tune image quality and convergence speed **without recompiling core algorithms**.  

**Where it is used:**  
The ISP pipeline loads this JSON and feeds parameters to the internal IPA engine each frame.  
In this project, IMX708 always uses:

```
esp_cam_sensor/sensors/imx708/cfg/imx708_default.json
```

**Why it matters:**  
Defaults target “good in most scenes”, but your use case (indoor, outdoor, motion, low light) benefits from explicit choices on:
- ⚡ Stability vs. Speed  
- 📉 Noise vs. Detail  
- 🎨 Color tone and contrast  

---

## 🔄 How It Fits in the Pipeline

```
Capture → ISP statistics → IPA algorithms → Metadata (exposure, gain, color, etc.) → ISP/camera update next frame
```

📈 **Telemetry:**  
The ISP logs compact metadata lines like:
```
[META] exp=xxxxus gain=Y.YY ag=A dg=D
```
to help validate tuning.

---

## 🧩 Top-Level JSON Sections (what they do and trade-offs)

### 1️⃣ `adn` (Denoising)
- 🧠 **What:** Controls raw-domain Bayer-filter denoise.  
- ⚙️ **Key fields:** `level` (strength), `matrix` (3x3 kernel weights), optional per-gain entries.  
- ⚖️ **Trade-offs:**
  - Higher `level` → less noise but may smear fine detail.  
  - Larger center weights → better edges, but possible noise leftover.  
  - Tune per gain: low gain → light denoise; high gain → strong denoise.  

---

### 2️⃣ `aen` (Enhancement)
- 🪞 **What:** Luma-space image enhancement post-ISP (contrast and sharpening).  
- 📊 **Subsections:**
  - `contrast`: Adjusts global contrast.  
  - `sharpen`: Unsharp mask-style sharpening.  
- ⚙️ **Key fields:**
  - `h_thresh`/`l_thresh`: Edge thresholds.  
  - `h_coeff`/`m_coeff`: Sharpening strength.  
  - `matrix`: Kernel for edge enhancement.  
- ⚖️ **Trade-offs:**  
  More sharpening increases crispness ✨ but can cause halos or noise amplification 🔊.

---

### 3️⃣ `agc` (Auto Exposure/Gain Control)
- 📸 **What:** Controls exposure and sensor gain logic.  
- ⚙️ **Fields:**
  - `exposure.frame_delay`: Frames to wait when applying exposure changes.  
  - `adjust_delay`: Extra debounce for smoother updates.  
  - `gain.min_step`: Minimum change in total gain.  
  - `anti_flicker`: `full`, `part`, or `none`.  
  - `ac_freq`: 50 or 60 Hz depending on mains lighting.  
  - `luma_adjust`: Defines AE target levels.  
  - `luma.ae.weight`: 5×5 matrix for metering weights.  

⚖️ **Trade-offs:**
- Faster AE → responsive but may oscillate 🔁  
- `anti_flicker.full` removes banding ⚡ but limits exposure freedom  
- Center-weighted AE favors subjects 👤 but risks background overexposure 🌅  

---

### 4️⃣ `awb` (Auto White Balance)
- 🌈 **What:** Dynamically estimates color temperature and applies red/blue channel gains.  
- ⚙️ **Fields:**
  - `min_counted`, `speed`, `temporal_stability`, `range`, `white_points`.  
- ⚖️ **Trade-offs:**  
  - Higher `speed` → faster adaptation but more flicker 🔄  
  - Higher `temporal_stability` → smoother colors but slower reaction 🕐  
  - Narrow `range` → avoids color swings but limits correction range 🎨  

---

### 5️⃣ `acc` (Auto Color Correction)
- 🎨 **What:** Controls saturation and color correction matrices (CCM).  
- ⚙️ **Fields:**
  - `saturation`: Global or per-temperature color intensity.  
  - `ccm.low_luma`: CCM for low-light protection.  
  - `ccm.table`: CCMs per color temperature.  
- ⚖️ **Trade-offs:**  
  More saturation makes vivid images 🌈 but may overshoot tones or clip colors 💥  

---

## 💡 Why These Defaults

- AE target ≈ 68 🎯 balances brightness and highlight protection.  
- Anti-flicker “part” at 60 Hz ⚡ balances banding and exposure freedom.  
- Exposure delays moderate → faster adaptation with small oscillation risk.  
- AWB stability prioritized → smoother color, less flicker.  
- ACC boosted slightly for pleasing tones and skies ☀️  

---

## 🧪 How To Tune (practical steps)

1️⃣ **Start with AE**  
- Set `agc.luma_adjust.target` for your scene.  
  - Outdoor → lower target (protect highlights).  
  - Indoor → higher target (lift shadows).  
- Adjust delays and steps for your desired responsiveness.  
- Choose `anti_flicker` mode depending on lighting.

2️⃣ **Balance AWB**  
- Increase `temporal_stability` for steady color.  
- Adjust `speed` for faster response.  
- Review `white_points` for your lens/lighting.

3️⃣ **Detail vs. Noise**  
- Increase `aen.sharpen.h_coeff` for crisp edges.  
- Increase `adn.bf.level` at high gain to reduce noise.  

4️⃣ **Color Rendering**  
- Adjust `acc.saturation` to taste 🎨  
- Fine-tune `ccm.table` for accurate skin tones and skies ☁️  

---

## 🔍 Validation Tips

Watch `[META]` logs like:  
```
[META] exp=XXXXus gain=Y.YY ag=A dg=D rG=R bG=B ae_tl=T
```
✅ Ensure exposure and color changes are smooth.  
✅ Check for banding or flicker.  
✅ Validate natural colors in multiple scenes (bright, indoor, low light).  

---

## 📁 Where Is the File and How To Swap It

Default path for IMX708:  
```
esp_cam_sensor/sensors/imx708/cfg/imx708_default.json
```

To create variants (e.g., indoor/outdoor):  
Duplicate the JSON file, adjust values, and update build rules to use your version.

---

## 🧮 Cheat-Sheet: Common Trade-offs

| ⚙️ Parameter | Lower value → | Higher value → |
|--------------|----------------|----------------|
| `frame_delay` | Faster response ⚡ | More stable 📷 |
| `anti_flicker` | More exposure freedom 🌄 | No banding 💡 |
| `denoise.level` | More texture 🪶 | Less noise 🧊 |
| `AE target` | Protects highlights 🌞 | Brighter image 💡 |

---

## 📚 Appendix: Field Reference (as used in default IMX708 JSON)

```
adn.bf[]                → { gain, param:{ level, matrix[9] } }  
aen.contrast[]          → { gain, value }  
aen.sharpen[]           → { gain, param:{ h_thresh, l_thresh, h_coeff, m_coeff, matrix[9] } }  
agc.exposure            → { frame_delay, adjust_delay }  
agc.gain                → { min_step, frame_delay }  
agc.anti_flicker        → { mode, ac_freq }  
agc.luma_adjust         → { target_low, target_high, target }  
awb                     → { speed, temporal_stability, range, white_points[], target }  
acc.saturation[]        → { color_temp, value }  
acc.ccm.table[]         → { color_temp, matrix[9] }  
```
