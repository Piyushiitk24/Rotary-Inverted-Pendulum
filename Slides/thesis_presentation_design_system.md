# MTech Thesis — Presentation Design System
**Author:** Piyush Tiwari · M.Tech (Control & Automation) · IIT Kanpur  
**Applies to:** All three thesis experiments (DC Motor SysID · Rotary Inverted Pendulum · Ball & Beam)  
**Toolchain:** `pptxgenjs` (Node.js) · `npm install pptxgenjs`  
**Layout:** `LAYOUT_16x9` → 10" × 5.625"

---

## 1. Colour Palette

Copy this object verbatim at the top of every presentation script. Do not change these values between projects — this is what makes the three decks look like one person's work.

```javascript
const C = {
  navy:       "1B2A4A",  // dark slides background + primary text
  blue:       "0D6EAF",  // primary accent (bars, buttons, circles)
  blueMid:    "1A7EC8",  // decorative mid-blue
  blueLight:  "D6E8F5",  // tinted panel backgrounds
  bluePale:   "EEF5FB",  // very-light blue row alternation
  teal:       "0A7A6E",  // secondary accent (step 3/4 in flows)
  amber:      "D97706",  // KEY CONTRIBUTION badge + accent bar
  amberLight: "FFFBEB",  // key-contribution slide background
  amberPale:  "FEF3C7",  // key-contribution panel fill
  green:      "047857",  // success / result / selected
  greenLight: "ECFDF5",  // success panel fill
  red:        "B91C1C",  // failure / rejected / before
  redLight:   "FEF2F2",  // failure panel fill
  white:      "FFFFFF",
  offWhite:   "F6F8FA",  // standard content slide background
  border:     "DDE3EC",  // all card / table borders
  textDark:   "1B2A4A",  // headings on light slides
  textMid:    "3D5472",  // body text
  textGray:   "64748B",  // secondary / muted text
  textLight:  "94A3B8",  // captions, page numbers
};
```

**Colour usage rules:**
- Blue is the neutral workhorse — most icons, accent bars, numbered circles.
- Teal is a secondary accent — use for later steps in a pipeline or secondary categories.
- Amber is **reserved exclusively for key/novel contribution slides**. Do not use amber decoratively on regular content slides.
- Green = good result, selected model, success. Red = failure, rejected, baseline.
- Never introduce new colours outside this palette.

---

## 2. Typography

One font throughout: **Calibri**. No exceptions. This keeps all three decks identical.

| Element | `fontFace` | `fontSize` | `bold` | Notes |
|---------|-----------|-----------|--------|-------|
| Slide title | Calibri | 26 pt | true | Dark slides: white. Light slides: `C.textDark` |
| Key-contribution title | Calibri | 20–22 pt | true | Narrowed to `w:6.48"` to avoid badge overlap |
| Subtitle / tagline | Calibri | 12 pt | false | Italic, `C.textGray` |
| Card header | Calibri | 11–12 pt | true | `C.textDark` or accent colour |
| Body / bullet | Calibri | 10–11 pt | false | `C.textMid` |
| Caption / footnote | Calibri | 9–10 pt | false | `C.textGray` or `C.textLight`, often italic |
| Page counter | Calibri | 9 pt | false | `C.textLight`, bottom-right |
| Badge text | Calibri | 9 pt | true | White on coloured badge, `charSpacing:1` |

---

## 3. Slide Background Pattern ("Sandwich")

Use this same three-tier structure in every project:

| Slide position | Background | Accent bar colour |
|----------------|-----------|------------------|
| Title slide (first) | `C.navy` dark | None (decorative circles instead) |
| All content slides | `C.offWhite` (`F6F8FA`) | `C.blue` left bar |
| Key-contribution slides | `C.amberLight` (`FFFBEB`) | `C.amber` left bar |
| Conclusion / contributions (last) | `C.navy` dark | Top amber stripe `h:0.08"` |

The "sandwich" (dark → light → dark) gives visual rhythm and signals transitions.

---

## 4. Fixed Chrome Elements

These appear identically on every content slide in every project. Paste these helper functions into every presentation script without modification.

### 4a. Left accent bar
```javascript
function accentBar(slide, color) {
  slide.addShape(pres.shapes.RECTANGLE, {
    x:0, y:0, w:0.13, h:5.625,
    fill:{ color }, line:{ color, width:0 }
  });
}
// Usage: accentBar(s, C.blue)  — or C.amber for key-contribution slides
```

### 4b. Slide title
```javascript
function titleText(slide, text, y=0.22, color=C.textDark, size=26) {
  slide.addText(text, {
    x:0.28, y, w:9.5, h:0.62,
    fontSize:size, bold:true, fontFace:"Calibri",
    color, margin:0
  });
}
```

### 4c. Subtitle / tagline
```javascript
function subtitleText(slide, text, y=0.84, color=C.textGray) {
  slide.addText(text, {
    x:0.28, y, w:9.5, h:0.28,
    fontSize:12, fontFace:"Calibri",
    color, margin:0, italic:true
  });
}
```

### 4d. Horizontal rule under subtitle
```javascript
function rule(slide, y=0.90, color=C.border) {
  slide.addShape(pres.shapes.LINE, {
    x:0.28, y, w:9.44, h:0,
    line:{ color, width:0.75 }
  });
}
```

### 4e. Progress bar + page counter (footer)
```javascript
function footer(slide, num, total, isKeyContrib=false) {
  const col = isKeyContrib ? C.amber : C.blue;
  slide.addText(`${num} / ${total}`, {
    x:9.2, y:5.35, w:0.7, h:0.22,
    fontSize:9, fontFace:"Calibri", color:C.textLight, align:"right", margin:0
  });
  // filled portion
  slide.addShape(pres.shapes.RECTANGLE, {
    x:0.28, y:5.34, w:(9.44*(num/total)), h:0.04,
    fill:{ color:col }, line:{ color:col, width:0 }
  });
  // unfilled track
  slide.addShape(pres.shapes.RECTANGLE, {
    x:0.28, y:5.34, w:9.44, h:0.04,
    fill:{ color:col, transparency:80 }, line:{ color:col, transparency:80, width:0 }
  });
}
// Usage: footer(s, slideNum, totalSlides)
// Key-contribution slides: footer(s, slideNum, totalSlides, true)
```

### 4f. Content card
```javascript
const mkShadow = () => ({
  type:"outer", blur:6, offset:2, angle:135, color:"000000", opacity:0.08
});

function card(slide, x, y, w, h, fill=C.white, borderColor=C.border) {
  slide.addShape(pres.shapes.RECTANGLE, {
    x, y, w, h,
    fill:{ color:fill },
    line:{ color:borderColor, width:0.5 },
    shadow: mkShadow()
  });
}
// Note: NEVER reuse the mkShadow() object — always call mkShadow() fresh per shape
```

### 4g. Numbered circle (for step flows, lists)
```javascript
function circleNum(slide, x, y, num, fillColor, textColor=C.white) {
  slide.addShape(pres.shapes.OVAL, {
    x, y, w:0.38, h:0.38,
    fill:{ color:fillColor }, line:{ color:fillColor, width:0 }
  });
  slide.addText(String(num), {
    x, y, w:0.38, h:0.38,
    fontSize:13, bold:true, fontFace:"Calibri",
    color:textColor, align:"center", valign:"middle", margin:0
  });
}
```

---

## 5. Key Contribution Slides

Every project has findings that are more original than the rest. These slides get special treatment — **but only use it for genuine contributions, not just important content**.

```javascript
function novelBadge(slide) {
  slide.addShape(pres.shapes.RECTANGLE, {
    x:6.85, y:0.20, w:2.90, h:0.35,
    fill:{ color:C.amber }, line:{ color:C.amber, width:0 }
  });
  slide.addText("★  KEY CONTRIBUTION", {   // ← change label per project if needed
    x:6.85, y:0.20, w:2.90, h:0.35,
    fontSize:9, bold:true, fontFace:"Calibri",
    color:C.white, align:"center", valign:"middle", margin:0, charSpacing:1
  });
}
```

**Key-contribution slide template (combine these):**
```javascript
// In the slide:
bg(s, C.amberLight);          // warm cream background
accentBar(s, C.amber);        // amber left bar (not blue)
novelBadge(s);                // top-right badge
// Title: use w:6.48 not 9.5 to avoid badge overlap
s.addText("Contribution title here", {
  x:0.28, y:0.22, w:6.48, h:0.62,
  fontSize:22, bold:true, fontFace:"Calibri", color:C.navy, margin:0
});
rule(s, 0.90, C.amber);       // amber rule, not grey
footer(s, num, total, true);  // amber progress bar
```

---

## 6. Card Colour Usage

When using coloured top-stripe cards, stay within this subset of palette colours for the stripe:

| Stripe colour | Use for |
|-------------|---------|
| `C.blue` | General/neutral items, hardware, parameters |
| `C.teal` | Secondary steps, analysis stages |
| `C.amber` | Key contribution items only |
| `C.green` | Results, selected models, confirmed outcomes |
| `C.red` | Failure cases, rejected approaches, before-state |
| `C.navy` | Dark contrast cards on dark slides |

Pattern for a coloured-stripe card (used consistently throughout):
```javascript
card(slide, x, y, w, h, C.white);          // white background card
slide.addShape(pres.shapes.RECTANGLE, {    // top stripe
  x, y, w, h:0.07,                        // 0.07" top stripe height
  fill:{ color: C.blue }, line:{ color: C.blue, width:0 }
});
// Then add text inside the card above y+0.10
```

---

## 7. Content Area Boundaries

These safe zones apply identically in all three decks:

| Zone | Value |
|------|-------|
| Slide width | 10.00" |
| Slide height | 5.625" |
| Left margin (after accent bar) | 0.28" |
| Right edge | 9.72" |
| Content width available | 9.44" |
| Top of content (below title+rule) | ≈ 1.02" |
| Bottom of content (above footer) | ≈ 5.28" |
| Content height available | ≈ 4.26" |

---

## 8. Standard Slide Flow (Adapt per Project)

Not all slides are needed in every project, and their order can change. But this is the logical arc used in the DC Motor deck — reference it when planning a new deck so the three presentations feel structurally parallel:

1. **Title** — dark navy, project name, your name, supervisor, date
2. **Motivation / Problem** — why this is hard, what fails naively
3. **Physical Setup** — hardware / system description
4. **End-to-end approach** — pipeline overview (how method connects to deployment)
5–N. **Method slides** — excitation, modelling, identification / design / control (project-specific)
K. **Key Contribution slides** — amber treatment, badge, for novel work only
N+1. **Results / Validation** — real data, overlay plots, metrics
N+2. **Contributions, Limitations & Next Steps** — dark navy conclusion slide

The three projects will naturally differ in the middle (slides 5–N) but the outer frame (title, motivation, setup, pipeline, key contribution, results, conclusion) should feel parallel.

---

## 9. Title Slide Pattern

Same decorative geometry in all three decks (only text changes):

```javascript
bg(s, C.navy);
// Two decorative circles (top right)
s.addShape(pres.shapes.OVAL, {
  x:7.8, y:-0.6, w:4.0, h:4.0,
  fill:{ color:C.blue, transparency:80 }, line:{ color:C.blue, transparency:60, width:0 }
});
s.addShape(pres.shapes.OVAL, {
  x:8.4, y:-0.2, w:2.8, h:2.8,
  fill:{ color:C.blueMid, transparency:70 }, line:{ color:C.blueMid, transparency:60, width:0 }
});
// Bottom bar: full-width blue with amber accent block
s.addShape(pres.shapes.RECTANGLE, {
  x:0, y:5.0, w:10, h:0.625,
  fill:{ color:C.blue }, line:{ color:C.blue, width:0 }
});
s.addShape(pres.shapes.RECTANGLE, {
  x:0, y:5.0, w:0.9, h:0.625,
  fill:{ color:C.amber }, line:{ color:C.amber, width:0 }
});
// Experiment tag (top left)
s.addText("M.Tech Thesis  ·  Experiment N", {
  x:0.55, y:0.95, w:7, h:0.32,
  fontSize:11, fontFace:"Calibri", color:"B8C8DC", charSpacing:2, margin:0
});
// Amber accent line under title
s.addShape(pres.shapes.RECTANGLE, {
  x:0.55, y:4.12, w:0.5, h:0.04,
  fill:{ color:C.amber }, line:{ color:C.amber, width:0 }
});
// Author line in the bottom bar
s.addText("Piyush Tiwari  |  M.Tech (Control & Automation)  |  IIT Kanpur", {
  x:0.55, y:5.05, w:7, h:0.42,
  fontSize:11, fontFace:"Calibri", color:C.white, margin:0
});
```

---

## 10. Conclusion Slide Pattern

Same structure in all three decks:

```javascript
bg(s, C.navy);
s.addShape(pres.shapes.RECTANGLE, {   // thin amber top stripe
  x:0, y:0, w:10, h:0.08,
  fill:{ color:C.amber }, line:{ color:C.amber, width:0 }
});
// Two-column layout:
// Left: contributions (numbered dark cards with amber circle numbers)
// Right: limitations (red text) + next steps (blue arrows)
// Bottom: amber-tinted summary result bar
s.addShape(pres.shapes.RECTANGLE, {
  x:0.28, y:5.00, w:9.44, h:0.54,
  fill:{ color:C.amber, transparency:88 }, line:{ color:C.amber, width:0.5 }
});
```

---

## 11. Inserting Figures from Colab/MATLAB

Use python-pptx (not pptxgenjs) for post-hoc figure insertion to avoid rewriting the full script:

```python
from pptx import Presentation
from pptx.util import Inches
from pptx.dml.color import RGBColor

def white_rect(slide, x, y, w, h):
    shape = slide.shapes.add_shape(1, Inches(x), Inches(y), Inches(w), Inches(h))
    shape.fill.solid()
    shape.fill.fore_color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
    shape.line.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
    shape.line.width = 0

def add_img(slide, path, x, y, w, h):
    slide.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h))

prs = Presentation("deck.pptx")
s = prs.slides[SLIDE_INDEX]   # 0-indexed
white_rect(s, x, y, w, h)     # cover any synthetic chart first
add_img(s, "figure.png", x, y, w, h)
prs.save("deck.pptx")
```

Workflow: build the deck with pptxgenjs (all chrome, text, synthetic charts) → export CSV/PNG figures from Colab → insert with python-pptx.

---

## 12. Quick-Start Checklist for a New Project Deck

- [ ] Copy colour palette `C` verbatim
- [ ] Copy all 7 helper functions (bg, accentBar, titleText, subtitleText, rule, card, footer, circleNum, mkShadow, novelBadge)
- [ ] Use `LAYOUT_16x9`
- [ ] Title slide: navy bg, two blue circles top-right, amber+blue bottom bar
- [ ] Content slides: offWhite bg + blue accent bar + rule at y=0.90
- [ ] Key-contribution slides: amberLight bg + amber accent bar + badge (only for genuine novel work)
- [ ] Conclusion: navy bg + thin amber top stripe
- [ ] Footer on every non-title slide: `footer(s, num, total)`
- [ ] Fonts: Calibri only, sizes from the table in §2
- [ ] Figure insertion: python-pptx post-hoc (don't embed images in JS)
