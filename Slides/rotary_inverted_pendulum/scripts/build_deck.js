const fs = require("fs");
const path = require("path");
const PptxGenJS = require("pptxgenjs");
let SHAPE = null;

const C = {
  navy: "1B2A4A",
  blue: "0D6EAF",
  blueMid: "1A7EC8",
  blueLight: "D6E8F5",
  bluePale: "EEF5FB",
  teal: "0A7A6E",
  amber: "D97706",
  amberLight: "FFFBEB",
  amberPale: "FEF3C7",
  green: "047857",
  greenLight: "ECFDF5",
  red: "B91C1C",
  redLight: "FEF2F2",
  white: "FFFFFF",
  offWhite: "F6F8FA",
  border: "DDE3EC",
  textDark: "1B2A4A",
  textMid: "3D5472",
  textGray: "64748B",
  textLight: "94A3B8",
};

function loadManifest(manifestPath) {
  return JSON.parse(fs.readFileSync(manifestPath, "utf8"));
}

function bg(slide, color) {
  slide.background = { color };
}

function accentBar(slide, color) {
  slide.addShape(SHAPE.rect, {
    x: 0,
    y: 0,
    w: 0.13,
    h: 5.625,
    fill: { color },
    line: { color, width: 0 },
  });
}

function titleText(slide, text, y = 0.22, color = C.textDark, size = 26, w = 9.5) {
  slide.addText(text, {
    x: 0.28,
    y,
    w,
    h: 0.62,
    fontSize: size,
    bold: true,
    fontFace: "Calibri",
    color,
    margin: 0,
    fit: "shrink",
  });
}

function subtitleText(slide, text, y = 0.84, color = C.textGray) {
  slide.addText(text, {
    x: 0.28,
    y,
    w: 9.5,
    h: 0.3,
    fontSize: 12,
    fontFace: "Calibri",
    color,
    italic: true,
    margin: 0,
    fit: "shrink",
  });
}

function rule(slide, y = 0.90, color = C.border) {
  slide.addShape(SHAPE.line, {
    x: 0.28,
    y,
    w: 9.44,
    h: 0,
    line: { color, width: 0.75 },
  });
}

function footer(slide, num, total, color = C.blue) {
  slide.addText(`${num} / ${total}`, {
    x: 9.2,
    y: 5.35,
    w: 0.7,
    h: 0.22,
    fontSize: 9,
    fontFace: "Calibri",
    color: C.textLight,
    align: "right",
    margin: 0,
  });
  slide.addShape(SHAPE.rect, {
    x: 0.28,
    y: 5.34,
    w: 9.44,
    h: 0.04,
    fill: { color, transparency: 85 },
    line: { color, width: 0 },
  });
  slide.addShape(SHAPE.rect, {
    x: 0.28,
    y: 5.34,
    w: 9.44 * (num / total),
    h: 0.04,
    fill: { color, transparency: 0 },
    line: { color, width: 0 },
  });
}

function novelBadge(slide) {
  slide.addShape(SHAPE.rect, {
    x: 6.85,
    y: 0.20,
    w: 2.90,
    h: 0.35,
    fill: { color: C.amber },
    line: { color: C.amber, width: 0 },
  });
  slide.addText("KEY CONTRIBUTION", {
    x: 6.85,
    y: 0.20,
    w: 2.90,
    h: 0.35,
    fontSize: 9,
    bold: true,
    fontFace: "Calibri",
    color: C.white,
    align: "center",
    valign: "mid",
    margin: 0,
    charSpace: 1.0,
  });
}

function textBox(slide, el) {
  slide.addText(el.text, {
    x: el.x,
    y: el.y,
    w: el.w,
    h: el.h,
    fontFace: "Calibri",
    fontSize: el.size || 11,
    bold: Boolean(el.bold),
    italic: Boolean(el.italic),
    color: el.color || C.textMid,
    margin: 0,
    align: el.align || "left",
    valign: el.valign || "top",
    fit: "shrink",
    breakLine: false,
  });
}

function bulletBox(slide, el) {
  const bulletText = el.items.map((item) => `• ${item}`).join("\n");
  slide.addText(bulletText, {
    x: el.x,
    y: el.y,
    w: el.w,
    h: el.h,
    fontFace: "Calibri",
    fontSize: el.size || 10,
    color: el.color || C.textMid,
    margin: 0.02,
    breakLine: true,
    fit: "shrink",
    paraSpaceAfterPt: 6,
  });
}

function drawCard(slide, el) {
  slide.addShape(SHAPE.rect, {
    x: el.x,
    y: el.y,
    w: el.w,
    h: el.h,
    fill: { color: el.fill || C.white },
    line: { color: C.border, width: 0.5 },
    radius: 0.06,
  });
  slide.addShape(SHAPE.rect, {
    x: el.x,
    y: el.y,
    w: el.w,
    h: 0.07,
    fill: { color: el.stripe || C.blue },
    line: { color: el.stripe || C.blue, width: 0 },
  });
  if (el.title) {
    slide.addText(el.title, {
      x: el.x + 0.12,
      y: el.y + 0.12,
      w: el.w - 0.24,
      h: 0.24,
      fontFace: "Calibri",
      fontSize: 11.5,
      bold: true,
      color: el.title_color || C.textDark,
      margin: 0,
      fit: "shrink",
    });
  }

  const bodyY = el.title ? el.y + 0.42 : el.y + 0.16;
  const bodyH = el.h - (bodyY - el.y) - 0.12;
  const bodyColor = el.body_color || C.textMid;
  if (el.body) {
    slide.addText(el.body, {
      x: el.x + 0.12,
      y: bodyY,
      w: el.w - 0.24,
      h: bodyH,
      fontFace: "Calibri",
      fontSize: 10,
      color: bodyColor,
      margin: 0,
      fit: "shrink",
      breakLine: true,
    });
  } else if (el.bullets && el.bullets.length) {
    slide.addText(el.bullets.map((item) => `• ${item}`).join("\n"), {
      x: el.x + 0.12,
      y: bodyY,
      w: el.w - 0.24,
      h: bodyH,
      fontFace: "Calibri",
      fontSize: 9.8,
      color: bodyColor,
      margin: 0,
      fit: "shrink",
      breakLine: true,
      paraSpaceAfterPt: 6,
    });
  }
}

function drawTable(slide, el) {
  const rows = [el.headers, ...el.rows];
  const rowCount = rows.length;
  const colCount = el.headers.length;
  const cellW = el.w / colCount;
  const cellH = el.h / rowCount;
  rows.forEach((row, rIdx) => {
    row.forEach((cell, cIdx) => {
      const isHeader = rIdx === 0;
      const fill = isHeader ? (el.fill_header || C.blueLight) : (el.fill_body || C.white);
      slide.addShape(SHAPE.rect, {
        x: el.x + cIdx * cellW,
        y: el.y + rIdx * cellH,
        w: cellW,
        h: cellH,
        fill: { color: fill },
        line: { color: C.border, width: 0.5 },
      });
      slide.addText(String(cell), {
        x: el.x + cIdx * cellW + 0.06,
        y: el.y + rIdx * cellH + 0.05,
        w: cellW - 0.12,
        h: cellH - 0.1,
        fontFace: "Calibri",
        fontSize: isHeader ? 9.5 : 9.0,
        bold: isHeader,
        color: isHeader ? C.textDark : C.textMid,
        margin: 0,
        fit: "shrink",
        breakLine: true,
        valign: "mid",
        align: "center",
      });
    });
  });
}

function drawFlow(slide, el) {
  const nodes = el.nodes || [];
  if (!nodes.length) return;
  const gap = 0.18;
  const nodeW = (el.w - gap * (nodes.length - 1)) / nodes.length;
  const nodeH = el.h * 0.72;
  nodes.forEach((node, idx) => {
    const x = el.x + idx * (nodeW + gap);
    slide.addShape(SHAPE.roundRect, {
      x,
      y: el.y + 0.08,
      w: nodeW,
      h: nodeH,
      fill: { color: node.fill || C.blueLight },
      line: { color: C.border, width: 0.5 },
      radius: 0.08,
    });
    slide.addText(node.label, {
      x: x + 0.06,
      y: el.y + 0.17,
      w: nodeW - 0.12,
      h: nodeH - 0.18,
      fontFace: "Calibri",
      fontSize: nodes.length > 6 ? 9.0 : 10.0,
      bold: true,
      color: C.textDark,
      align: "center",
      valign: "mid",
      margin: 0,
      fit: "shrink",
      breakLine: true,
    });
    if (idx < nodes.length - 1) {
      slide.addText("→", {
        x: x + nodeW + 0.02,
        y: el.y + 0.24,
        w: gap - 0.04,
        h: 0.25,
        fontFace: "Calibri",
        fontSize: 18,
        color: el.arrow_color || C.blue,
        bold: true,
        align: "center",
        margin: 0,
      });
    }
  });
}

function drawSlot(slide, el, label) {
  slide.addShape(SHAPE.rect, {
    x: el.x,
    y: el.y,
    w: el.w,
    h: el.h,
    fill: { color: C.white },
    line: { color: C.border, width: 0.5 },
  });
  slide.addText(label, {
    x: el.x + 0.08,
    y: el.y + 0.06,
    w: 1.0,
    h: 0.16,
    fontFace: "Calibri",
    fontSize: 8,
    color: C.textGray,
    italic: true,
    margin: 0,
  });
}

function renderElement(slide, el) {
  switch (el.type) {
    case "text":
      textBox(slide, el);
      break;
    case "bullets":
      bulletBox(slide, el);
      break;
    case "card":
      drawCard(slide, el);
      break;
    case "table":
      drawTable(slide, el);
      break;
    case "flow":
      drawFlow(slide, el);
      break;
    case "image":
      drawSlot(slide, el, "Figure");
      break;
    case "equation":
      drawSlot(slide, el, "Equation");
      break;
    default:
      throw new Error(`Unsupported element type: ${el.type}`);
  }
}

function renderTitleSlide(pptx, slideDef) {
  const slide = pptx.addSlide();
  bg(slide, C.navy);
  slide.addShape(SHAPE.ellipse, {
    x: 7.8,
    y: -0.6,
    w: 4.0,
    h: 4.0,
    fill: { color: C.blue, transparency: 80 },
    line: { color: C.blue, transparency: 60, width: 0 },
  });
  slide.addShape(SHAPE.ellipse, {
    x: 8.4,
    y: -0.2,
    w: 2.8,
    h: 2.8,
    fill: { color: C.blueMid, transparency: 70 },
    line: { color: C.blueMid, transparency: 60, width: 0 },
  });
  slide.addShape(SHAPE.rect, {
    x: 0,
    y: 5.0,
    w: 10,
    h: 0.625,
    fill: { color: C.blue },
    line: { color: C.blue, width: 0 },
  });
  slide.addShape(SHAPE.rect, {
    x: 0,
    y: 5.0,
    w: 0.9,
    h: 0.625,
    fill: { color: C.amber },
    line: { color: C.amber, width: 0 },
  });
  slide.addText(slideDef.tag || "", {
    x: 0.55,
    y: 0.95,
    w: 7,
    h: 0.32,
    fontSize: 11,
    fontFace: "Calibri",
    color: "B8C8DC",
    charSpace: 2,
    margin: 0,
  });
  slide.addText(slideDef.title, {
    x: 0.55,
    y: 1.65,
    w: 6.3,
    h: 1.35,
    fontSize: 24,
    fontFace: "Calibri",
    bold: true,
    color: C.white,
    margin: 0,
    fit: "shrink",
    breakLine: true,
  });
  slide.addText(slideDef.subtitle, {
    x: 0.55,
    y: 3.0,
    w: 5.7,
    h: 0.75,
    fontSize: 12.5,
    fontFace: "Calibri",
    color: "DCE7F4",
    italic: true,
    margin: 0,
    fit: "shrink",
    breakLine: true,
  });
  slide.addShape(SHAPE.rect, {
    x: 0.55,
    y: 4.12,
    w: 0.5,
    h: 0.04,
    fill: { color: C.amber },
    line: { color: C.amber, width: 0 },
  });
  slide.addText(slideDef.date_line || "", {
    x: 0.55,
    y: 4.32,
    w: 2.0,
    h: 0.25,
    fontSize: 11,
    fontFace: "Calibri",
    color: "B8C8DC",
    margin: 0,
  });
  slide.addText(slideDef.author_line || "", {
    x: 0.55,
    y: 5.05,
    w: 8.0,
    h: 0.42,
    fontSize: 11,
    fontFace: "Calibri",
    color: C.white,
    margin: 0,
  });
}

function renderContentSlide(pptx, slideDef, totalSlides) {
  const slide = pptx.addSlide();
  const isKey = slideDef.key_contribution || slideDef.kind === "key";
  const isConclusion = slideDef.kind === "conclusion";
  if (isConclusion) {
    bg(slide, C.navy);
    slide.addShape(SHAPE.rect, {
      x: 0,
      y: 0,
      w: 10,
      h: 0.08,
      fill: { color: C.amber },
      line: { color: C.amber, width: 0 },
    });
  } else {
    bg(slide, isKey ? C.amberLight : C.offWhite);
    accentBar(slide, isKey ? C.amber : C.blue);
  }

  if (isKey) {
    novelBadge(slide);
    titleText(slide, slideDef.title, 0.22, C.navy, 22, 6.48);
    subtitleText(slide, slideDef.subtitle, 0.84, C.textGray);
    rule(slide, 0.90, C.amber);
  } else if (isConclusion) {
    titleText(slide, slideDef.title, 0.22, C.white, 26, 9.2);
    subtitleText(slide, slideDef.subtitle, 0.84, "B8C8DC");
    rule(slide, 0.90, "36506E");
  } else {
    titleText(slide, slideDef.title);
    subtitleText(slide, slideDef.subtitle);
    rule(slide);
  }

  (slideDef.elements || []).forEach((el) => renderElement(slide, el));
  footer(slide, slideDef.number, totalSlides, isKey || isConclusion ? C.amber : C.blue);
}

async function main() {
  const manifestPath = process.argv[2];
  const outputPath = process.argv[3];
  if (!manifestPath || !outputPath) {
    throw new Error("Usage: node scripts/build_deck.js <manifest.json> <output.pptx>");
  }
  const manifest = loadManifest(manifestPath);
  const pptx = new PptxGenJS();
  SHAPE = pptx.ShapeType;
  pptx.layout = "LAYOUT_16x9";
  pptx.author = "Piyush Tiwari";
  pptx.company = "IIT Kanpur";
  pptx.subject = "Rotary Inverted Pendulum";
  pptx.title = manifest.meta.title;
  pptx.lang = "en-US";
  pptx.theme = {
    headFontFace: "Calibri",
    bodyFontFace: "Calibri",
    lang: "en-US",
  };

  const totalSlides = manifest.slides.length;
  manifest.slides.forEach((slideDef) => {
    if (slideDef.kind === "title") {
      renderTitleSlide(pptx, slideDef);
    } else {
      renderContentSlide(pptx, slideDef, totalSlides);
    }
  });

  await pptx.writeFile({ fileName: path.resolve(outputPath) });
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
