from __future__ import annotations

import re
from pathlib import Path

from manifest_helpers import bullets, card, equation_slot, flow, image_slot, slide, text


BLUE = "0D6EAF"
TEAL = "0A7A6E"
AMBER = "D97706"
GREEN = "047857"
RED = "B91C1C"
NAVY = "1B2A4A"

REPO_ROOT = Path(__file__).resolve().parents[3]
THESIS_FIG = "thesis/figures"
REPORT_FIG = "reports/thesis/report_thesis_20260209/figures"
NUDGE_TABLE = REPO_ROOT / "thesis" / "tables" / "tab_nudge_steps.tex"

NUDGE_ROW_RE = re.compile(
    r"^\s*(\d+)\s*&\s*([-\d.]+)\s*&\s*([^&]+)\s*&\s*([-\d.]+)\s*&\s*([^&]+)\s*&\s*([-\d.]+)\s*&\s*([-\d.]+)\s*\\\\"
)


def _thesis_png(name: str) -> str:
    return f"{THESIS_FIG}/{name}"


def _report_png(name: str) -> str:
    return f"{REPORT_FIG}/{name}"


def _load_nudge_rows() -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for line in NUDGE_TABLE.read_text(encoding="utf-8").splitlines():
        match = NUDGE_ROW_RE.match(line)
        if not match:
            continue
        rows.append(
            {
                "step": match.group(1),
                "target": match.group(2),
                "rise": match.group(3).strip(),
                "overshoot": match.group(4),
                "settle": match.group(5).strip(),
                "sse": match.group(6),
                "max_alpha": match.group(7),
            }
        )
    if len(rows) != 3:
        raise RuntimeError(f"Expected 3 nudge rows in {NUDGE_TABLE}, found {len(rows)}")
    return rows


def _nudge_metric_body() -> str:
    rows = _load_nudge_rows()
    return "\n".join(
        [
            f"{rows[0]['target']} deg: SSE {rows[0]['sse']} deg, max |alpha| {rows[0]['max_alpha']} deg",
            f"{rows[1]['target']} deg: rise {rows[1]['rise']} s, overshoot {rows[1]['overshoot']} deg, SSE {rows[1]['sse']} deg",
            f"{rows[2]['target']} deg: rise {rows[2]['rise']} s, overshoot {rows[2]['overshoot']} deg, SSE {rows[2]['sse']} deg",
            "Profile: vmax = 12 deg/s, amax = 60 deg/s^2",
        ]
    )


def get_manifest() -> dict:
    slides = [
        {
            "number": 1,
            "kind": "title",
            "title": "Rotary (Furuta) Inverted Pendulum",
            "subtitle": "Modelling and experimental comparison of linear and sliding-mode controllers",
            "tag": "M.Tech Thesis  ·  Experiment 2",
            "author_line": "Piyush Tiwari  |  M.Tech (Control & Automation)  |  IIT Kanpur",
            "date_line": "April 2026",
            "elements": [],
        },
        slide(
            2,
            "content",
            "Problem Statement",
            "",
            elements=[
                image_slot("rig_photo", _thesis_png("rig_photo.png"), 0.45, 1.02, 6.2, 4.18),
                bullets(
                    6.95,
                    1.45,
                    2.25,
                    2.2,
                    [
                        "Balance upright on a real stepper-driven Furuta rig",
                        "Estimate all 4 states from dual encoders and filtering",
                        "Compare 3 controllers on one platform",
                    ],
                    size=12,
                    color="1B2A4A",
                ),
            ],
        ),
        slide(
            3,
            "content",
            "Hardware Overview",
            "",
            elements=[
                image_slot("rig_photo", _thesis_png("rig_photo.png"), 0.45, 1.0, 5.45, 4.2),
                card(6.15, 1.08, 1.52, 0.82, title="Arduino Mega", stripe=BLUE, fill="FFFFFF", title_size=10.2),
                card(7.83, 1.08, 1.52, 0.82, title="TMC2209", stripe=TEAL, fill="FFFFFF", title_size=10.2),
                card(6.15, 2.18, 1.52, 0.82, title="NEMA17", stripe=AMBER, fill="FFFFFF", title_size=10.2),
                card(7.83, 2.18, 1.52, 0.82, title="Dual AS5600s", stripe=BLUE, fill="FFFFFF", title_size=10.2),
                card(6.15, 3.28, 1.52, 0.82, title="PCA9548A", stripe=TEAL, fill="FFFFFF", title_size=10.2),
                card(7.83, 3.28, 1.52, 0.82, title="3 x 688RS", stripe=AMBER, fill="FFFFFF", title_size=10.2),
            ],
        ),
        slide(
            4,
            "content",
            "Stepper in Closed Loop",
            "",
            elements=[
                image_slot("sensor_housing_front", _thesis_png("sensor_housing_front.png"), 0.45, 1.0, 2.7, 2.15),
                image_slot("sensor_housing_back", _thesis_png("sensor_housing_back.png"), 3.2, 1.0, 2.7, 2.15),
                image_slot("sensor_housing_motor", _thesis_png("sensor_housing_fixed_to_motor.png"), 5.95, 1.0, 3.0, 2.15),
                flow(
                    0.55,
                    3.45,
                    8.9,
                    1.0,
                    [
                        {"label": "Open-loop\nstepper actuation", "fill": "D6E8F5"},
                        {"label": "Measured base\nangle retrofit", "fill": "E6F6F3"},
                        {"label": "Pendulum\nencoder", "fill": "FEF3C7"},
                        {"label": "Filtered derivatives\n-> 4 estimated states", "fill": "D6E8F5"},
                    ],
                ),
                text(
                    0.58,
                    4.75,
                    8.6,
                    0.22,
                    "Base angle sensing turns the motor axis into a measured state without changing the stepper into a torque servo.",
                    size=9,
                    color="64748B",
                    italic=True,
                    align="center",
                ),
            ],
        ),
        slide(
            5,
            "content",
            "Hardware Constraints and Limitations",
            "",
            elements=[
                card(
                    0.55,
                    1.1,
                    4.0,
                    1.55,
                    title="Base travel",
                    body="+/-80 deg travel means the arm has limited room to recover.",
                    stripe=BLUE,
                    title_size=12,
                    body_size=10.5,
                ),
                card(
                    5.0,
                    1.1,
                    4.0,
                    1.55,
                    title="Upright-only operation",
                    body="The controller is meant to catch and hold near upright, not swing up from large angles.",
                    stripe=TEAL,
                    title_size=12,
                    body_size=10.5,
                ),
                card(
                    0.55,
                    3.05,
                    4.0,
                    1.55,
                    title="Actuation reality",
                    body="The stepper accepts speed commands, not torque. Missed steps can happen under load.",
                    stripe=AMBER,
                    title_size=12,
                    body_size=10.5,
                ),
                card(
                    5.0,
                    3.05,
                    4.0,
                    1.55,
                    title="Sensor and timing reality",
                    body="Both encoders share one I2C bus, so timing delay, noise, and glitch rejection matter.",
                    stripe=RED,
                    title_size=12,
                    body_size=10.5,
                ),
            ],
        ),
        slide(
            6,
            "content",
            "Mathematical Modelling",
            "",
            elements=[
                text(0.58, 0.95, 3.4, 0.2, "Chosen coordinates", size=11, color=NAVY, bold=True),
                text(5.02, 0.95, 2.8, 0.2, "Kinematics", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s06_coords",
                    r"\begin{aligned} q_1 &= \theta,\quad q_2 = \alpha \\ \mathbf{r}_p &= L_r\mathbf{e}_r + l_p(\sin\alpha\,\mathbf{e}_t + \cos\alpha\,\mathbf{k}) \\ x &= L_r\cos\theta - l_p\sin\alpha\,\sin\theta \\ y &= L_r\sin\theta + l_p\sin\alpha\,\cos\theta \\ z &= l_p\cos\alpha \end{aligned}",
                    0.55,
                    1.2,
                    4.05,
                    2.95,
                    font_size=18,
                ),
                equation_slot(
                    "s06_vsq",
                    r"\begin{aligned} v^2&=(L_r^2+l_p^2\sin^2\alpha)\dot{\theta}^2+l_p^2\dot{\alpha}^2 \\ &\quad +2L_r l_p\cos\alpha\,\dot{\theta}\dot{\alpha} \end{aligned}",
                    4.95,
                    1.55,
                    4.15,
                    1.75,
                    font_size=19,
                ),
                text(
                    0.7,
                    4.55,
                    8.5,
                    0.22,
                    "First-principles model with the L-shaped pendulum geometry retained.",
                    size=10,
                    color="64748B",
                    italic=True,
                    align="center",
                ),
            ],
        ),
        slide(
            7,
            "content",
            "Mathematical Modelling",
            "",
            elements=[
                equation_slot(
                    "s07_ttrans",
                    r"T_{\mathrm{trans}}=\frac{1}{2}m_p v^2",
                    0.55,
                    1.12,
                    4.0,
                    1.1,
                    font_size=22,
                ),
                equation_slot(
                    "s07_trot",
                    r"T_{\mathrm{rot}}=\frac{1}{2}J_p(\dot{\alpha}^2+\sin^2\alpha\,\dot{\theta}^2)",
                    5.0,
                    1.12,
                    4.0,
                    1.1,
                    font_size=20,
                ),
                equation_slot(
                    "s07_tarm",
                    r"T_{\mathrm{arm}}=\frac{1}{2}\hat{J}_1\dot{\theta}^2",
                    0.55,
                    3.0,
                    4.0,
                    1.1,
                    font_size=22,
                ),
                equation_slot(
                    "s07_v",
                    r"\begin{aligned} V&=G\cos\alpha \\ &=m_p g l_p\cos\alpha \end{aligned}",
                    5.0,
                    2.92,
                    4.0,
                    1.28,
                    font_size=22,
                ),
            ],
        ),
        slide(
            8,
            "content",
            "Mathematical Modelling",
            "",
            elements=[
                text(0.58, 0.96, 3.2, 0.2, "Collected kinetic energy", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s08_total_t",
                    r"\begin{aligned} T&=\frac{1}{2}(\hat{J}_0+\hat{J}_2\sin^2\alpha)\dot{\theta}^2+\frac{1}{2}\hat{J}_2\dot{\alpha}^2 \\ &\quad +K\cos\alpha\,\dot{\theta}\dot{\alpha} \end{aligned}",
                    0.55,
                    1.1,
                    8.6,
                    1.55,
                    font_size=18,
                ),
                text(0.58, 2.95, 2.5, 0.2, "Grouped constants", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s08_constants",
                    r"\begin{aligned} \hat{J}_0&=\hat{J}_1+m_pL_r^2,\quad \hat{J}_2=J_p+m_pl_p^2 \\ K&=m_pL_rl_p,\quad G=m_pgl_p \end{aligned}",
                    0.55,
                    3.15,
                    4.55,
                    1.2,
                    font_size=19,
                ),
                text(5.45, 2.95, 1.5, 0.2, "Lagrangian", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s08_lagrangian",
                    r"\mathcal{L}=T-V",
                    5.35,
                    3.38,
                    3.2,
                    0.75,
                    font_size=24,
                ),
            ],
        ),
        slide(
            9,
            "content",
            "Final Nonlinear Equations of Motion",
            "",
            elements=[
                equation_slot(
                    "s09_theta_eom",
                    r"\begin{aligned} (\hat{J}_0+\hat{J}_2\sin^2\alpha)\ddot{\theta}+K\cos\alpha\,\ddot{\alpha} \\ \qquad +\hat{J}_2\sin(2\alpha)\dot{\theta}\dot{\alpha}-K\sin\alpha\,\dot{\alpha}^2=\tau \end{aligned}",
                    0.55,
                    1.0,
                    8.95,
                    1.15,
                    font_size=16,
                ),
                equation_slot(
                    "s09_alpha_eom",
                    r"\begin{aligned} K\cos\alpha\,\ddot{\theta}+\hat{J}_2\ddot{\alpha} \\ \qquad -\frac{1}{2}\hat{J}_2\sin(2\alpha)\dot{\theta}^2-G\sin\alpha=0 \end{aligned}",
                    0.55,
                    2.35,
                    8.95,
                    1.15,
                    font_size=16,
                ),
                text(2.75, 3.78, 4.5, 0.2, "Using commanded base acceleration as the input", size=10, color=NAVY, bold=True, align="center"),
                equation_slot(
                    "s09_accel_input",
                    r"\begin{aligned} \ddot{\alpha}&=A\sin\alpha+\sin\alpha\cos\alpha\,\dot{\theta}^2 \\ &\quad -B\cos\alpha\,\ddot{\theta} \end{aligned}",
                    1.15,
                    3.92,
                    7.4,
                    0.82,
                    font_size=22,
                ),
            ],
        ),
        slide(
            10,
            "content",
            "Linearisation About Upright",
            "",
            elements=[
                text(0.7, 1.05, 2.3, 0.2, "Small-angle terms", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s10_small_angle",
                    r"\begin{aligned} \sin\alpha&\approx\alpha,\quad \cos\alpha\approx 1 \\ \sin(2\alpha)&\approx 2\alpha \end{aligned}",
                    0.58,
                    1.22,
                    4.2,
                    0.92,
                    font_size=18,
                ),
                text(0.7, 2.28, 2.8, 0.2, "Reduced upright model", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s10_linear_model",
                    r"\ddot{\alpha}=100.8\,\alpha-1.952\,\ddot{\theta}",
                    0.58,
                    2.55,
                    7.6,
                    0.7,
                    font_size=24,
                ),
                text(0.7, 3.62, 3.2, 0.2, "Open-loop fall rate", size=11, color=NAVY, bold=True),
                equation_slot(
                    "s10_fall_rate",
                    r"J_{eq}=J_1-\frac{K^2}{J_0},\qquad \lambda=\pm\sqrt{\frac{G}{J_{eq}}}=\pm 12.47\ \mathrm{rad/s}",
                    0.58,
                    3.92,
                    8.35,
                    0.75,
                    font_size=20,
                ),
            ],
        ),
        slide(
            11,
            "content",
            "State-Space Representation",
            "",
            elements=[
                equation_slot(
                    "s11_state_def",
                    r"\begin{aligned} \mathbf{x}&=\begin{bmatrix}\theta & \alpha & \dot{\theta} & \dot{\alpha}\end{bmatrix}^{\mathsf T},\quad u\equiv\ddot{\theta} \\ \dot{x}_1&=x_3,\quad \dot{x}_2=x_4,\quad \dot{x}_3=u,\quad \dot{x}_4=A\,x_2-B\,u \end{aligned}",
                    0.55,
                    1.15,
                    4.15,
                    2.15,
                    font_size=18,
                ),
                equation_slot(
                    "s11_matrices",
                    r"\dot{\mathbf{x}}=\mathbf{A}\mathbf{x}+\mathbf{B}u,\qquad \mathbf{A}=\begin{bmatrix}0&0&1&0\\0&0&0&1\\0&0&0&0\\0&100.8&0&0\end{bmatrix},\qquad \mathbf{B}=\begin{bmatrix}0\\0\\1\\-1.952\end{bmatrix}",
                    4.95,
                    1.25,
                    4.2,
                    2.0,
                    font_size=17,
                ),
                text(
                    1.0,
                    4.45,
                    8.0,
                    0.22,
                    "This linearized upright form is the model used for controller design.",
                    size=10,
                    color="64748B",
                    italic=True,
                    align="center",
                ),
            ],
        ),
        slide(
            12,
            "content",
            "State Estimation Using a Discrete Filter",
            "",
            elements=[
                equation_slot(
                    "s12_hs",
                    r"H(s)=\frac{s}{1+s/\omega_c}",
                    0.58,
                    1.2,
                    3.8,
                    0.72,
                    font_size=26,
                ),
                equation_slot(
                    "s12_difference_eq",
                    r"\begin{aligned} \dot{x}_{\mathrm{filt}}[n]&=b_0\,\Delta x[n]-a_1\,\dot{x}_{\mathrm{filt}}[n-1] \\ b_0&=\frac{2\omega_c}{2+\omega_c T},\qquad a_1=\frac{\omega_c T-2}{\omega_c T+2} \end{aligned}",
                    0.55,
                    2.35,
                    4.35,
                    1.45,
                    font_size=18,
                ),
                card(
                    5.2,
                    1.2,
                    3.9,
                    1.2,
                    title="At 200 Hz",
                    body="omega_c = 450 rad/s\nb0 = 211.7647\na1 = 0.0588",
                    stripe=BLUE,
                    title_size=11.5,
                    body_size=10.2,
                ),
                bullets(
                    5.25,
                    2.95,
                    3.75,
                    1.4,
                    [
                        "Compute a wrap-safe angle difference each tick",
                        "Filter that increment to estimate the derivative",
                        "Clamp the result to reject impossible samples",
                    ],
                    size=10.5,
                    color="1B2A4A",
                ),
            ],
        ),
        slide(
            13,
            "content",
            "Controller Design: Linear Controller",
            "",
            elements=[
                equation_slot(
                    "s13_control_poly",
                    r"\begin{aligned} u&=-\mathbf{K}\mathbf{x} \\ (s^2+2\zeta_1\omega_1 s+\omega_1^2)(s^2+2\zeta_2\omega_2 s+\omega_2^2) \\ &=s^4+\tilde{a}_3s^3+\tilde{a}_2s^2+\tilde{a}_1s+\tilde{a}_0 \end{aligned}",
                    0.55,
                    1.0,
                    5.2,
                    1.55,
                    font_size=17,
                ),
                equation_slot(
                    "s13_gain_formulas",
                    r"\begin{aligned} k_{\theta}&=-\frac{\tilde{a}_0}{A},\qquad k_{\dot{\theta}}=-\frac{\tilde{a}_1}{A} \\ k_{\dot{\alpha}}&=\frac{k_{\dot{\theta}}-\tilde{a}_3}{B},\qquad k_{\alpha}=\frac{k_{\theta}-A-\tilde{a}_2}{B} \end{aligned}",
                    0.55,
                    2.7,
                    5.2,
                    1.35,
                    font_size=18,
                ),
                card(
                    6.1,
                    1.02,
                    3.2,
                    1.0,
                    title="Chosen poles",
                    body="omega1 = 15, zeta1 = 0.8\nomega2 = 1, zeta2 = 1\nomega1 is faster than the 12.47 rad/s fall rate",
                    stripe=BLUE,
                    title_size=11,
                    body_size=9.2,
                ),
                card(
                    6.1,
                    2.28,
                    3.2,
                    1.18,
                    title="Calculated gains",
                    body="K = [-2.232, -193.15, -4.702, -15.729]\nK_steps = [-9.92, -858.4, -20.9, -69.9]",
                    stripe=TEAL,
                    title_size=11,
                    body_size=9.1,
                ),
                card(
                    6.1,
                    3.72,
                    3.2,
                    0.98,
                    title="Deployed firmware gains",
                    body="31.5, 725.6, 28.3, 73.5",
                    stripe=AMBER,
                    title_size=11,
                    body_size=10.2,
                ),
            ],
        ),
        slide(
            14,
            "content",
            "Controller Design: Hybrid SMC",
            "",
            elements=[
                equation_slot(
                    "s14_hybrid_setup",
                    r"\begin{aligned} \ddot{\alpha}&=A\sin\alpha+\sin\alpha\cos\alpha\,\dot{\theta}^2-B\cos\alpha\,\ddot{\theta} \\ s&=\dot{\alpha}+\lambda\alpha \\ f&=A\sin\alpha+\sin\alpha\cos\alpha\,\dot{\theta}^2+\lambda\dot{\alpha} \end{aligned}",
                    0.55,
                    1.05,
                    5.45,
                    1.75,
                    font_size=17,
                ),
                equation_slot(
                    "s14_hybrid_law",
                    r"\ddot{\theta}=\frac{f+K\,\mathrm{sat}(s/\phi)}{B\cos\alpha}",
                    0.55,
                    3.2,
                    5.45,
                    0.72,
                    font_size=24,
                ),
                card(
                    6.35,
                    1.05,
                    3.0,
                    0.98,
                    title="Parameters",
                    body="lambda = 15\nK = 800\nphi = 50",
                    stripe=BLUE,
                    title_size=11,
                    body_size=10,
                ),
                card(
                    6.35,
                    2.35,
                    3.0,
                    1.35,
                    title="Chattering reduction",
                    body="sat(s/phi) replaces sgn(s).\nOutside phi the law behaves like sign control.\nInside phi it becomes smooth.",
                    stripe=TEAL,
                    title_size=11,
                    body_size=9.5,
                ),
            ],
        ),
        slide(
            15,
            "content",
            "Controller Design: SMC Controller",
            "",
            elements=[
                equation_slot(
                    "s15_full_surface",
                    r"s=\dot{\alpha}+\lambda_{\alpha}\alpha+k(\dot{\theta}_{\mathrm{err}}+\lambda_{\theta}\theta_{\mathrm{err}})",
                    0.55,
                    1.1,
                    5.75,
                    0.82,
                    font_size=23,
                ),
                equation_slot(
                    "s15_full_law",
                    r"\begin{aligned} \ddot{\theta}&=\frac{f_0+\lambda_{\alpha}\dot{\alpha}+k\lambda_{\theta}\dot{\theta}_{\mathrm{err}}-k\ddot{\theta}_{\mathrm{ref}}+K\,\mathrm{sat}(s/\phi)}{B\cos\alpha-k} \\ B\cos\alpha-k&>0 \end{aligned}",
                    0.55,
                    2.45,
                    5.85,
                    1.65,
                    font_size=17,
                ),
                card(
                    6.55,
                    1.05,
                    2.8,
                    1.2,
                    title="Parameters",
                    body="lambda_alpha = 15\nlambda_theta = 2\nk = 0.5\nK = 800, phi = 50",
                    stripe=BLUE,
                    title_size=11,
                    body_size=9.5,
                ),
                card(
                    6.55,
                    2.6,
                    2.8,
                    1.55,
                    title="Safety and smoothing",
                    body="B cos(alpha) - k is the control effectiveness term.\nIt must stay positive.\nThe same sat(s/phi) law is used to avoid chattering.",
                    stripe=RED,
                    title_size=11,
                    body_size=9.3,
                ),
            ],
        ),
        slide(
            16,
            "content",
            "Results: Linear Controller Standard Plots",
            "",
            elements=[
                image_slot(
                    "hold_LIN_timeseries",
                    _report_png("hold_LIN_rep_session_20260209_095631_trial0_timeseries.png"),
                    0.45,
                    1.0,
                    6.2,
                    4.0,
                ),
                image_slot(
                    "hold_LIN_phase_alpha",
                    _report_png("hold_LIN_rep_session_20260209_095631_trial0_phase_alpha.png"),
                    6.95,
                    1.1,
                    2.3,
                    1.65,
                ),
                image_slot(
                    "hold_LIN_effort_hist",
                    _report_png("hold_LIN_rep_session_20260209_095631_trial0_effort_hist.png"),
                    6.95,
                    3.1,
                    2.3,
                    1.65,
                ),
                text(0.6, 5.02, 0.35, 0.15, "H1", size=8, color="64748B"),
            ],
        ),
        slide(
            17,
            "content",
            "Results: Linear Controller Nudge Mode",
            "",
            elements=[
                image_slot(
                    "nudge_LIN_nudge",
                    _report_png("nudge_LIN_rep_session_20260209_114605_trial0_nudge.png"),
                    0.45,
                    1.0,
                    9.0,
                    4.15,
                ),
                text(0.6, 5.02, 0.35, 0.15, "N1", size=8, color="64748B"),
            ],
        ),
        slide(
            18,
            "content",
            "Results: Linear Controller Trapezoidal Positioning Mode",
            "",
            elements=[
                image_slot(
                    "nudge_LIN_timeseries",
                    _report_png("nudge_LIN_rep_session_20260209_114605_trial0_timeseries.png"),
                    0.45,
                    1.0,
                    6.35,
                    4.1,
                ),
                card(
                    7.05,
                    1.2,
                    2.35,
                    2.9,
                    title="N1 metrics",
                    body=_nudge_metric_body(),
                    stripe=BLUE,
                    title_size=11,
                    body_size=8.7,
                ),
                text(
                    7.15,
                    4.35,
                    2.1,
                    0.4,
                    "Same N1 run shown as trapezoidal-reference detail.",
                    size=9,
                    color="64748B",
                    italic=True,
                    align="left",
                ),
            ],
        ),
        slide(
            19,
            "content",
            "Results: Hybrid SMC Controller Plots",
            "",
            elements=[
                image_slot(
                    "hold_SMC_timeseries",
                    _report_png("hold_SMC_rep_session_20260209_095908_trial0_timeseries.png"),
                    0.45,
                    1.15,
                    4.35,
                    2.95,
                ),
                image_slot(
                    "tap_SMC_timeseries",
                    _report_png("tap_SMC_rep_session_20260209_114953_trial0_timeseries.png"),
                    5.2,
                    1.15,
                    4.35,
                    2.95,
                ),
                text(0.58, 4.95, 0.35, 0.15, "H2", size=8, color="64748B"),
                text(5.32, 4.95, 0.35, 0.15, "T2", size=8, color="64748B"),
            ],
        ),
        slide(
            20,
            "content",
            "Results: SMC Controller Plots",
            "",
            elements=[
                image_slot(
                    "hold_SMC4_timeseries",
                    _report_png("hold_SMC4_rep_session_20260209_121635_trial0_timeseries.png"),
                    0.45,
                    1.15,
                    4.35,
                    2.95,
                ),
                image_slot(
                    "tap_SMC4_timeseries",
                    _report_png("tap_SMC4_rep_session_20260209_121803_trial0_timeseries.png"),
                    5.2,
                    1.15,
                    4.35,
                    2.95,
                ),
                text(0.58, 4.95, 0.35, 0.15, "H3", size=8, color="64748B"),
                text(5.32, 4.95, 0.35, 0.15, "T3", size=8, color="64748B"),
            ],
        ),
        slide(
            21,
            "content",
            "Results: Comparison of All 3 Controller Plots",
            "",
            elements=[
                text(0.55, 1.02, 0.45, 0.18, "Hold", size=10, color=NAVY, bold=True),
                image_slot(
                    "hold_compare_timeseries",
                    _report_png("hold_compare_timeseries.png"),
                    0.95,
                    0.95,
                    8.4,
                    2.0,
                ),
                text(0.55, 3.18, 0.45, 0.18, "Tap", size=10, color=NAVY, bold=True),
                image_slot(
                    "tap_compare_timeseries",
                    _report_png("tap_compare_timeseries.png"),
                    0.95,
                    3.1,
                    8.4,
                    2.0,
                ),
            ],
        ),
    ]

    return {
        "meta": {
            "title": "Rotary Inverted Pendulum Thesis Deck",
            "deck_filename": "rotary_inverted_pendulum_thesis_deck.pptx",
            "base_deck_filename": "rotary_inverted_pendulum_thesis_base.pptx",
            "manifest_filename": "build_manifest.json",
        },
        "slides": slides,
    }
