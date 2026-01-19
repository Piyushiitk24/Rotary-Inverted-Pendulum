"""
Visualization and Animation for Furuta Pendulum Simulation
==========================================================
Provides:
- Static plots (matching hardware log analysis)
- Real-time animation of the pendulum
- Phase portraits and stability analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
from matplotlib.animation import FuncAnimation
from typing import Optional, List
import matplotlib.gridspec as gridspec

from simulator import SimulationResult


def plot_simulation_result(result: SimulationResult, 
                          title: str = "Furuta Pendulum Simulation",
                          save_path: Optional[str] = None):
    """
    Create a comprehensive plot of simulation results.
    Matches the style used for hardware log analysis.
    """
    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.25)
    
    t = result.t
    
    # 1. Pendulum angle
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, result.alpha, 'b-', linewidth=1, label='α (pendulum)')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.fill_between(t, -30, 30, alpha=0.1, color='green', label='Safe zone')
    ax1.set_ylabel('α (degrees)')
    ax1.set_xlabel('Time (s)')
    ax1.set_title('Pendulum Angle')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([-35, 35])
    
    # 2. Arm angle  
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, result.theta, 'r-', linewidth=1, label='θ (arm)')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.fill_between(t, -80, 80, alpha=0.1, color='green', label='Safe zone')
    ax2.set_ylabel('θ (degrees)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Arm Position')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-90, 90])
    
    # 3. Angular velocities
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, result.alpha_dot, 'b-', linewidth=1, alpha=0.7, label='α̇ (pendulum)')
    ax3.plot(t, result.theta_dot, 'r-', linewidth=1, alpha=0.7, label='θ̇ (arm)')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_ylabel('Angular velocity (deg/s)')
    ax3.set_xlabel('Time (s)')
    ax3.set_title('Angular Velocities')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # 4. Control commands
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, result.acc_cmd, 'g-', linewidth=1, label='Acceleration cmd')
    ax4.axhline(y=20000, color='r', linestyle='--', alpha=0.5, label='Limit')
    ax4.axhline(y=-20000, color='r', linestyle='--', alpha=0.5)
    ax4.set_ylabel('Acceleration (steps/s²)')
    ax4.set_xlabel('Time (s)')
    ax4.set_title('Control Command')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    
    # 5. Phase portrait (α vs α̇)
    ax5 = fig.add_subplot(gs[2, 0])
    # Color by time
    points = ax5.scatter(result.alpha, result.alpha_dot, 
                         c=t, cmap='viridis', s=2, alpha=0.7)
    ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax5.set_xlabel('α (degrees)')
    ax5.set_ylabel('α̇ (deg/s)')
    ax5.set_title('Phase Portrait (Pendulum)')
    plt.colorbar(points, ax=ax5, label='Time (s)')
    ax5.grid(True, alpha=0.3)
    
    # 6. Arm position command vs actual
    ax6 = fig.add_subplot(gs[2, 1])
    steps_per_deg = 4.444
    ax6.plot(t, result.pos_cmd / steps_per_deg, 'g-', linewidth=1, 
             alpha=0.7, label='Commanded θ')
    ax6.plot(t, result.theta, 'r--', linewidth=1, alpha=0.7, label='Actual θ')
    ax6.set_ylabel('Arm angle (degrees)')
    ax6.set_xlabel('Time (s)')
    ax6.set_title('Command vs Actual Arm Position')
    ax6.legend(loc='upper right')
    ax6.grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14, fontweight='bold')
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {save_path}")
    
    plt.tight_layout()
    return fig


def animate_pendulum(result: SimulationResult,
                    fps: int = 30,
                    speed: float = 1.0,
                    save_path: Optional[str] = None):
    """
    Create an animation of the pendulum motion.
    
    Args:
        result: Simulation result
        fps: Frames per second
        speed: Playback speed multiplier
        save_path: If provided, save animation to file
    """
    # Physical dimensions (scaled for visualization)
    L_arm = 0.19  # meters
    L_pend = 0.12  # meters
    scale = 2.0   # Scale factor for display
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.3, 0.4)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    
    # Create elements
    arm_line, = ax.plot([], [], 'b-', linewidth=4, label='Arm')
    pend_line, = ax.plot([], [], 'r-', linewidth=3, label='Pendulum')
    pivot, = ax.plot([], [], 'ko', markersize=10)
    tip, = ax.plot([], [], 'ro', markersize=8)
    
    # Text displays
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
    alpha_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, fontsize=10)
    theta_text = ax.text(0.02, 0.85, '', transform=ax.transAxes, fontsize=10)
    
    ax.legend(loc='upper right')
    ax.set_title('Furuta Pendulum Animation')
    
    # Downsample for smooth animation
    dt_anim = 1.0 / fps
    dt_sim = result.t[1] - result.t[0] if len(result.t) > 1 else 0.02
    skip = max(1, int(dt_anim / dt_sim / speed))
    
    indices = list(range(0, len(result.t), skip))
    
    def init():
        arm_line.set_data([], [])
        pend_line.set_data([], [])
        pivot.set_data([], [])
        tip.set_data([], [])
        time_text.set_text('')
        alpha_text.set_text('')
        theta_text.set_text('')
        return arm_line, pend_line, pivot, tip, time_text, alpha_text, theta_text
    
    def update(frame_idx):
        i = indices[frame_idx]
        t = result.t[i]
        theta = np.radians(result.theta[i])  # Arm angle
        alpha = np.radians(result.alpha[i])  # Pendulum angle (0 = up)
        
        # Arm endpoint (in horizontal plane, project to XZ)
        arm_x = L_arm * np.cos(theta)
        arm_y = 0  # Arm stays in horizontal plane
        
        # Pendulum: starts at arm tip, swings in vertical plane
        # When alpha=0, pendulum points up (Z direction)
        # When alpha>0, pendulum tilts in positive theta direction
        pend_x = arm_x + L_pend * np.sin(alpha) * np.cos(theta)
        pend_z = L_pend * np.cos(alpha)
        
        # Update lines
        arm_line.set_data([0, arm_x], [0, 0])
        pend_line.set_data([arm_x, pend_x], [0, pend_z])
        pivot.set_data([arm_x], [0])
        tip.set_data([pend_x], [pend_z])
        
        # Update text
        time_text.set_text(f't = {t:.2f} s')
        alpha_text.set_text(f'α = {result.alpha[i]:.1f}°')
        theta_text.set_text(f'θ = {result.theta[i]:.1f}°')
        
        return arm_line, pend_line, pivot, tip, time_text, alpha_text, theta_text
    
    anim = FuncAnimation(fig, update, frames=len(indices),
                        init_func=init, blit=True, interval=1000/fps)
    
    if save_path:
        print(f"Saving animation to {save_path}...")
        anim.save(save_path, writer='pillow', fps=fps)
        print("Done!")
    
    return anim, fig


def plot_gain_stability_map(results: dict,
                           kp_values: List[float],
                           kd_values: List[float],
                           save_path: Optional[str] = None):
    """
    Create a heatmap showing stability for different gain combinations.
    
    Args:
        results: Dict from run_gain_sweep()
        kp_values: List of KP values tested
        kd_values: List of KD values tested
    """
    # Create stability matrix
    stability = np.zeros((len(kd_values), len(kp_values)))
    
    for i, kd in enumerate(kd_values):
        for j, kp in enumerate(kp_values):
            info = results.get((kp, kd), {'success': False})
            if info['success']:
                stability[i, j] = 1.0
            else:
                # Normalize fall time (longer = more stable)
                fall_time = info.get('fall_time', 0)
                stability[i, j] = min(fall_time / 3.0, 0.9)  # Cap at 0.9
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    im = ax.imshow(stability, cmap='RdYlGn', aspect='auto', 
                   origin='lower', vmin=0, vmax=1)
    
    ax.set_xticks(range(len(kp_values)))
    ax.set_xticklabels([f'{kp:.0f}' for kp in kp_values], rotation=45)
    ax.set_yticks(range(len(kd_values)))
    ax.set_yticklabels([f'{kd:.1f}' for kd in kd_values])
    
    ax.set_xlabel('ACC_KP (steps/s²/deg)')
    ax.set_ylabel('ACC_KD (steps/s²/(deg/s))')
    ax.set_title('Controller Stability Map\n(Green = Stable, Red = Unstable)')
    
    plt.colorbar(im, ax=ax, label='Stability (1.0 = stable)')
    
    # Mark the current hardware values
    current_kp = 742
    current_kd = 54.6
    if current_kp in kp_values and current_kd in kd_values:
        kp_idx = kp_values.index(current_kp)
        kd_idx = kd_values.index(current_kd)
        ax.plot(kp_idx, kd_idx, 'b*', markersize=20, label='Current')
        ax.legend()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    plt.tight_layout()
    return fig


def compare_simulations(results: List[SimulationResult],
                       labels: List[str],
                       title: str = "Simulation Comparison"):
    """
    Compare multiple simulation results on the same plot.
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(results)))
    
    for result, label, color in zip(results, labels, colors):
        # Alpha
        axes[0, 0].plot(result.t, result.alpha, color=color, 
                       linewidth=1, label=label, alpha=0.8)
        # Theta
        axes[0, 1].plot(result.t, result.theta, color=color,
                       linewidth=1, label=label, alpha=0.8)
        # Alpha dot
        axes[1, 0].plot(result.t, result.alpha_dot, color=color,
                       linewidth=1, label=label, alpha=0.8)
        # Acc cmd
        axes[1, 1].plot(result.t, result.acc_cmd, color=color,
                       linewidth=1, label=label, alpha=0.8)
    
    axes[0, 0].set_ylabel('α (deg)')
    axes[0, 0].set_title('Pendulum Angle')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].set_ylabel('θ (deg)')
    axes[0, 1].set_title('Arm Position')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].set_ylabel('α̇ (deg/s)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_title('Pendulum Velocity')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].set_ylabel('Acc (steps/s²)')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_title('Control Command')
    axes[1, 1].grid(True, alpha=0.3)
    
    fig.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    return fig


def quick_plot(result: SimulationResult):
    """Quick and simple plot for interactive use."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    
    axes[0].plot(result.t, result.alpha, 'b-', label='α (pendulum)')
    axes[0].plot(result.t, result.theta, 'r-', label='θ (arm)')
    axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[0].set_ylabel('Angle (deg)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(result.t, result.acc_cmd, 'g-', label='Acc cmd')
    axes[1].axhline(20000, color='r', linestyle='--', alpha=0.3)
    axes[1].axhline(-20000, color='r', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Accel (steps/s²)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


if __name__ == "__main__":
    # Demo: run simulation and plot
    from simulator import FurutaSimulator, SimulationConfig
    
    config = SimulationConfig(
        duration_s=3.0,
        alpha_0_deg=5.0,
        engage_delay_s=0.05
    )
    
    sim = FurutaSimulator(config=config)
    result = sim.run()
    
    if result.success:
        fig = plot_simulation_result(result, "Demo Simulation")
        plt.show()
