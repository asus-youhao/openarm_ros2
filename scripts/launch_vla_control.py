#!/usr/bin/env python3
"""
Launch Script for OpenArm VLA Control System
=============================================

This script launches all necessary components for the GR00T N1.5 VLA control system.

Usage:
    python3 launch_vla_control.py [options]

Examples:
    python3 launch_vla_control.py --build                    # Build and launch everything
    python3 launch_vla_control.py --no-hardware              # Launch without hardware (simulation)
    python3 launch_vla_control.py --help                     # Show help message

Requirements:
    - ROS2 Humble installed
    - Workspace built at ~/openarm_ros2
    - Hardware connected (CAN interface, Serial port)
"""

import argparse
import subprocess
import sys
import os
import signal
import time
from pathlib import Path
from typing import List, Optional


# ANSI color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color


def print_colored(message: str, color: str = Colors.NC):
    """Print a colored message to the console."""
    print(f"{color}{message}{Colors.NC}")


def print_banner():
    """Print the launch script banner."""
    print_colored("""
==============================================
   OpenArm VLA Control System Launcher
   GR00T N1.5 Integration
==============================================
""", Colors.BLUE)


class VLALauncher:
    """Launcher for the VLA control system components."""
    
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.script_dir = Path(__file__).parent.resolve()
        self.workspace_dir = self.script_dir.parent
        self.processes: List[subprocess.Popen] = []
        self.env: dict = os.environ.copy()  # Will be updated after sourcing workspace
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """Terminate all launched processes."""
        print_colored("\nShutting down...", Colors.YELLOW)
        for proc in self.processes:
            if proc.poll() is None:  # Process is still running
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        print_colored("All components stopped.", Colors.GREEN)
    
    def run_command(self, cmd: List[str], name: str, background: bool = True) -> Optional[subprocess.Popen]:
        """
        Run a command, optionally in the background.
        
        Args:
            cmd: Command and arguments to run
            name: Name of the component (for logging)
            background: Run in background if True
            
        Returns:
            subprocess.Popen object if background, None otherwise
        """
        print_colored(f"  Starting: {name}", Colors.CYAN)
        print_colored(f"    Command: {' '.join(cmd)}", Colors.NC)
        
        if background:
            proc = subprocess.Popen(
                cmd,
                env=self.env,  # Pass sourced ROS2 workspace environment
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid if os.name != 'nt' else None
            )
            self.processes.append(proc)
            print_colored(f"    PID: {proc.pid}", Colors.GREEN)
            return proc
        else:
            result = subprocess.run(cmd, capture_output=True, text=True, env=self.env)
            if result.returncode != 0:
                print_colored(f"    Error: {result.stderr}", Colors.RED)
                return None
            return None
    
    def build_workspace(self):
        """Build the ROS2 workspace."""
        print_colored("Building workspace...", Colors.YELLOW)
        
        cmd = [
            "colcon", "build",
            "--packages-select", "openarm_hardware"
        ]
        
        result = subprocess.run(
            cmd,
            cwd=self.workspace_dir,
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            print_colored(f"Build failed:\n{result.stderr}", Colors.RED)
            sys.exit(1)
        
        print_colored("Build complete!", Colors.GREEN)
    
    def source_workspace(self):
        """Source the workspace and update self.env."""
        install_dir = self.workspace_dir / "install" / "setup.bash"
        
        if not install_dir.exists():
            print_colored(f"Workspace not built. Run with --build first.", Colors.RED)
            sys.exit(1)
        
        # Get the environment after sourcing
        cmd = f"source {install_dir} && env"
        result = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True
        )
        
        env = os.environ.copy()
        for line in result.stdout.splitlines():
            if '=' in line:
                key, value = line.split('=', 1)
                env[key] = value
        
        self.env = env
    
    def launch_hardware(self):
        """Launch the robot hardware."""
        print_colored("[1/3] Launching robot hardware...", Colors.GREEN)
        
        cmd = [
            "ros2", "launch", "openarm_bringup", "openarm.bimanual.launch.py"
        ]
        
        self.run_command(cmd, "Robot Hardware", background=True)
        
        print_colored("  Waiting for hardware to initialize...", Colors.YELLOW)
        time.sleep(5)
    
    def launch_state_publisher(self):
        """Launch the GR00T state publisher."""
        print_colored("[2/3] Launching GR00T state publisher (50Hz)...", Colors.GREEN)
        
        script_path = self.script_dir / "gr00t_state_publisher.py"
        cmd = ["python3", str(script_path)]
        
        self.run_command(cmd, "GR00T State Publisher", background=True)
        
        time.sleep(1)
    
    def launch_action_controller(self):
        """Launch the action chunk controller."""
        print_colored("[3/3] Launching action chunk controller...", Colors.GREEN)
        
        script_path = self.script_dir / "action_chunk_controller.py"
        cmd = ["python3", str(script_path)]
        
        self.run_command(cmd, "Action Chunk Controller", background=True)
        
        time.sleep(1)
    
    def print_status(self):
        """Print the system status."""
        print_colored("""
==============================================
   System Status
==============================================""", Colors.BLUE)
        
        status_hardware = Colors.GREEN + "Running" if self.args.hardware else Colors.YELLOW + "Skipped"
        status_state = Colors.GREEN + "Running" if self.args.state else Colors.YELLOW + "Skipped"
        status_action = Colors.GREEN + "Running" if self.args.action else Colors.YELLOW + "Skipped"
        
        print(f"  Hardware:           {status_hardware}{Colors.NC}")
        print(f"  State Publisher:    {status_state}{Colors.NC}")
        print(f"  Action Controller:  {status_action}{Colors.NC}")
        
        print_colored("""
  Topics:
    /joint_states           - Raw joint states (100Hz)
    /gr00t/joint_states     - Filtered states for GR00T (50Hz)
    /gr00t/state_health     - Health statistics
    /action_chunk           - Action chunk input (from GR00T)
""", Colors.CYAN)
        
        print_colored("Press Ctrl+C to shutdown all components", Colors.YELLOW)
        print()
    
    def run(self):
        """Run the launcher."""
        print_banner()
        
        # Build if requested
        if self.args.build:
            self.build_workspace()
        
        # Source workspace environment
        self.source_workspace()
        
        # Launch components
        if self.args.hardware:
            self.launch_hardware()
        
        if self.args.state:
            self.launch_state_publisher()
        
        if self.args.action:
            self.launch_action_controller()
        
        # Print status
        self.print_status()
        
        # Wait for processes
        try:
            while True:
                # Check if any process has died
                for i, proc in enumerate(self.processes):
                    if proc.poll() is not None:
                        print_colored(f"  Process {proc.pid} exited with code {proc.returncode}", Colors.YELLOW)
                
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="OpenArm VLA Control System Launcher for GR00T N1.5 Integration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --build                    Build and launch everything
  %(prog)s --no-hardware              Launch without hardware (simulation)
  %(prog)s --no-state --no-action     Launch hardware only

Topics:
  /joint_states           Raw joint states (100Hz)
  /gr00t/joint_states     Filtered states for GR00T (50Hz)
  /gr00t/state_health     Health statistics
  /action_chunk           Action chunk input (from GR00T)
"""
    )
    
    # Component launch flags
    parser.add_argument(
        "--hardware", action="store_true", default=True,
        help="Launch robot hardware (default: True)"
    )
    parser.add_argument(
        "--no-hardware", dest="hardware", action="store_false",
        help="Skip hardware launch"
    )
    
    parser.add_argument(
        "--state", action="store_true", default=True,
        help="Launch state publisher (default: True)"
    )
    parser.add_argument(
        "--no-state", dest="state", action="store_false",
        help="Skip state publisher"
    )
    
    parser.add_argument(
        "--action", action="store_true", default=True,
        help="Launch action controller (default: True)"
    )
    parser.add_argument(
        "--no-action", dest="action", action="store_false",
        help="Skip action controller"
    )
    
    # Build flag
    parser.add_argument(
        "--build", "-b", action="store_true",
        help="Build workspace before launch"
    )
    
    # Verbosity
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable verbose output"
    )
    
    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_arguments()
    
    launcher = VLALauncher(args)
    launcher.run()


if __name__ == "__main__":
    main()