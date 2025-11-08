"""
main.py - Main Simulation Loop for Digital Twin PID Control System

This script simulates a closed-loop control system where a PID controller
regulates the level in a process tank. This represents a digital twin of
industrial process control applications such as:
- Distillation column level control
- Reactor vessel level management
- Storage tank level regulation
- Buffer tank control in refineries

The simulation demonstrates:
- Feedback control principles
- PID tuning and response characteristics
- Process dynamics and disturbances
- Steady-state convergence
"""

import numpy as np
from tank import Tank
from pid_controller import PIDController


def run_simulation(duration=1000, delta_time=1.0, setpoint=50.0,
                   save_data=True):
    """
    Run the digital twin simulation of a PID-controlled tank.
    
    Args:
        duration (float): Total simulation time in seconds (default: 1000s)
        delta_time (float): Time step for simulation in seconds (default: 1.0s)
        setpoint (float): Target tank level in % (default: 50%)
        save_data (bool): Whether to save data to CSV (default: True)
        
    Returns:
        dict: Dictionary containing simulation data arrays:
            - 'time': Time points
            - 'level': Tank level over time
            - 'setpoint': Setpoint values
            - 'inflow': Controller output (inflow rate)
            - 'error': Control error over time
    """
    
    print("=" * 60)
    print("Pure Controls - Digital Twin PID Controller Simulation")
    print("=" * 60)
    print(f"Simulation Parameters:")
    print(f"  Duration: {duration} seconds")
    print(f"  Time Step: {delta_time} seconds")
    print(f"  Setpoint: {setpoint}%")
    print("=" * 60)
    
    # Initialize the plant (tank) and controller
    tank = Tank(
        initial_level=30.0,      # Start below setpoint
        max_level=100.0,
        outflow_rate=0.5,        # Constant base outflow
        noise_level=0.05         # Small random disturbances
    )
    
    pid = PIDController(
        Kp=0.1,                  # Proportional gain
        Ki=0.02,                  # Integral gain (very strong to eliminate steady-state error)
        Kd=0.05,                  # Derivative gain (strong damping)
        output_min=0.0,          # Minimum inflow (valve closed)
        output_max=10.0          # Maximum inflow (valve fully open)
    )
    
    print(f"\nController Tuning:")
    print(f"  Kp = {pid.Kp}")
    print(f"  Ki = {pid.Ki}")
    print(f"  Kd = {pid.Kd}")
    print(f"\nTank Configuration:")
    print(f"  Initial Level: {tank.level}%")
    print(f"  Outflow Rate: {tank.outflow_rate}%/s")
    print(f"  Noise Level: {tank.noise_level}%/s")
    print("=" * 60)
    
    # Initialize data logging lists
    time_data = []
    level_data = []
    setpoint_data = []
    inflow_data = []
    error_data = []
    
    # Calculate number of simulation steps
    num_steps = int(duration / delta_time)
    
    # Simulation loop
    print(f"\nRunning simulation... ({num_steps} steps)")
    
    for step in range(num_steps):
        current_time = step * delta_time
        
        # Get current tank level
        current_level = tank.get_level()
        
        # Calculate control action using PID
        inflow_rate = pid.calculate(
            setpoint=setpoint,
            current_value=current_level,
            delta_time=delta_time
        )
        
        # Apply control action to the plant (tank)
        new_level = tank.update(
            inflow_rate=inflow_rate,
            delta_time=delta_time
        )
        
        # Log data
        time_data.append(current_time)
        level_data.append(current_level)
        setpoint_data.append(setpoint)
        inflow_data.append(inflow_rate)
        error_data.append(setpoint - current_level)
        
        # Print progress every 100 steps
        if step % 100 == 0:
            print(f"  t={current_time:6.0f}s | Level={current_level:5.2f}% | "
                  f"Inflow={inflow_rate:5.3f}%/s | Error={setpoint-current_level:+6.2f}%")
    
    print("\nSimulation complete!")
    
    # Calculate performance metrics
    final_level = level_data[-1]
    steady_state_error = abs(setpoint - final_level)
    settling_time = calculate_settling_time(time_data, level_data, setpoint)
    overshoot = calculate_overshoot(level_data, setpoint)
    
    print("\n" + "=" * 60)
    print("Performance Metrics:")
    print("=" * 60)
    print(f"  Final Level: {final_level:.2f}%")
    print(f"  Steady-State Error: {steady_state_error:.2f}%")
    print(f"  Settling Time (2%): {settling_time:.1f}s")
    print(f"  Maximum Overshoot: {overshoot:.2f}%")
    print("=" * 60)
    
    # Prepare data dictionary
    simulation_data = {
        'time': np.array(time_data),
        'level': np.array(level_data),
        'setpoint': np.array(setpoint_data),
        'inflow': np.array(inflow_data),
        'error': np.array(error_data)
    }
    
    # Save to CSV if requested
    if save_data:
        save_to_csv(simulation_data)
    
    return simulation_data


def calculate_settling_time(time_data, level_data, setpoint, tolerance=0.02):
    """
    Calculate the settling time - time to reach and stay within tolerance band.
    
    Args:
        time_data (list): Time points
        level_data (list): Tank level values
        setpoint (float): Target setpoint
        tolerance (float): Tolerance band as fraction (default: 2%)
        
    Returns:
        float: Settling time in seconds
    """
    tolerance_band = setpoint * tolerance
    
    for i in range(len(level_data) - 50):  # Check if stays settled for 50 steps
        if all(abs(level_data[j] - setpoint) <= tolerance_band 
               for j in range(i, min(i + 50, len(level_data)))):
            return time_data[i]
    
    return time_data[-1]  # Never settled


def calculate_overshoot(level_data, setpoint):
    """
    Calculate the maximum overshoot percentage.
    
    Args:
        level_data (list): Tank level values
        setpoint (float): Target setpoint
        
    Returns:
        float: Maximum overshoot in percentage points
    """
    max_level = max(level_data)
    overshoot = max(0, max_level - setpoint)
    return overshoot


def save_to_csv(data, filename='simulation_data.csv'):
    """
    Save simulation data to CSV file using pandas.
    
    Args:
        data (dict): Dictionary containing simulation arrays
        filename (str): Output filename (default: 'simulation_data.csv')
    """
    try:
        import pandas as pd
        
        df = pd.DataFrame(data)
        df.to_csv(filename, index=False)
        print(f"\nData saved to '{filename}'")
        
    except ImportError:
        print("\nWarning: pandas not installed. Skipping CSV export.")
        print("Install with: pip install pandas")


def main():
    """
    Main entry point for the simulation.
    """
    # Run the simulation
    data = run_simulation(
        duration=1000,      # 1000 second simulation
        delta_time=1.0,     # 1 second time steps
        setpoint=50.0,      # Target level of 50%
        save_data=True      # Save results to CSV
    )
    
    print("\nSimulation data ready for visualization.")
    print("Run 'python visualize.py' to generate plots.")
    
    return data


if __name__ == "__main__":
    main()