"""
tank.py - Process Tank Simulation (Plant)

This module simulates a process tank with variable inflow and outflow rates.
The tank represents the "plant" in a control system, responding to control
inputs (inflow rate) and exhibiting realistic disturbances (outflow variations).
"""

import numpy as np


class Tank:
    """
    Simulates a process tank with dynamic level control.
    
    The tank level changes based on the difference between inflow and outflow
    rates. Outflow includes random disturbances to simulate real-world process
    variations such as valve fluctuations, pump variations, or downstream
    demand changes.
    
    Attributes:
        level (float): Current liquid level in the tank (units: %)
        max_level (float): Maximum capacity of the tank (units: %)
        outflow_rate (float): Base outflow rate (units: %/s)
        noise_level (float): Magnitude of random disturbances (units: %/s)
    """
    
    def __init__(self, initial_level=30.0, max_level=100.0, 
                 outflow_rate=0.5, noise_level=0.05):
        """
        Initialize the tank with starting conditions.
        
        Args:
            initial_level (float): Starting level percentage (default: 30%)
            max_level (float): Maximum tank capacity (default: 100%)
            outflow_rate (float): Base outflow rate in %/s (default: 0.5)
            noise_level (float): Standard deviation of outflow noise (default: 0.05)
        """
        self.level = initial_level
        self.max_level = max_level
        self.outflow_rate = outflow_rate
        self.noise_level = noise_level
        
    def update(self, inflow_rate, delta_time):
        """
        Update the tank level based on inflow, outflow, and time step.
        
        The level changes according to:
            dLevel/dt = inflow_rate - (outflow_rate + noise)
        
        Outflow includes random Gaussian noise to simulate real disturbances.
        The tank level is constrained between 0 and max_level.
        
        Args:
            inflow_rate (float): Control input - rate of liquid entering (%/s)
            delta_time (float): Time step for simulation (seconds)
            
        Returns:
            float: Updated tank level after the time step
        """
        # Add random disturbance to outflow (simulates process noise)
        actual_outflow = self.outflow_rate + np.random.normal(0, self.noise_level)
        
        # Calculate net change in level
        net_flow = inflow_rate - actual_outflow
        
        # Update level
        self.level += net_flow * delta_time
        
        # Enforce physical constraints (level cannot be negative or exceed max)
        self.level = max(0, min(self.level, self.max_level))
        
        return self.level
    
    def get_level(self):
        """
        Get the current tank level.
        
        Returns:
            float: Current level in percentage
        """
        return self.level
    
    def reset(self, initial_level=30.0):
        """
        Reset the tank to a specified initial level.
        
        Args:
            initial_level (float): Level to reset to (default: 30%)
        """
        self.level = initial_level