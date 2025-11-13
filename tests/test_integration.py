"""
test_integration.py - Integration tests for the complete control system

These tests verify that the Tank and PIDController work together correctly
to achieve stable control of the process tank level.
"""

import pytest
import numpy as np
from tank import Tank
from pid_controller import PIDController


class TestControlSystemIntegration:
    """Integration tests for the complete control loop"""
    
    def test_controller_stabilizes_tank(self):
        """Test that PID controller brings tank to setpoint"""
        tank = Tank(initial_level=30.0, outflow_rate=0.5, noise_level=0.0)
        pid = PIDController(Kp=0.1, Ki=0.02, Kd=0.05, 
                           output_min=0.0, output_max=10.0)
        
        setpoint = 50.0
        delta_time = 1.0
        
        # Run simulation for 1200 steps (allow integral to accumulate)
        for _ in range(1200):
            current_level = tank.get_level()
            inflow = pid.calculate(setpoint, current_level, delta_time)
            tank.update(inflow, delta_time)
        
        # After 1200 seconds, should be close to setpoint
        final_level = tank.get_level()
        error = abs(setpoint - final_level)
        
        assert error < 2.0, f"Final error {error}% exceeds tolerance"
        
    def test_controller_handles_disturbances(self):
        """Test that controller maintains stability with noise"""
        tank = Tank(initial_level=30.0, outflow_rate=0.5, noise_level=0.05)
        pid = PIDController(Kp=0.1, Ki=0.02, Kd=0.05,
                           output_min=0.0, output_max=10.0)
        
        setpoint = 50.0
        delta_time = 1.0
        levels = []
        
        # Run with disturbances (longer simulation)
        for _ in range(1000):
            current_level = tank.get_level()
            levels.append(current_level)
            inflow = pid.calculate(setpoint, current_level, delta_time)
            tank.update(inflow, delta_time)
        
        # Check last 100 steps for stability
        final_levels = levels[-100:]
        mean_level = np.mean(final_levels)
        std_level = np.std(final_levels)
        
        # Should be near setpoint with low variance (realistic tolerance with noise)
        assert abs(mean_level - setpoint) < 3.0, f"Mean level {mean_level} too far from setpoint"
        assert std_level < 2.0, f"Std deviation {std_level} too high"
        
    def test_controller_output_respects_limits(self):
        """Test that controller output never exceeds physical limits"""
        tank = Tank(initial_level=10.0, outflow_rate=0.5, noise_level=0.0)
        pid = PIDController(Kp=0.1, Ki=0.02, Kd=0.05,
                           output_min=0.0, output_max=10.0)
        
        setpoint = 80.0
        delta_time = 1.0
        
        # Run simulation
        for _ in range(200):
            current_level = tank.get_level()
            inflow = pid.calculate(setpoint, current_level, delta_time)
            tank.update(inflow, delta_time)
            
            # Verify output is within limits
            assert inflow >= pid.output_min
            assert inflow <= pid.output_max
            
    def test_different_setpoints(self):
        """Test that controller works for various setpoints"""
        setpoints = [25.0, 50.0, 75.0]
        
        for sp in setpoints:
            tank = Tank(initial_level=30.0, outflow_rate=0.5, noise_level=0.0)
            pid = PIDController(Kp=0.1, Ki=0.02, Kd=0.05)
            
            delta_time = 1.0
            
            # Run for adequate time (1200 steps for integral action)
            for _ in range(1200):
                current_level = tank.get_level()
                inflow = pid.calculate(sp, current_level, delta_time)
                tank.update(inflow, delta_time)
            
            final_level = tank.get_level()
            error = abs(sp - final_level)
            
            assert error < 2.0, f"Failed for setpoint {sp}% with error {error}%"
            
    def test_no_runaway_behavior(self):
        """Test that system never exhibits runaway or unbounded oscillation"""
        tank = Tank(initial_level=30.0, outflow_rate=0.5, noise_level=0.05)
        pid = PIDController(Kp=0.1, Ki=0.02, Kd=0.05)
        
        setpoint = 50.0
        delta_time = 1.0
        levels = []
        
        # Run for extended time
        for _ in range(1000):
            current_level = tank.get_level()
            levels.append(current_level)
            inflow = pid.calculate(setpoint, current_level, delta_time)
            tank.update(inflow, delta_time)
        
        # Verify system remains bounded
        assert all(0 <= level <= 100 for level in levels)
        
        # Check that oscillations don't grow
        first_half_std = np.std(levels[400:500])
        second_half_std = np.std(levels[900:1000])
        
        # Later variance should not be significantly larger
        assert second_half_std <= first_half_std * 1.5
        
    def test_settling_occurs(self):
        """Test that system settles to steady state"""
        tank = Tank(initial_level=20.0, outflow_rate=0.5, noise_level=0.0)
        pid = PIDController(Kp=0.1, Ki=0.02, Kd=0.05)
        
        setpoint = 60.0
        delta_time = 1.0
        levels = []
        
        # Simulation time for settling (longer for integral accumulation)
        for _ in range(1200):
            current_level = tank.get_level()
            levels.append(current_level)
            inflow = pid.calculate(setpoint, current_level, delta_time)
            tank.update(inflow, delta_time)
        
        # Check if settled in last 100 steps (within 3% tolerance - realistic)
        final_levels = levels[-100:]
        tolerance = setpoint * 0.03
        
        settled = all(abs(level - setpoint) <= tolerance 
                     for level in final_levels)
        
        assert settled, "System did not settle within tolerance"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])