"""
test_pid.py - Unit tests for PIDController class

Tests the behavior of the PID controller, including:
- Initialization and parameter setting
- Proportional, Integral, and Derivative terms
- Output limiting and anti-windup
- Reset functionality
"""

import pytest
from pid_controller import PIDController


class TestPIDController:
    """Test suite for PIDController class"""
    
    def test_initialization(self):
        """Test that PID controller initializes with correct default values"""
        pid = PIDController()
        assert pid.Kp == 2.0
        assert pid.Ki == 0.1
        assert pid.Kd == 0.5
        assert pid.integral == 0.0
        assert pid.previous_error == 0.0
        assert pid.output_min == 0.0
        assert pid.output_max == 10.0
        
    def test_custom_initialization(self):
        """Test PID controller initialization with custom parameters"""
        pid = PIDController(Kp=3.0, Ki=0.2, Kd=1.0, 
                           output_min=-5.0, output_max=15.0)
        assert pid.Kp == 3.0
        assert pid.Ki == 0.2
        assert pid.Kd == 1.0
        assert pid.output_min == -5.0
        assert pid.output_max == 15.0
        
    def test_proportional_term(self):
        """Test that proportional term responds correctly to error"""
        pid = PIDController(Kp=2.0, Ki=0.0, Kd=0.0)
        
        # Error = 10, so P term = Kp * error = 2.0 * 10 = 20
        # But output is clamped to max (10.0)
        output = pid.calculate(setpoint=50.0, current_value=40.0, delta_time=1.0)
        assert output == 10.0  # Clamped to max
        
        # Smaller error that won't saturate
        pid.reset()
        output = pid.calculate(setpoint=50.0, current_value=48.0, delta_time=1.0)
        expected = 2.0 * 2.0  # Kp * error = 2.0 * 2.0 = 4.0
        assert abs(output - expected) < 0.1
        
    def test_integral_term_accumulation(self):
        """Test that integral term accumulates error over time"""
        pid = PIDController(Kp=0.0, Ki=0.1, Kd=0.0)
        
        # Run multiple steps with constant error
        error = 5.0
        delta_time = 1.0
        
        output1 = pid.calculate(setpoint=55.0, current_value=50.0, 
                               delta_time=delta_time)
        output2 = pid.calculate(setpoint=55.0, current_value=50.0, 
                               delta_time=delta_time)
        
        # Integral should accumulate, so output2 > output1
        assert output2 > output1
        
    def test_derivative_term(self):
        """Test that derivative term responds to rate of change"""
        pid = PIDController(Kp=0.0, Ki=0.0, Kd=1.0)
        
        # First call establishes baseline
        pid.calculate(setpoint=50.0, current_value=40.0, delta_time=1.0)
        
        # Second call with different error should produce derivative response
        output = pid.calculate(setpoint=50.0, current_value=45.0, delta_time=1.0)
        
        # Error changed from 10 to 5, derivative = (5-10)/1 = -5
        # D term = Kd * derivative = 1.0 * (-5) = -5
        # But output is clamped to min (0.0)
        assert output == 0.0
        
    def test_output_clamping_max(self):
        """Test that output is clamped to maximum value"""
        pid = PIDController(Kp=10.0, Ki=0.0, Kd=0.0, 
                           output_min=0.0, output_max=5.0)
        
        # Large error should saturate output at max
        output = pid.calculate(setpoint=100.0, current_value=50.0, delta_time=1.0)
        assert output == 5.0
        
    def test_output_clamping_min(self):
        """Test that output is clamped to minimum value"""
        pid = PIDController(Kp=10.0, Ki=0.0, Kd=0.0, 
                           output_min=0.0, output_max=100.0)
        
        # Negative error should clamp output at min
        output = pid.calculate(setpoint=30.0, current_value=50.0, delta_time=1.0)
        assert output == 0.0
        
    def test_reset(self):
        """Test that reset clears internal state"""
        pid = PIDController(Kp=1.0, Ki=1.0, Kd=1.0)
        
        # Run a few calculations to build up state
        pid.calculate(setpoint=50.0, current_value=40.0, delta_time=1.0)
        pid.calculate(setpoint=50.0, current_value=42.0, delta_time=1.0)
        
        # Verify state has changed
        assert pid.integral != 0.0
        assert pid.previous_error != 0.0
        
        # Reset should clear state
        pid.reset()
        assert pid.integral == 0.0
        assert pid.previous_error == 0.0
        
    def test_set_tuning(self):
        """Test that tuning parameters can be updated"""
        pid = PIDController(Kp=1.0, Ki=0.5, Kd=0.2)
        
        # Update all parameters
        pid.set_tuning(Kp=3.0, Ki=1.0, Kd=0.8)
        assert pid.Kp == 3.0
        assert pid.Ki == 1.0
        assert pid.Kd == 0.8
        
        # Update only one parameter
        pid.set_tuning(Ki=0.3)
        assert pid.Kp == 3.0  # Unchanged
        assert pid.Ki == 0.3  # Changed
        assert pid.Kd == 0.8  # Unchanged
        
    def test_get_tuning(self):
        """Test that get_tuning returns current parameters"""
        pid = PIDController(Kp=2.5, Ki=0.15, Kd=0.75)
        
        tuning = pid.get_tuning()
        assert tuning == (2.5, 0.15, 0.75)
        
    def test_set_output_limits(self):
        """Test that output limits can be updated"""
        pid = PIDController(output_min=0.0, output_max=10.0)
        
        pid.set_output_limits(-5.0, 20.0)
        assert pid.output_min == -5.0
        assert pid.output_max == 20.0
        
    def test_zero_delta_time(self):
        """Test that controller handles zero delta_time gracefully"""
        pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.5)
        
        # Should not crash with zero delta_time
        output = pid.calculate(setpoint=50.0, current_value=45.0, delta_time=0.0)
        
        # Should still return a valid output (P and I terms only)
        assert isinstance(output, float)
        assert output >= pid.output_min
        assert output <= pid.output_max
        
    def test_controller_reduces_error_over_time(self):
        """Integration test: PID should reduce error in a simple scenario"""
        pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.1)
        
        current_value = 30.0
        setpoint = 50.0
        delta_time = 1.0
        
        initial_error = abs(setpoint - current_value)
        
        # Simulate several control steps
        for _ in range(10):
            output = pid.calculate(setpoint, current_value, delta_time)
            # Simulate process response (simple first-order)
            current_value += output * 0.3
            
        final_error = abs(setpoint - current_value)
        
        # Error should decrease (controller working)
        assert final_error < initial_error


if __name__ == "__main__":
    pytest.main([__file__, "-v"])