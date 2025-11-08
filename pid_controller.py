"""
pid_controller.py - PID Controller Implementation

This module implements a Proportional-Integral-Derivative (PID) controller,
the most widely used control algorithm in industrial automation.

The PID controller calculates a control output based on three terms:
- Proportional (P): Responds to current error
- Integral (I): Responds to accumulated past errors
- Derivative (D): Responds to rate of error change
"""


class PIDController:
    """
    PID Controller for process control applications.
    
    The controller implements the standard PID algorithm:
        output = Kp*error + Ki*integral(error) + Kd*derivative(error)
    
    This is commonly used in industrial settings for:
    - Level control in tanks and vessels
    - Temperature control in reactors
    - Pressure control in pipelines
    - Flow control in process systems
    
    Attributes:
        Kp (float): Proportional gain
        Ki (float): Integral gain
        Kd (float): Derivative gain
        integral (float): Accumulated error over time
        previous_error (float): Error from previous time step
        output_min (float): Minimum controller output (anti-windup)
        output_max (float): Maximum controller output (anti-windup)
    """
    
    def __init__(self, Kp=2.0, Ki=0.1, Kd=0.5, 
                 output_min=0.0, output_max=10.0):
        """
        Initialize the PID controller with tuning parameters.
        
        Args:
            Kp (float): Proportional gain (default: 2.0)
            Ki (float): Integral gain (default: 0.1)
            Kd (float): Derivative gain (default: 0.5)
            output_min (float): Minimum output value (default: 0.0)
            output_max (float): Maximum output value (default: 10.0)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # Internal state variables
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Output limits for anti-windup
        self.output_min = output_min
        self.output_max = output_max
        
    def calculate(self, setpoint, current_value, delta_time):
        """
        Calculate the PID controller output.
        
        This method computes the control action based on the error between
        setpoint and current value. It updates internal state (integral and
        previous error) and returns the calculated output.
        
        Args:
            setpoint (float): Desired target value
            current_value (float): Current measured process value
            delta_time (float): Time since last calculation (seconds)
            
        Returns:
            float: Controller output (control action)
        """
        # Calculate error
        error = setpoint - current_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term (accumulated error over time)
        self.integral += error * delta_time
        I = self.Ki * self.integral
        
        # Derivative term (rate of change of error)
        if delta_time > 0:
            derivative = (error - self.previous_error) / delta_time
        else:
            derivative = 0.0
        D = self.Kd * derivative
        
        # Calculate total output
        output = P + I + D
        
        # Apply output limits (anti-windup)
        output = max(self.output_min, min(output, self.output_max))
        
        # Anti-windup: Limit integral term if output is saturated
        # This prevents integral windup when output hits limits
        if output == self.output_max or output == self.output_min:
            # Back-calculate the allowed integral to prevent windup
            allowed_I = output - P - D
            self.integral = allowed_I / self.Ki if self.Ki != 0 else 0.0
        
        # Store error for next derivative calculation
        self.previous_error = error
        
        return output
    
    def reset(self):
        """
        Reset the controller's internal state.
        
        This clears the integral accumulator and previous error,
        useful when restarting a control loop or changing setpoints.
        """
        self.integral = 0.0
        self.previous_error = 0.0
        
    def set_tuning(self, Kp=None, Ki=None, Kd=None):
        """
        Update PID tuning parameters during runtime.
        
        Args:
            Kp (float, optional): New proportional gain
            Ki (float, optional): New integral gain
            Kd (float, optional): New derivative gain
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
            
    def get_tuning(self):
        """
        Get current PID tuning parameters.
        
        Returns:
            tuple: (Kp, Ki, Kd) current tuning values
        """
        return (self.Kp, self.Ki, self.Kd)
    
    def set_output_limits(self, output_min, output_max):
        """
        Set the output limits for the controller.
        
        Args:
            output_min (float): Minimum output value
            output_max (float): Maximum output value
        """
        self.output_min = output_min
        self.output_max = output_max