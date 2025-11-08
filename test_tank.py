"""
test_tank.py - Unit tests for Tank class

Tests the behavior of the process tank simulation, including:
- Initialization
- Level updates with inflow/outflow
- Physical constraints (min/max levels)
- Reset functionality
"""

import pytest
from tank import Tank


class TestTank:
    """Test suite for Tank class"""
    
    def test_initialization(self):
        """Test that tank initializes with correct default values"""
        tank = Tank()
        assert tank.level == 30.0
        assert tank.max_level == 100.0
        assert tank.outflow_rate == 0.5
        assert tank.noise_level == 0.05
        
    def test_custom_initialization(self):
        """Test tank initialization with custom parameters"""
        tank = Tank(initial_level=50.0, max_level=150.0, 
                   outflow_rate=1.0, noise_level=0.1)
        assert tank.level == 50.0
        assert tank.max_level == 150.0
        assert tank.outflow_rate == 1.0
        assert tank.noise_level == 0.1
        
    def test_level_increases_with_positive_net_flow(self):
        """Test that level increases when inflow > outflow"""
        tank = Tank(initial_level=30.0, outflow_rate=0.5, noise_level=0.0)
        initial_level = tank.level
        
        # Inflow > outflow, so level should increase
        tank.update(inflow_rate=1.0, delta_time=1.0)
        
        assert tank.level > initial_level
        
    def test_level_decreases_with_negative_net_flow(self):
        """Test that level decreases when inflow < outflow"""
        tank = Tank(initial_level=50.0, outflow_rate=0.5, noise_level=0.0)
        initial_level = tank.level
        
        # Inflow < outflow, so level should decrease
        tank.update(inflow_rate=0.2, delta_time=1.0)
        
        assert tank.level < initial_level
        
    def test_level_remains_stable_with_balanced_flow(self):
        """Test that level stays approximately constant when inflow = outflow"""
        tank = Tank(initial_level=50.0, outflow_rate=0.5, noise_level=0.0)
        initial_level = tank.level
        
        # Balanced flow (no noise)
        tank.update(inflow_rate=0.5, delta_time=1.0)
        
        # Should be very close to initial level
        assert abs(tank.level - initial_level) < 0.01
        
    def test_level_cannot_exceed_max(self):
        """Test that tank level is capped at max_level"""
        tank = Tank(initial_level=95.0, max_level=100.0, 
                   outflow_rate=0.1, noise_level=0.0)
        
        # Large inflow should not exceed max_level
        tank.update(inflow_rate=10.0, delta_time=2.0)
        
        assert tank.level <= tank.max_level
        assert tank.level == 100.0
        
    def test_level_cannot_go_negative(self):
        """Test that tank level is constrained to be non-negative"""
        tank = Tank(initial_level=5.0, outflow_rate=2.0, noise_level=0.0)
        
        # Large outflow should not make level negative
        tank.update(inflow_rate=0.0, delta_time=5.0)
        
        assert tank.level >= 0.0
        assert tank.level == 0.0
        
    def test_get_level(self):
        """Test that get_level returns current level"""
        tank = Tank(initial_level=42.0)
        assert tank.get_level() == 42.0
        
    def test_reset(self):
        """Test that reset restores tank to initial state"""
        tank = Tank(initial_level=30.0)
        
        # Change the level
        tank.update(inflow_rate=2.0, delta_time=10.0)
        assert tank.level != 30.0
        
        # Reset should restore to 30.0
        tank.reset(initial_level=30.0)
        assert tank.level == 30.0
        
    def test_reset_to_custom_level(self):
        """Test that reset can set a custom initial level"""
        tank = Tank(initial_level=30.0)
        tank.update(inflow_rate=1.0, delta_time=5.0)
        
        # Reset to a different level
        tank.reset(initial_level=75.0)
        assert tank.level == 75.0
        
    def test_update_returns_new_level(self):
        """Test that update method returns the new level"""
        tank = Tank(initial_level=40.0, outflow_rate=0.5, noise_level=0.0)
        
        new_level = tank.update(inflow_rate=1.0, delta_time=1.0)
        
        assert new_level == tank.level
        assert isinstance(new_level, float)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])