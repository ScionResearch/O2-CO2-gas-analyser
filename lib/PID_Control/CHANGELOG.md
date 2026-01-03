# Changelog

All notable changes to the PID_Control library will be documented in this file.

## [1.0.0] - 2024-01-01

### Added
- **Safety Features**:
  - Stale data detection with configurable rate and timeout
  - Safe value limits for process value range checking
  - NaN detection for invalid sensor readings
  - Error state management with automatic controller disable
- **PID_Tune Library**:
  - Modular tuning interface for easy integration
  - Callback-based sensor reading
  - Configurable serial port support (Serial, Serial1, etc.)
  - Non-blocking design
- **Python Tuning Application**:
  - Real-time GUI with live plotting
  - Step response testing with automatic analysis
  - Min/max tracking
  - Data export to CSV
  - Debug message display
  - Keyboard shortcuts
- **Enhanced Examples**:
  - PID_Safety_Example demonstrating all safety features
  - PID_Tune_Example showing integration
  - Multiple companion sketches
- **Documentation**:
  - Comprehensive README files
  - API documentation
  - Usage examples and best practices

### Changed
- Improved error handling throughout
- Better serial communication robustness
- Enhanced anti-windup protection
- More efficient memory usage

### Fixed
- Fixed derivative kick on setpoint changes
- Improved numerical stability
- Better handling of edge cases

## [0.9.0] - 2023-12-01

### Added
- Initial PID control implementation
- Anti-windup protection
- Output limiting
- Configurable sample time
- Basic examples

### Features
- Proportional-Integral-Derivative control
- Derivative on measurement
- Enable/disable functionality
- Output limits
- Integral limits
