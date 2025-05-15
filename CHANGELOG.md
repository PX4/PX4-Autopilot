# Changelog

## [Unreleased]

### Added
- Added new message definitions:
  - Added ExternalVehicleLocalPosition.msg for external INS position data
  - Added support for external INS position data structure
  - Added proper message versioning for external INS data

### Changed
- Modified existing message definitions:
  - Updated VehicleAttitude.msg for external INS support
  - Updated VehicleGlobalPosition.msg for external INS support
  - Updated VehicleLocalPosition.msg for external INS support

### Configuration
- Added development environment configuration:
  - Added .vscode/settings.json for development setup
  - Added proper IDE configuration for external INS development

### Technical Details

### Message System
- Added new external INS message types:
  - Created ExternalVehicleLocalPosition.msg
  - Added proper message structure for external INS data
  - Added versioning support for external INS messages
- Modified existing vehicle state messages:
  - Updated message structures for external INS compatibility
  - Added external INS specific fields
  - Maintained backward compatibility


### Changed
- Modified EKF2 to handle external data
- Updated parameter handling system
- Improved error reporting



## Technical Details

### EKF2 Changes
- Added external INS support
- Added fallback mechanisms
- Improved quaternion handling
- Added proper parameter handling
- Fixed array assignment issues

### VectorNav Driver
- Added binary message support
- Added multiple data output groups
- Added proper error handling
- Added timeout mechanisms
- Added parameter configuration

### System Integration
- Added proper message routing
- Added fallback mechanisms
- Added error handling
- Added timeout handling
- Added parameter management

### Message System
- Added new external INS message types
- Modified existing vehicle state messages
- Updated DDS topic configuration
- Added message versioning support
