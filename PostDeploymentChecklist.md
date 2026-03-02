# FRC Robot Post-Deployment Checklist

## 1. Limelight Configuration
- [ ] Set Limelight team number in the web interface.
- [ ] Set a unique Limelight hostname (if using multiple Limelights).
- [ ] Set static IP address (recommended).
- [ ] Configure vision pipelines as needed.
- [ ] Test camera stream and target detection in the web interface.

## 2. SysId Characterization
- [ ] Run the SysId drive characterization routine from SmartDashboard.
- [ ] Download and analyze the SysId log file.
- [ ] Update `kDriveS`, `kDriveV`, and `kDriveA` in `Constants.SwerveConstants` with measured values.
- [ ] Rebuild and redeploy code after updating constants.

## 3. CAN IDs
- [ ] Verify all motor controllers (drive, angle, shooter, intake, etc.) have unique CAN IDs.
- [ ] Confirm all sensors (encoders, CAN-based sensors, etc.) have unique CAN IDs.
- [ ] Check that all CAN IDs in hardware match the IDs expected in your code/configuration.
- [ ] Use Phoenix Tuner, REV Hardware Client, or other vendor tools to scan and confirm CAN bus health and device IDs.
- [ ] Ensure no duplicate CAN IDs are present on the bus.
- [ ] Document all CAN IDs for future reference and troubleshooting.

## 4. Robot Code & Controls
- [ ] Verify all controller bindings (driver and secondary).
- [ ] Test all subsystems (swerve, shooter, intake, vision, etc.).
- [ ] Confirm SmartDashboard/Shuffleboard displays all key data.

## 5. Network & Communication
- [ ] Confirm robot and Limelight are on the correct network.
- [ ] Test NetworkTables communication (robot <-> Limelight <-> dashboard).

## 6. Safety & Hardware
- [ ] Check all wiring and connectors (power, CAN, Ethernet).
- [ ] Verify all breakers and fuses are secure.
- [ ] Confirm all sensors and actuators are functioning.

## 7. Final Testing
- [ ] Perform a full system test (teleop and autonomous).
- [ ] Check for error messages or warnings in Driver Station.
- [ ] Back up all configuration files and code.
