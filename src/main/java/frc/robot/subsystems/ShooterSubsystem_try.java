package frc.robot.subsystems;

// in the libraries we need. You'll get red squiggles if something is missing.

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class ShooterSubsystem_try extends SubsystemBase {


    // TODO: Measure your shooter wheels with a ruler!
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // <-- CHANGE THIS
    
    // TODO: Check your gearbox. If direct drive (no gears), use 1.0
    // If you have a 2:1 reduction, use 2.0, etc.
    private static final double GEAR_RATIO = 1.0;  // <-- CHANGE THIS
    
    // TODO: These are placeholder values! Run SysId to find YOUR values.
    // kS = static friction (voltage to overcome friction and start moving)
    // kV = velocity gain (voltage per unit of velocity)
    // kA = acceleration gain (usually 0 for flywheels)
    private static final double kS = 0.191;     // <-- FIND WITH SYSID
    private static final double kV = 0.11858;   // <-- FIND WITH SYSID
    private static final double kA = 0.0;       // <-- FIND WITH SYSID
    
    // TODO: Start with a small P value, increase until responsive but not oscillating
    private static final double kP = 0.00936;   // <-- TUNE THIS
    private static final double kI = 0.0;       // Usually keep at 0 for flywheels
    private static final double kD = 0.0;       // Usually keep at 0 for flywheels
    
    // Safety limits
    private static final double CURRENT_LIMIT_AMPS = 40;  // Protects motors & breakers
    private static final double MAX_RPM = 6000;           // Software speed limit
    
    // Default shooting speed - tune this for your game!
    private static final double DEFAULT_SHOOTING_RPM = 5500;

    // Leader motor - the primary motor that receives commands
    // Gets CAN ID from Constants file so it's easy to change in one place
    private final SparkFlex leadMotor = new SparkFlex(
        ShooterConstants.LEAD_shooterMotorID,   // <-- Set this in Constants.java!
        MotorType.kBrushless                     // NEO Vortex is brushless
    );

    // Follower motor - mirrors the leader motor's commands
    // Two motors = more power for spinning the flywheel
    private final SparkFlex followMotor = new SparkFlex(
        ShooterConstants.FOLLOW_shooterMotorID,  // <-- Set this in Constants.java!
        MotorType.kBrushless
    );

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        // Follower setup: second motor mirrors leader
        // 'true' means inverted - so both motors spin the flywheel the same direction
        // (one motor is physically flipped, so we invert its direction)
        .withFollowers(Pair.of(followMotor, true))
        
        // Use closed-loop control (PID + feedforward) for accurate speed control
        // Alternative: OPEN_LOOP just sends voltage directly (less accurate)
        .withControlMode(ControlMode.CLOSED_LOOP)
        
        // PID controller - fine-tunes the speed
        // P = how aggressively to correct errors
        // I = accumulates error over time (usually 0 for flywheels)
        // D = dampens oscillation (usually 0 for flywheels)
        .withClosedLoopController(kP, kI, kD)
        
        // Feedforward - predicts voltage needed BEFORE PID kicks in
        // This does most of the work, PID just fine-tunes
        .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
        
        // Telemetry - sends data to dashboard for debugging
        // "ShooterMotor" is the prefix for all logged values
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        
        // Gear ratio - tells the controller about any gearing
        // 1:1 = direct drive, 2:1 = motor spins twice for each flywheel rotation
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_RATIO)))
        
        // Motor direction - false = default direction
        // If your flywheel spins backwards, change to true
        .withMotorInverted(false)
        
        // Coast mode - motor spins freely when stopped
        // Good for flywheels (maintains momentum)
        // Alternative: BRAKE mode stops quickly (bad for flywheels)
        .withIdleMode(MotorMode.COAST)
        
        // Current limit - protects motor from burning out
        // 40A is safe for NEO Vortex
        .withStatorCurrentLimit(Amps.of(CURRENT_LIMIT_AMPS));


    private final SmartMotorController smartController = new SparkWrapper(
        leadMotor,                    // The physical motor controller
        DCMotor.getNeoVortex(2),      // Simulation physics: 2 NEO Vortex motors
        motorConfig                   // All the configuration from above
    );


    private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(smartController)
        // Physical wheel size - needed for tangential velocity calculation
        .withDiameter(Inches.of(WHEEL_DIAMETER_INCHES))
        
        // Rotating mass - used for simulation accuracy
        .withMass(Pounds.of(1))
        
        // Software speed limits - prevents commanding unsafe speeds
        .withUpperSoftLimit(RPM.of(MAX_RPM))
        .withLowerSoftLimit(RPM.of(0))
        
        // Flywheel telemetry - separate from motor telemetry
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    // The flywheel object we'll use for most commands
    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    public ShooterSubsystem_try() {
        // All configuration is done in the field declarations above
        // This constructor is empty, but you could add initialization code here
    }

    /**
     * Sets the flywheel to a specific speed.
     * 
     * Example usage in RobotContainer:
     *   shooterButton.whileTrue(shooter.setSpeed(RPM.of(5500)));
     * 
     * @param speed Target speed (e.g., RPM.of(5500))
     * @return Command that maintains the speed until interrupted
     */
    public Command setSpeed(AngularVelocity speed) {
        return flywheel.setSpeed(speed);
    }

    /**
     * Spins up to default shooting speed.
     * Use this as a simple "shoot" button.
     * 
     * @return Command that spins flywheel to DEFAULT_SHOOTING_RPM
     */
    public Command shoot() {
        return setSpeed(RPM.of(DEFAULT_SHOOTING_RPM));
    }

    /**
     * Stops the flywheel (sets target to 0 RPM).
     * Note: Due to COAST mode, flywheel will slowly spin down, not stop instantly.
     * 
     * @return Command that stops the flywheel
     */
    public Command stop() {
        return setSpeed(RPM.of(0));
    }

    /**
     * Runs System Identification routine to find feedforward values.
     * 
     * HOW TO USE:
     * 1. Bind this to a button in RobotContainer
     * 2. Run the robot and press the button
     * 3. Watch the console/logs for kS, kV, kA values
     * 4. Update the constants at the top of this file
     * 
     * @return Command that runs the SysId routine
     */
    public Command runSysId() {
        // Ramps voltage from 0V to 12V at 3V/second for 7 seconds
        return flywheel.sysId(
            Volts.of(12),              // Max voltage
            Volts.of(3).per(Second),   // Ramp rate
            Seconds.of(7)              // Duration
        );
    }

    /**
     * Gets the current flywheel speed from the encoder.
     * 
     * @return Current angular velocity
     */
    public AngularVelocity getSpeed() {
        return flywheel.getSpeed();
    }

    /**
     * Checks if the flywheel is at the target speed (within tolerance).
     * Useful for autonomous routines - wait until ready before feeding game piece.
     * 
     * @param targetSpeed The speed we're trying to reach
     * @param tolerancePercent How close is "close enough" (0.05 = 5%)
     * @return true if within tolerance
     */
    public boolean isAtSpeed(AngularVelocity targetSpeed, double tolerancePercent) {
        double currentRPM = getSpeed().in(RPM);
        double targetRPM = targetSpeed.in(RPM);
        double tolerance = targetRPM * tolerancePercent;
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    /**
     * Checks if flywheel is ready to shoot (at default speed).
     * 
     * @return true if within 5% of DEFAULT_SHOOTING_RPM
     */
    public boolean isReadyToShoot() {
        return isAtSpeed(RPM.of(DEFAULT_SHOOTING_RPM), 0.05);
    }

    /**
     * Calculates the linear velocity at the edge of the flywheel.
     * This is the actual launch speed of game pieces!
     * 
     * Formula: v = ω × r (linear velocity = angular velocity × radius)
     * 
     * @return Launch speed in meters per second
     */
    public LinearVelocity getLaunchSpeed() {
        double radiusMeters = Inches.of(WHEEL_DIAMETER_INCHES / 2.0).in(Meters);
        double angularVelocityRadPerSec = getSpeed().in(RadiansPerSecond);
        return MetersPerSecond.of(angularVelocityRadPerSec * radiusMeters);
    }

    @Override
    public void periodic() {
        // Log motor velocities to AdvantageScope/Shuffleboard for debugging
        Logger.recordOutput("Shooter/LeaderVelocityRPM", leadMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FollowerVelocityRPM", followMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/IsReadyToShoot", isReadyToShoot());
        Logger.recordOutput("Shooter/LaunchSpeedMPS", getLaunchSpeed().in(MetersPerSecond));
    }

    @Override
    public void simulationPeriodic() {
        // Updates simulation physics - REQUIRED for simulation to work
        flywheel.simIterate();
    }
}

//                     NEXT STEPS FOR YOU                                 
// 1. Go to Constants.java and set your actual CAN IDs:                         
//       LEAD_shooterMotorID = ???  (check your wiring!)                         
//      FOLLOW_shooterMotorID = ???                                             
//                                                                               
//  2. Measure your wheel diameter and update WHEEL_DIAMETER_INCHES              
//                                                                               
//  3. Check your gear ratio and update GEAR_RATIO                               
//                                                                               
//  4. In RobotContainer.java, create an instance:                               
//       private final ShooterSubsystem_try shooter = new ShooterSubsystem_try();
//                                                                               
//  5. Bind buttons to commands:                                                 
//       driverController.a().whileTrue(shooter.shoot());                        
//       driverController.b().whileTrue(shooter.runSysId());                     
//                                                                               
//  6. Run SysId to find your feedforward values, then update kS, kV, kA         
//                                                                               
//  7. Tune kP until the flywheel responds quickly without oscillating          