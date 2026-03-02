// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.CANSparkFlexSendableAdapter;


import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// Notes for if we ever decide to SysId this swerve:
// - the math is here https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
// - the default feedforward controller in YAGSL is configured without Ks term,
//   which I find  strange, as Ks seems to be the most useful part
//   (see here https://github.com/BroncBotz3481/YAGSL-Lib/blob/2024/src/main/java/swervelib/parser/SwerveParser.java#L116
//    and SwerveMath.createDriveFeedforward that it calls)
//   - kV term is calculated as optimalVoltage / maxSpeed, which I do not understand,
//     should it not be a cruising voltage? "how much voltage is needed to hold (or "cruise") at a given constant velocity",
//     according to the wpilib article referenced above. But also probably does not
//     matter because we do not need to hold constant speed anywhere
//     - maxSpeed is provided in the construction code below,
//     - optimalVoltage is from PhysicalPropertiesJson.swervelib.parser.json, 12V by
//       default. Can be overridden in physicalproperties.json.
// - thus, we might want to set our own feedforward, by calling SwerveDrive.replaceSwerveModuleFeedforward,
//   but be careful about other functions that reset it, SwerveDrive.setMaximumSpeed at the moment is the only one
// - the current SwerveSubsystem.sysIdDriveMotorCommand is too high-level, it does all
//   four routines in a single call, but we migth want to do it by hand for better
//   safety control (so that the robot does not ram into anything)
public class SwerveSubsystem extends SubsystemBase {
  /**
   * Swerve drive object.
   */
  public final SwerveDrive swerveDrive;
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {

       // ! CHANGE
       // ! instead of calculating the conversion ratios, just input the gear ratios
       // ! of drive and pivot; yasgl will automatically calculate those values internally
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
        SwerveConstants.kMaxSpeed.in(MetersPerSecond));

    } catch (Exception e) {

      throw new RuntimeException(e);

    }

    swerveDrive.setHeadingCorrection(false); // ! You lowkey don't need this since yasgl has this off by default I think
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

    // Override YAGSL's default feedforward (which sets kS=0 and kA=0) with real
    // values obtained from SysId characterization.
    // TODO: Run driveCharacterizationSysIdCommand() on the real robot, then update
    //       the constants in Constants.SwerveConstants with the measured values.
    // after running sys id, check the log values from robot station to get the measured real life values for kS, kV and kA and then update them in constants. 
    swerveDrive.replaceSwerveModuleFeedforward(
        new SimpleMotorFeedforward(
            SwerveConstants.kDriveS,
            SwerveConstants.kDriveV,
            SwerveConstants.kDriveA));

    setupPathPlanner();

    // 0 to 3. front left -> front right -> back left -> back right (from SverveModule.moduleNumber documentation)
    addLiveWindowModule("FL", 0);
    addLiveWindowModule("FR", 1);
    addLiveWindowModule("BL", 2);
    addLiveWindowModule("BR", 3);

    // Example of how to change a single motor's PID config
    // swerveDrive.getModules()[idx].configuration.anglePIDF = new PIDFConfig(0.1, 0, 0);

    SmartDashboard.putData("swerve/subsystem", this);
  }

  private void addLiveWindowModule(String name, int idx) {
    SwerveModule module = swerveDrive.getModules()[idx];
    addChild(name + " angle motor", new CANSparkFlexSendableAdapter(
      (SparkFlex) module.getAngleMotor().getMotor()));
    addChild(name + " drive motor", new CANSparkFlexSendableAdapter(
      (SparkFlex) module.getDriveMotor().getMotor()));
    addChild(name + " encoder",
      (Sendable) module.getAbsoluteEncoder().getAbsoluteEncoder());
  }
  
  // ! CHANGE cleaned up method
  /**
   * Check if robot alliance is red
   * 
   * @return {@link Boolean}
   */
  public boolean isFieldFlipped() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {

      return alliance.get() == Alliance.Red;

    } else {

      return false; // Default to false if ds has no alliance

    }
  }

  // ! CHANGE cleaned up method
  /**
   * Invert {@link Pose2d} if field is flipped
   * 
   * @param pose {@link Pose2d} to modify
   * @return {@link Pose2d}
   */
  public Pose2d invertIfFieldFlipped(Pose2d pose) {
    if (isFieldFlipped()) {
      return FlippingUtil.flipFieldPose(pose);
    }
    return pose;
  }

  // ! CHANGE cleaned up method
  /** Setup PathPlanner */
  public void setupPathPlanner()
  {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity, // Robot-Relative ChassisSpeeds supplier
          this::setChassisSpeeds, // ChassiSpeeds consumer for following paths
          new PPHolonomicDriveController(
              PathPlannerConstants.kTranslationPID, // Translation PID constants
              PathPlannerConstants.kAnglePID // Rotation PID constants
          ),
          config, // Robot config from PathPlanner GUI
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          () -> isFieldFlipped(),
          this // Reference to this subsystem to set requirements
      );

      // See end of https://pathplanner.dev/pplib-follow-a-single-path.html#java-warmup
      FollowPathCommand.warmupCommand();

    } catch (Exception e) {

      throw new RuntimeException("Failed to load PathPlanner config from GUI settings", e);

    }
  }

  // ! If you aren't using photonvision, just remove this lmao
  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @param camera {@link PhotonCamera} to communicate with.
   * @return A {@link Command} which will run the alignment.
   */
  // public Command aimAtTarget(PhotonCamera camera)
  // {
  //   return run(() -> {
  //     PhotonPipelineResult result = camera.getLatestResult();
  //     if (result.hasTargets())
  //     {
  //       drive(getTargetSpeeds(0,
  //                             0,
  //                             Rotation2d.fromDegrees(result.getBestTarget()
  //                                                          .getYaw()))); // Not sure if this will work, more math may be required.
  //     }
  //   });
  // }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding {@link Command}
   */
  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    // ! These acceleration values are lowkey small, you could definitly go higher
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0, // SHOU:D WE CHANGE THIS BIGGER????
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0 // Goal end velocity in meters/sec
       // 0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  // ! CHANGE use a command factory instead of that weird command wrapper stuff
  /**
   * Use PathPlanner Path finding to go to a point on the field.
   * 
   * <p> This command will reset odometry to origin (0, 0) before starting for testing purposes
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding {@link Command}
   */
  public Command testDriveToPose(Pose2d pose) {
    return this.runOnce(() -> resetOdometry(new Pose2d()))
    // resetOdometry tell the robot to forget where it believes is its current position and instead believe that it is at the provided pose, which is the pose that it detects currently. 
      .andThen(driveToPose(pose));
  }

  // ! CHANGE cleanup
  // ! Also changed z to omega since the z axis is up/down
  // ! To my knowledge, frc robots can't fly :|
  /**
   * Drive based on robot relative velocity inputs
   * 
   * @param xVelocitySupplier {@link DoubleSupplier} providing desired x velocity
   * @param yVelocitySupplier {@link DoubleSupplier} providing desired y velocity
   * @param omegaSupplier {@link DoubleSupplier} providing desired angular rate
   * @return {@link Command}
   */
  public Command driveRobotRelative(
        DoubleSupplier xVelocitySupplier, 
        DoubleSupplier yVelocitySupplier, 
        DoubleSupplier omegaSupplier) {

    return run(() -> {
      swerveDrive.drive(
        new Translation2d(
          xVelocitySupplier.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          yVelocitySupplier.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
        omegaSupplier.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        false,
        false);
    });    

  }

  // ! CHANGE formatting
  // ! Also hanged the name since this can also be used outside of sim?
  // ! Also used Rotation2d instead of a double supplier which will probably be helpful later on
  // ! you can do like `Rotation2d.fromDegrees` to get it
  /**
   * Drive based on field relative velocities while facing an angle
   *
   * @param xVelocitySupplier {@link DoubleSupplier} providing desired x velocity
   * @param yVelocitySupplier {@link DoubleSupplier} providing desired y velocity
   * @param rotation {@link Rotation2d} representing the angle robot should face
   * @return {@link Command}
   */
  public Command driveWhileFacing(
      DoubleSupplier xVelocitySupplier, 
      DoubleSupplier yVelocitySupplier, 
      Rotation2d rotation) {
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      driveFieldOriented(
        swerveDrive.swerveController.getTargetSpeeds(
          xVelocitySupplier.getAsDouble(),
          yVelocitySupplier.getAsDouble(),
          rotation.getRadians(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return {@link Command}
   */
  public Command driveCharacterizationSysIdCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, 
            swerveDrive, 
            12,
            true),
        // ! I would lowkey remove the timeout and just let it run until it finishes
        3.0,
        5.0, 
        3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return {@link Command}
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
          new Config(),
          this, 
          swerveDrive),
        // ! same as above
        3.0, 
        5.0, 
        3.0);
  }

  // ! CHANGE cleanup
  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   * 
   * <p> Speeds are cubed for smoother control
   *
   * @param xVelocitySupplier     {@link DoubleSupplier} supplying desired x velocity
   * @param yVelocitySupplier     {@link DoubleSupplier} supplying desired y velocity
   * @param omegaSupplier         {@link DoubleSupplier} supplying desired angular velocity
   * @param slow                  {@link BooleanSupplier} that should return true when driver wants slow movement
   * @return {@link Command}
   */
  public Command driveRobotRelativeCommand(
      DoubleSupplier xVelocitySupplier, 
      DoubleSupplier yVelocitySupplier, 
      DoubleSupplier omegaSupplier,
      BooleanSupplier slow) {
    return run(() -> {

      int multiplier = isFieldFlipped() ? -1 : 1;
      if (slow.getAsBoolean()) {
        multiplier *= 0.75;
      }

      swerveDrive.drive(new Translation2d(
        multiplier * Math.pow(xVelocitySupplier.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity(),
        multiplier * Math.pow(yVelocitySupplier.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity()),
        multiplier * Math.pow(omegaSupplier.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false);
    });
  }

  // ! CHANGE cleanup
  /**
   * Drive robot at a set velocity
   * 
   * @param xVelocity x velocity
   * @param yVelocity y velocity
   * @param omega angular velocity
   * @param fieldRelative
   * @return {@link Command} 
   */
  public Command driveAtSpeed(double xVelocity, double yVelocity, double omega, boolean fieldRelative) {
    return run(() -> swerveDrive.drive(new Translation2d(xVelocity, yVelocity), omega, fieldRelative, false));
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic(){
    // ! CHANGE
    // Should be called every loop for syncronizing relative/absolute encoders
    swerveDrive.updateOdometry();
  }

  // ! CHANGE you should use this to pass limelight estimated positions for odometry
  // ! This should allow you to just use the position estimated by odometry for all
  // ! shooting calculations and such
  /**
   * Compensate odometry error from vision estimated positions
   * 
   * @param positionEstimate {@link Pose2d} representing estimated position
   * @param timestampSeconds {@link Double} representing the timestamp robot was estimated to be at position in seconds
   */
  public void addVisionEstimate(Pose2d positionEstimate, double timestampSeconds) {
    swerveDrive.addVisionMeasurement(positionEstimate, timestampSeconds);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        SwerveConstants.kMaxSpeed.in(MetersPerSecond));
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        SwerveConstants.kMaxSpeed.in(MetersPerSecond));
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
}