// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem_try;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.TunableNumber;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer
{
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                           "swerve"));
  private final ShooterSubsystem_try m_shooter = new ShooterSubsystem_try();
  private final CANFuelSubsystem m_fuel = new CANFuelSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final CommandXboxController m_driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_secondaryDriverXbox =
      new CommandXboxController(OperatorConstants.kSecondaryDriverControllerPort);

  TunableNumber m_angleP = new TunableNumber("Swerve/PID/ModuleAngle/P", SwerveParser.pidfPropertiesJson.angle.p);
  TunableNumber m_angleD = new TunableNumber("Swerve/PID/ModuleAngle/D", SwerveParser.pidfPropertiesJson.angle.d);
  TunableNumber m_driveP = new TunableNumber("Swerve/PID/ModuleDrive/P", SwerveParser.pidfPropertiesJson.drive.p);
  TunableNumber m_driveD = new TunableNumber("Swerve/PID/ModuleDrive/D", SwerveParser.pidfPropertiesJson.drive.d);

  private SendableChooser<Command> m_autoChooser = null;

  public RobotContainer() {
    // Field-oriented drive (default)
    NamedCommands.registerCommand("Intake", m_fuel.intake());
    NamedCommands.registerCommand("ShooterOn", m_shooter.shoot());
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveRobotRelativeCommand(
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getRightX() * -1, OperatorConstants.RIGHT_X_DEADBAND),
        () -> false
    );

    // Robot-oriented drive (left bumper)
    Command driveRobotOriented = m_drivebase.driveRobotRelativeCommand(
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverXbox.getRightX() * -1, OperatorConstants.RIGHT_X_DEADBAND),
        () -> false
    );
    addCommandToDashboard(driveRobotOriented);

    Command zeroGyro = m_drivebase.runOnce(() -> m_drivebase.zeroGyro()).withName("zeroGyro");
    addCommandToDashboard(zeroGyro);

    Command resetOdometrytoAllianceZero = m_drivebase.runOnce(
        () -> m_drivebase.resetOdometry(m_drivebase.invertIfFieldFlipped(new Pose2d(0, 0, new Rotation2d()))))
        .withName("resetOdometrytoAllianceZero");
    addCommandToDashboard(resetOdometrytoAllianceZero);

    Command addFakeVisionReading = m_drivebase.runOnce(() -> m_drivebase.addFakeVisionReading())
        .withName("addFakeVisionReading");
    addCommandToDashboard(addFakeVisionReading);

    Command testMotors = m_drivebase.run(() -> {
      SwerveDriveTest.powerAngleMotorsDutyCycle(m_drivebase.swerveDrive, m_driverXbox.getLeftX());
      SwerveDriveTest.powerDriveMotorsDutyCycle(m_drivebase.swerveDrive, m_driverXbox.getLeftY());
    }).withName("testMotors");
    addCommandToDashboard(testMotors);

    Command testAngleMotors = m_drivebase.run(() -> {
      SwerveDriveTest.angleModules(m_drivebase.swerveDrive, 
        // Divided by 2 because want to go -180 to 180, not full circle
        Rotation2d.fromRotations(m_driverXbox.getLeftX() / 2));
    }).withName("testAngleMotors");
    addCommandToDashboard(testAngleMotors);

    TunableNumber angle = new TunableNumber("testAngle", 90);
    Command testSetAngle = m_drivebase.runEnd(
        () -> SwerveDriveTest.angleModules(m_drivebase.swerveDrive, Rotation2d.fromDegrees(angle.get())),
        () -> SwerveDriveTest.angleModules(m_drivebase.swerveDrive, Rotation2d.fromDegrees(0)))
        .withName("testSetAngle");
    addCommandToDashboard(testSetAngle);

    /*
     * Command testDriveToPose = drivebase.runOnce(
     * () -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))).andThen(
     * drivebase.driveToPose(
     * new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(0))))
     * .withName("testDriveToPose");
     */
    Command testDriveToPose = m_drivebase.testDriveToPose(
      new Pose2d(new Translation2d(Meters.of(0), Meters.of(0.1)),
                                   new Rotation2d(Degrees.of(0))))
        /* .asProxy() */.withName("testDriveToPose");
    addCommandToDashboard(testDriveToPose);

    m_drivebase.setDefaultCommand(
        // testMotors);
        // driveRobotOriented);
        // driveFieldOrientedDirectAngle);
        // !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle :
        // driveFieldOrientedDirectAngleSim);
        driveFieldOrientedAnglularVelocity);
    m_driverXbox.leftBumper().whileTrue(driveRobotOriented);

    // SHOOTER: Hold A to spin up shooter
    m_driverXbox.a().whileTrue(m_shooter.shoot());

    // X = Intake
    m_driverXbox.x().whileTrue(m_fuel.intake());

    // Y = Launch
    m_driverXbox.y().whileTrue(m_fuel.launch());

    // B = SysId drive motor characterization
    // TODO: Remove after collecting kS, kV, kA values
    // m_driverXbox.b().whileTrue(
    //     m_drivebase.driveCharacterizationSysIdCommand());

    // RIGHT BUMPER = Auto-aim to target using Limelight
    m_driverXbox.rightBumper().whileTrue(
        m_drivebase.run(() -> {
            if (m_vision.hasTarget()) {
                double turnSpeed = -m_vision.getTargetX() * 0.03;  // P controller
                m_drivebase.swerveDrive.drive(
                    new Translation2d(0, 0),   // No forward/sideways movement
                    turnSpeed,                  // Rotation speed (rad/s)
                    true,                       // Field relative
                    false                       // Not open loop
                );
            }
        })
    );


    /*
    Command disableArm = m_arm.runOnce(
    addCommandToDashboard(disableArm);
*/


   

    SmartDashboard.putData(CommandScheduler.getInstance());

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Add SysId drive characterization command to SmartDashboard for manual start
    SmartDashboard.putData("SysId/Drive Characterization", m_drivebase.driveCharacterizationSysIdCommand());
  }

  private void addCommandToDashboard(Command cmd) {
    SmartDashboard.putData("cmd/" + cmd.getName(), cmd);
  }

  public void simulationInit() {
   
  }

  public void robotPeriodic() {


    SmartDashboard.putNumber("joystick/left-X", m_driverXbox.getLeftX());
    SmartDashboard.putNumber("joystick/left-Y", m_driverXbox.getLeftY());
    SmartDashboard.putNumber("joystick/right-X", m_driverXbox.getRightX());
    SmartDashboard.putNumber("joystick/right-Y", m_driverXbox.getRightY());

    SmartDashboard.putNumber("pose/x", m_drivebase.getPose().getX());
    SmartDashboard.putNumber("pose/y", m_drivebase.getPose().getY());
    SmartDashboard.putNumber("pose/z", m_drivebase.getPose().getRotation().getDegrees());

    SmartDashboard.putString("alliance", m_drivebase.isFieldFlipped() ? "RED" : "BLUE");

    // Display SysId characterization constants
    SmartDashboard.putNumber("SysId/kS", frc.robot.Constants.SwerveConstants.kDriveS);
    SmartDashboard.putNumber("SysId/kV", frc.robot.Constants.SwerveConstants.kDriveV);
    SmartDashboard.putNumber("SysId/kA", frc.robot.Constants.SwerveConstants.kDriveA);

    if (m_angleD.hasChanged() || m_angleP.hasChanged()) {
      for (SwerveModule module : m_drivebase.swerveDrive.getModules()) {
        module.getAngleMotor().configurePIDF(new PIDFConfig(m_angleP.get(), m_angleD.get()));
      }
    }
    if (m_driveD.hasChanged() || m_driveP.hasChanged()) {
      for (SwerveModule module : m_drivebase.swerveDrive.getModules()) {
        module.getDriveMotor().configurePIDF(new PIDFConfig(m_driveP.get(), m_driveD.get()));
      }
    }
  }

  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("test auto");
     return m_autoChooser.getSelected();
    // return drivebase.getAutonomousCommand("small path");

    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
        
    /*return new SequentialCommandGroup(
      m_drivebase.driveAtSpeed(5, 0, 0, false).withTimeout(1.2)
     // drivebase.driveAtSpeed(-5, 0, 0, false).withTimeout(0.5) );*/
 }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }
}