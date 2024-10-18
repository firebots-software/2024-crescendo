// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

import org.ejml.data.FGrowArray;
import org.photonvision.EstimatedRobotPose;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();
  // private LightsSubsystem lightsSubsystem = LightsSubsystem.getInstance();

  private RobotContainer m_robotContainer;
  private static Matrix<N3, N1> visionMatrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1());

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture(0);
    visionMatrix.set(0, 0, 0.01);
    visionMatrix.set(1, 0, 0.03d);
    visionMatrix.set(2, 0, 100d);

    NetworkTableInstance.getDefault().setServerTeam(3501);
    DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureDs(true).withLogExtras(true));

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    absoluteInit();
    DataLogManager.start();
  }

  /**
   * 2 This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    m_robotContainer.doTelemetry();
    // Optional<EstimatedRobotPose> frontRobotPose =
    //     frontVision.getMultiTagPose3d(driveTrain.getState().Pose);
    // if (frontVision.hasTarget(frontVision.getPipeline()) && frontRobotPose.isPresent()) {
    //   AprilTagFieldLayout apr = PhotonVision.aprilTagFieldLayout;
    //   double distToAprilTag =
    //       apr.getTagPose(frontVision.getPipeline().getBestTarget().getFiducialId())
    //           .get()
    //           .getTranslation()
    //           .getDistance(
    //               new Translation3d(
    //                   driveTrain.getState().Pose.getX(), driveTrain.getState().Pose.getY(), 0.0));

    //   double xKalman = 0.01 * Math.pow(1.15, distToAprilTag);

    //   double yKalman = 0.01 * Math.pow(1.4, distToAprilTag);

    //   visionMatrix.set(0, 0, xKalman);
    //   visionMatrix.set(1, 0, yKalman);

    //   driveTrain.addVisionMeasurement(
    //       frontRobotPose.get().estimatedPose.toPose2d(),
    //       Timer.getFPGATimestamp() - 0.02,
    //       visionMatrix);
    // }

    // if (frontRobotPose.isPresent()) {
    // frontVision.get
    // AprilTagFieldLayout apr = PhotonVision.aprilTagFieldLayout;
    // double distToAprilTag =
    //     apr.getTagPose(frontVision.getPipeline().getBestTarget().getFiducialId())
    //         .get()
    //         .getTranslation()
    //         .getDistance(
    //             new Translation3d(
    //                 driveTrain.getState().Pose.getX(), driveTrain.getState().Pose.getY(),
    // 0.0));

    // double xKalman = 0.02 * Math.pow(1.15, distToAprilTag);

    // double yKalman = 0.02 * Math.pow(1.4, distToAprilTag);

    // visionMatrix.set(0, 0, xKalman);
    //   // visionMatrix.set(1, 0, yKalman);
    //   driveTrain.addVisionMeasurement(
    //       frontRobotPose.get().estimatedPose.toPose2d(),
    //       frontRobotPose.get().timestampSeconds - 0.02,
    //       visionMatrix);
    // }

    CommandScheduler.getInstance().run();
    // m_robotContainer.doTelemetry();
    // if (vision.hasTarget(vision.getPipeline())) {
    //   driveTrain.addVisionMeasurement(
    //       vision.getRobotPose2d(), Timer.getFPGATimestamp(), visionMatrix);
    // }

    // CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    SignalLogger.stop();
    armSubsystem.setEnable(false);
  }

  @Override
  public void disabledPeriodic() {

    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    armSubsystem.setTargetDegrees(armSubsystem.getCorrectedDegrees() + 15d);
    armSubsystem.setEnable(true);
    absoluteInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    absoluteInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    armSubsystem.setTargetDegrees(Constants.Arm.DEFAULT_ARM_ANGLE);
    armSubsystem.setEnable(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {  
    SmartDashboard.putBoolean(
        "joystickB right trigger", m_robotContainer.joystickB.rightTrigger(0.5).getAsBoolean());
    
    // SmartDashboard.putNumber("joystickB right trigger value",
    // m_robotContainer.joystickB.rightTrigger());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    absoluteInit();
    CommandScheduler.getInstance().cancelAll();
    armSubsystem.setEnable(true);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    absoluteInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void absoluteInit() {
    RobotContainer.setAlliance();
    SignalLogger.setPath("/home/lvuser/logs/");
    SignalLogger.start();
  }
}
