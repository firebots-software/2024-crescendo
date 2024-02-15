// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SysID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // CommandXboxController m_joystick = new CommandXboxController(0);
  private final CommandPS4Controller joystick =
      new CommandPS4Controller(Constants.OI.JOYSTICK_PORT);
  SysID m_mechanism = new SysID();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* Default command is duty cycle control with the left up/down stick */
    m_mechanism.setDefaultCommand(m_mechanism.joystickDriveCommand(joystick::getLeftY));

    /**
     * Joystick Y = quasistatic forward Joystick B = dynamic forward Joystick A = quasistatic
     * reverse Joystick X = dyanmic reverse
     */
    joystick.circle().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    joystick.square().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick.cross().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.triangle().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    /* Manually stop logging with left bumper after we're done with the tests */
    /* This isn't necessary, but is convenient to reduce the size of the hoot file */
    joystick.L1().onTrue(new RunCommand(SignalLogger::stop));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
