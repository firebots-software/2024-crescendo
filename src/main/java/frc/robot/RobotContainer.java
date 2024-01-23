// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj2.command.Command;
>>>>>>> 72fb8899ac45ab31b52f977fe44e3d7ec39dba6a
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.ArmRotateCommand;
import frc.robot.Constants.OperatorConstants;
<<<<<<< HEAD
import frc.robot.commands.PeterCommands.RunShooterCommand;
=======
import frc.robot.commands.IntakeCommands.ArmRotateCommand;
import frc.robot.commands.IntakeCommands.RunShooterCommand;
>>>>>>> 72fb8899ac45ab31b52f977fe44e3d7ec39dba6a
import frc.robot.subsystems.PeterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandPS4Controller driverController;

  private final PeterSubsystem peter = PeterSubsystem.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverController = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.square().whileTrue(new RunShooterCommand(peter));
<<<<<<< HEAD
    // driverController.circle().whileTrue(new ArmRotateCommand(peter, )); //to do: figure out
    // encoder vals

=======
    driverController
        .circle()
        .whileTrue(new ArmRotateCommand(peter)); // to do: figure out encoder vals
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
>>>>>>> 72fb8899ac45ab31b52f977fe44e3d7ec39dba6a
  }
}
