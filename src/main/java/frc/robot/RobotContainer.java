// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drivechain.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final String controllerType = "JOYSTICK"; // XBOX or JOYSTICK
  private CommandJoystick m_driverController;
  private CommandXboxController m_xboxController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    if(controllerType.equals("XBOX")) {
      m_xboxController = new CommandXboxController(0);
      m_swerveDrive.setDefaultCommand(
          new TeleopDrive(
              m_swerveDrive,
              () -> -m_xboxController.getLeftY(),  // forward/back
              () -> -m_xboxController.getLeftX(),  // strafe
              () -> -m_xboxController.getRightX() // rotate
          ));
      m_xboxController.start().onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));

    } else {
      m_driverController = new CommandJoystick(1);
      m_swerveDrive.setDefaultCommand(
      new TeleopDrive(
            m_swerveDrive,
            // Axis 1 is usually Y (Forward/Back)
            () -> -m_driverController.getRawAxis(1), 
            // Axis 0 is usually X (Left/Right)
            () -> -m_driverController.getRawAxis(0), 
            // Axis 2 is often the Twist/Z-axis on flight sticks, 
            // or Axis 4 on some gamepads. Adjust as needed!
            () -> -m_driverController.getRawAxis(2) 
        ));
        m_driverController.trigger().onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    }


    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
