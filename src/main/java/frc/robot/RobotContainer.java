// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drivechain.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

/**
 * subsystems, commands, and trigger mappings ->
 */
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private CommandJoystick m_driverController;
  private CommandXboxController m_xboxController;
  private Intake intake;
  private Shooter shooter;

  public RobotContainer() {
    
    // Main init for subsystems
    setupIntake();
    setupShooter();
    configureControllerBindings();
    
    
  }
//Intake definition
  private void setupIntake() {
    intake = new Intake(IntakeConstants.MotorID.SPIN_MOTOR, IntakeConstants.MotorID.POSITION_MOTOR);
  }
//shooter definition
  private void setupShooter() {
    shooter = new Shooter(ShooterConstants.MotorID.PRIMARY_SHOOTER, ShooterConstants.MotorID.SECONDARY_SHOOTER, ShooterConstants.MotorID.ANGLE_MOTOR_ONE, ShooterConstants.MotorID.ANGLE_MOTOR_TWO, ShooterConstants.MotorID.SHOOTER_INTAKE);
  }


  private void configureControllerBindings() {

    // xBox Operator Below ->
      m_xboxController = new CommandXboxController(0);
      // When the D-Pad is pressed down:
      m_xboxController.povDown()
      .whileTrue(Commands.run(() -> intake.adjustPosition(-0.05))
      .finallyDo(() -> intake.stopPosition()));
      // When the D-Pad is pressed up:
      m_xboxController.povUp()
      .whileTrue(Commands.run(() -> intake.adjustPosition(0.05))
      .finallyDo(() -> intake.stopPosition()));
      
      // Will either turn the spin motor on or off (runs each time A button is pressed)
      m_xboxController.a().onTrue(new InstantCommand(() -> intake.configureSpin()));
      //96.7% sure that this will turn the primary and secondary shoot motors on(runs when b is pressed)
      m_xboxController.b().onTrue(new InstantCommand(() -> shooter.configureShoot()));


    // SwerveDrive JoyStick Below -> 
    m_driverController = new CommandJoystick(1); // Initialize our joystick (on DriverStation port 1)
      m_swerveDrive.setDefaultCommand(
      new TeleopDrive(
            m_swerveDrive,
            // Axis 0: X (Left & Right)
            () -> -m_driverController.getRawAxis(0), 
            // Axis 1: Y (Forward & Backward)
            () -> -m_driverController.getRawAxis(1), 
            // Axis 2: Z Rotation 
            () -> m_driverController.getRawAxis(2) 
        ));
      m_driverController.trigger().onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));  // Sets the gryo heading to point 0 -> The direction its pointing becomes forward essentially 
      // While the side button is pressed we allow rotations. Otherwise, the joystick will pick up too much Z rot input for basic motions (such as a linear forward motion)
      m_driverController.button(2).onTrue(new InstantCommand(() -> TeleopDrive.SetRotMultiplier(.05)))  
      .onFalse(new InstantCommand(() -> TeleopDrive.SetRotMultiplier(0)));
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
