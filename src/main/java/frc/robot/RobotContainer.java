// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import swervelib.simulation.ironmaple.simulation.opponentsim.SmartOpponentConfig.ModuleConfig;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.drivechain.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Limit;
import frc.robot.subsystems.climb.Max;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import com.ctre.phoenix6.hardware.TalonFX;



/**
 * subsystems, commands, and trigger mappings ->
 */
public class RobotContainer {
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private CommandJoystick m_driverController;
  private CommandXboxController m_xboxController;
  private Intake intake;
  private Shooter shooter;
  private Feeder feeder;
  private Climb climb;
  private boolean goingUp = false;
  // private final RobotConfig robotConfig;

  public RobotContainer() {

    
    
    // Main init for subsystems
    setupIntake();
    setupShooter();
    setupFeeder();
    configureControllerBindings();

    setupClimb();

    registerCommands();
  }


  
//Intake definition
  private void setupIntake() {
    intake = new Intake(IntakeConstants.MotorID.SPIN_MOTOR, IntakeConstants.MotorID.POSITION_MOTOR);
  }
//shooter definition
  private void setupShooter() {
    shooter = new Shooter(ShooterConstants.MotorID.PRIMARY_SHOOTER, ShooterConstants.MotorID.SECONDARY_SHOOTER, ShooterConstants.MotorID.ANGLE_MOTOR_ONE, ShooterConstants.MotorID.ANGLE_MOTOR_TWO);
  }

  //feeder definition
  private void setupFeeder() {
    feeder = new Feeder(FeederConstants.MotorID.FEEDER_MOTOR, FeederConstants.MotorID.SHOOTER_INTAKE);
  }

  public void setupClimb() {
    climb = new Climb(ClimbConstants.MotorID.CLIMB_MOTOR);
  }

  public void startTask () {
    new Limit(climb.getMotor()).schedule();
  }



  private void configureControllerBindings() {

    // xBox Operator Below ->
      m_xboxController = new CommandXboxController(0);
      // When the D-Pad is pressed down:
      m_xboxController.povDown()
      .whileTrue(Commands.run(() -> intake.adjustPosition(-0.15))
      .finallyDo(() -> intake.stopPosition()));
      // When the D-Pad is pressed up:
      m_xboxController.povUp()
      .whileTrue(Commands.run(() -> intake.adjustPosition(0.15))
      .finallyDo(() -> intake.stopPosition()));



      // Trigger shooter angle for d-pad left
      // m_xboxController.povLeft()
      // .whileTrue(Commands.run(() -> shooter.adjustAngle(0.04))
      // .finallyDo(() -> shooter.stopAngle()));
      // m_xboxController.povRight()
      // .whileTrue(Commands.run(() -> shooter.adjustAngle(-0.04))
      // .finallyDo(() -> shooter.stopAngle()));

      new Trigger(() -> m_xboxController.getLeftY() == 1).whileTrue(Commands.run(() -> shooter.adjustAngle(0.04)).finallyDo(() -> shooter.stopAngle()));
      new Trigger(() -> m_xboxController.getLeftY() == -1).whileTrue(Commands.run(() -> shooter.adjustAngle(-.04)).finallyDo(() -> shooter.stopAngle()));
      
      // Will either turn the spin motor on or off (runs each time left trigger button is pressed)
      m_xboxController.leftTrigger().onTrue(new InstantCommand(() -> shooter.configureShoot(1)));
      m_xboxController.b().onTrue(new InstantCommand(() -> intake.configureSpin(1)));
    
      // turns on indexer and feed at the same time
      m_xboxController.rightTrigger().onTrue(new InstantCommand(() -> {
        feeder.configureFeedIdle(1);
      }));


      // safety that runs everything backward
      m_xboxController.x().onTrue(new InstantCommand(()->{
        feeder.configureFeedIdle(-1);
        intake.configureSpin(-1);
        shooter.configureShoot(-1);
        
      }));



      m_xboxController.y().onTrue(new InstantCommand(() -> {
        
        if(goingUp) {
          new Limit(climb.getMotor()).schedule();
          goingUp = false;
        } else if(climb.getMotor().getPosition().getValueAsDouble() < 1) {
          new Max(climb.getMotor()).schedule();
          goingUp = true;
        }
      }));


    // SwerveDrive JoyStick Below -> 
    m_driverController = new CommandJoystick(1); // Initialize our joystick (on DriverStation port 1)
      m_swerveDrive.setDefaultCommand(
      new TeleopDrive(
            m_swerveDrive,
            // Axis 0: X (Left & Right)
            () -> m_driverController.getRawAxis(0), 
            // Axis 1: Y (Forward & Backward)
            () -> -m_driverController.getRawAxis(1), 
            // Axis 2: Z Rotation 
            () -> -m_driverController.getRawAxis(2) 
        ));
      m_driverController.trigger().onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));  // Sets the gryo heading to point 0 -> The direction its pointing becomes forward essentially 
      // While the side button is pressed we allow rotations. Otherwise, the joystick will pick up too much Z rot input for basic motions (such as a linear forward motion)
      m_driverController.button(2).onTrue(new InstantCommand(() -> TeleopDrive.SetRotMultiplier(.25)))  
      .onFalse(new InstantCommand(() -> TeleopDrive.SetRotMultiplier(0)));
  }


  public void vibrateController() {
    m_xboxController.setRumble(RumbleType.kLeftRumble, 0.2);
  }

  public Command getAutonomousCommand() {

    String autoPath = "1";

    Command autoCommand = new PathPlannerAuto(autoPath);

    Command finalCommand = new SequentialCommandGroup(autoCommand);


    return finalCommand;
    
  }


  private void registerCommands() {

    NamedCommands.registerCommand("idlerFeeder", new InstantCommand(() -> {
      feeder.configureFeedIdle(1);
    }));

    NamedCommands.registerCommand("startShooter", new InstantCommand(() -> {
      shooter.configureShoot(1);
    }));

    NamedCommands.registerCommand("halt", new InstantCommand(() -> {
      shooter.configureShoot(0);
      feeder.configureFeedIdle(0);
    }));


  }
}
  
