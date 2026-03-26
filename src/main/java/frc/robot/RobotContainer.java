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
//import swervelib.simulation.ironmaple.simulation.opponentsim.SmartOpponentConfig.ModuleConfig;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.drivechain.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Limit;
import frc.robot.subsystems.climb.Max;
import frc.robot.subsystems.shooter.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.Preferences;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



/**
 * subsystems, commands, and trigger mappings ->
 */
public class RobotContainer {
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  private CommandJoystick m_driverController;
  private CommandXboxController m_xboxController;
  private SwerveDrive m_swerveDrive = new SwerveDrive();
  private Intake intake;
  private Shooter shooter;
  private Feeder feeder;
  private Climb climb;
  private Angle angle;

  private boolean goingUp = false;
 

  private NetworkTable climbTab;
  private DoublePublisher climbCurrent;
  private DoublePublisher climbRotations;
  private DoublePublisher climbVelocity;
  private BooleanPublisher climbAtZero;

  private NetworkTable shooterAngleTab;
  private DoublePublisher angleMotor1Rotations;
  private DoublePublisher angleMotor1Velocity;
  private DoublePublisher angleMotor2Rotations;
  private DoublePublisher angleMotor2Velocity;
  private IntegerPublisher angleLevel;
  private DoublePublisher YStickInput; 
  private IntegerPublisher yButtonCount; 
  private int yCount = 0;

  public RobotContainer() {

    
    // Main init for subsystems
    setupIntake();
    setupShooter();
    setupFeeder();
    configureControllerBindings();

    setupAngle();
    setupClimb();

    registerCommands();

    setupShuffleBoard();
    setupUsbCamera();
  }


  public void setupShuffleBoard() {
    NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    climbTab = networkInstance.getTable("Climb");
    climbCurrent = climbTab.getDoubleTopic("Climb Current").publish();
    climbRotations = climbTab.getDoubleTopic("Climb Rotations").publish();
    climbVelocity = climbTab.getDoubleTopic("Climb Velocity").publish();
    climbAtZero = climbTab.getBooleanTopic("At Zero").publish();
    yButtonCount = climbTab.getIntegerTopic("Times Y Pressed").publish();

    shooterAngleTab = networkInstance.getTable("ShooterAngle");
    angleMotor1Rotations = shooterAngleTab.getDoubleTopic("Angle Motor1 Rotations").publish();
    angleMotor2Rotations = shooterAngleTab.getDoubleTopic("Angle Motor2 Rotations").publish();

    angleMotor1Velocity = shooterAngleTab.getDoubleTopic("Angle Motor1 Velocity").publish();
    angleMotor2Velocity = shooterAngleTab.getDoubleTopic("Angle Motor2 Velocity").publish();

    angleLevel = shooterAngleTab.getIntegerTopic("Angle Level").publish();
    YStickInput = shooterAngleTab.getDoubleTopic("XboxYStickInput").publish();
    
  }

  /** First USB camera on RoboRIO; Shuffleboard.getTab creates the tab if missing. */
  private void setupUsbCamera() {
    if (!RobotBase.isReal()) {
      return;
    }
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    camera.setFPS(24);

    Shuffleboard.getTab("Driver")
        .add("USB Camera", camera)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 0)
        .withSize(5, 4);
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

  public void setupAngle() {
    angle = new Angle(shooter.getAngleMotor1(), shooter.getAngleMotor2());
  }

  public void startTask () {
    boolean climbAtZero = Preferences.getBoolean("ClimbAtZero", false);
    // climbAtZero = false;
    if(!climbAtZero) {
      new Limit(climb.getMotor()).schedule();
    } else {
        Limit.callCount++; // We add a call anyways just for consistency 
    }

    // angle.new Limit().schedule();
    
  }


  public void resetVariables() {
    Limit.resetVariables();
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


      final double stickUp = -0.9;
      final double stickDown = 0.9;

    //   new Trigger(() -> m_xboxController.getLeftY() <= stickUp && angle.getLevel() != 2).onTrue(new InstantCommand(() -> {

    //     angle.adjustLevel(1);
    //   }));

    //   // At max (2): rumble when they press up again
    //   new Trigger(() -> (m_xboxController.getLeftY() >= stickDown && angle.getLevel() == 0))
    //   .onTrue(new InstantCommand(() -> m_xboxController.setRumble(RumbleType.kLeftRumble, 0.2))
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(new InstantCommand(() -> m_xboxController.setRumble(RumbleType.kLeftRumble, 0.0))));

    //   new Trigger(() -> (m_xboxController.getLeftY() <= stickUp && angle.getLevel() == 2)) 
    //   .onTrue(new InstantCommand(() -> m_xboxController.setRumble(RumbleType.kLeftRumble, 0.2))
    //   .andThen(new WaitCommand(0.5))
    //   .andThen(new InstantCommand(() -> m_xboxController.setRumble(RumbleType.kLeftRumble, 0.0))));
    

    //   new Trigger(() -> m_xboxController.getLeftY() >= stickDown && angle.getLevel() > 0).onTrue(new InstantCommand(() -> {
    //     angle.adjustLevel(-1);
    //   }));


    new Trigger(() -> -m_xboxController.getLeftY() > 0.2).whileTrue(Commands.run(() -> shooter.moveAngle(0.04)).finallyDo(() -> shooter.stopAngle()));
    new Trigger(() -> -m_xboxController.getLeftY() < -0.2).whileTrue(Commands.run(() -> shooter.moveAngle(-.04)).finallyDo(() -> shooter.stopAngle()));
    
      
      // Will either turn the spin motor on or off (runs each time left trigger button is pressed)
      m_xboxController.leftTrigger().onTrue(new InstantCommand(() -> shooter.configureShoot(1, false)));
      m_xboxController.b().onTrue(new InstantCommand(() -> intake.configureSpin(1)));
    m_xboxController.rightBumper().onTrue(new InstantCommand(() -> shooter.configureShoot(1, true)));
      // turns on indexer and feed at the same time
      m_xboxController.rightTrigger().onTrue(new InstantCommand(() -> {
        feeder.configureFeedIdle(1);
      }));


      // safety that runs everything backward
      m_xboxController.x().onTrue(new InstantCommand(()->{
        feeder.configureFeedIdle(-1);
        intake.configureSpin(-1);
        shooter.configureShoot(-1, false);
        
      }));



      m_xboxController.y().onTrue(new InstantCommand(() -> {
        System.out.println("Pressed Y with goingup as: " + goingUp);
        yCount++;
        if(goingUp) {
          new Limit(climb.getMotor()).schedule();
          goingUp = false;
        } else {
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
            () -> -m_driverController.getRawAxis(2),
            () -> m_driverController.getHID().getPOV()));
      // m_driverController.trigger()
      // While the side button is pressed we allow rotations. Otherwise, the joystick will pick up too much Z rot input for basic motions (such as a linear forward motion)
      m_driverController.button(2).onTrue(new InstantCommand(() -> TeleopDrive.SetRotMultiplier(.45)))  
      .onFalse(new InstantCommand(() -> TeleopDrive.SetRotMultiplier(0)));


      m_driverController.button(12).onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));  // Sets the gryo heading to point 0 -> The direction its pointing becomes forward essentially 
      // POV hat: up = zero heading (handled in TeleopDrive on POV-up edge); left/right/down in TeleopDrive
  }




  public void updateShuffleBoard() { // Periodic
    
    // Climb
    climbCurrent.set(climb.getCurrent());
    climbVelocity.set(climb.getVelocity());
    climbRotations.set(climb.getMotor().getPosition().getValueAsDouble());
    climbAtZero.set(Preferences.getBoolean("ClimbAtZero", false));
    yButtonCount.set(yCount);
    //Shooter Angle
    angleMotor1Rotations.set(shooter.getAngleMotor1().getPosition().getValueAsDouble());
    angleMotor2Rotations.set(shooter.getAngleMotor2().getPosition().getValueAsDouble());
    angleLevel.set(angle.getLevel());
    YStickInput.set(m_xboxController.getLeftY());
  }

  public Command getAutonomousCommand() {

    String autoPath = "trenchShoot";
    Limit.callCount++; // Just to say that next call it needs to lift
    Command autoCommand = new PathPlannerAuto(autoPath);

    Command finalCommand = new SequentialCommandGroup(autoCommand);


    return finalCommand;
    
  }


  private void registerCommands() {

    NamedCommands.registerCommand("idlerFeeder", new InstantCommand(() -> {
      feeder.configureFeedIdle(1);
    }));

    NamedCommands.registerCommand("startShooter", new InstantCommand(() -> {
      shooter.configureShoot(1, false);
    }));

    NamedCommands.registerCommand("halt", new InstantCommand(() -> {
      shooter.configureShoot(1, false);
      feeder.configureFeedIdle(1);
    }));

    NamedCommands.registerCommand("climbMax", new InstantCommand(() -> {
        new Max(climb.getMotor());
      }));

    NamedCommands.registerCommand("climbLift", new InstantCommand(() -> {
        new Limit(climb.getMotor());
      }));

  }
}
  