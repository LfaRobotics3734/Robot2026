package frc.robot.subsystems.drivechain;

import java.io.IOException;

// import org.json.simple.parser.ParseException;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.LimeLight;
import frc.robot.Robot;
import frc.robot.subsystems.limelight.LimeLight;

public class SwerveDrive extends SubsystemBase {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Gyroscope gyro; 
    private final LimeLight limeLight;
    private final SwerveModule[] swerveModules;
    // private final SwerveDrivePoseEstimator poseEstimator;
    private boolean[] SwerveOn;

    private double xSpeed, ySpeed, rot;
    private Rotation2d currentHeading;
    
    // Define a max speed for the robot (m/s). Kraken X60s are fast!
    public static final double kMaxSpeed = 2; 

    public SwerveDrive() {
        gyro = new Gyroscope();
        limeLight = new LimeLight();
        currentHeading = gyro.getAngle();

        SwerveOn = new boolean[] {
            true,
            true,
            true,
            true
        };

        // 1. Initialize real SwerveModules (Drive ID, Steer ID, Analog Encoder ID)

        swerveModules = new SwerveModule[] {
            new SwerveModule(Constants.WheelConstants.MotorID.FRONT_LEFT_DRIVE, Constants.WheelConstants.MotorID.FRONT_LEFT_STEER, 2, false, false, .8911), // Front Left .8895
            new SwerveModule(Constants.WheelConstants.MotorID.FRONT_RIGHT_DRIVE, Constants.WheelConstants.MotorID.FRONT_RIGHT_STEER, 1, true, false, .8465), // Front Right .8515
            new SwerveModule(Constants.WheelConstants.MotorID.BACK_LEFT_DRIVE, Constants.WheelConstants.MotorID.BACK_LEFT_STEER, 3, false, false, .8675),  // Back Left .8685
            new SwerveModule(Constants.WheelConstants.MotorID.BACK_RIGHT_DRIVE, Constants.WheelConstants.MotorID.BACK_RIGHT_STEER, 0, true, false, .4835), // Back Right .4915
            
        };
         
        // 2. Setup Kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)),
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)),
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)),
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))
        );

        // 3. Initialize Odometry
        odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getAngle(),
            getModulePositions(),
            new Pose2d(0, 0, new Rotation2d())
        );

        // poseEstimator = new SwerveDrivePoseEstimator(kinematics, currentHeading, getModulePositions(), null);

        
        // try {
        //     robotConfig = RobotConfig.fromGUISettings();
        // } catch (IOException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // } catch (ParseException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }

        // AutoBuilder.configure(
        //     limeLight::getPose2d, 
        //     this::resetOrdometry, 
        //     this::getRobotRelativeSpeeds, 
        //     this::driveRelative, 
        //     new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in
        //         // your Constants class
        //         new com.pathplanner.lib.config.PIDConstants(1, 0, 0), // Translation PID constants
        //         new PIDConstants(1, 0, 0)
        //     ), // Rotation PID constants
        //     robotConfig, 
        //     () -> {
        //         // Boolean supplier that controls when the path will be mirrored for the red alliance
        //         // This will flip the path being followed to the red side of the field.
        //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //         }, 
        //     this
        // ); //Finish configuring once limelight is done

    }

    /**
     * Main drive method.
     * @param xSpeed Forward/Backward speed (m/s)
     * @param ySpeed Left/Right speed (m/s)
     * @param rot Angular speed (rad/s)
     * @param fieldRelative Whether inputs are relative to the field or robot
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        Rotation2d currentHeading = gyro.getAngle(); 

        // 2. Create ChassisSpeeds
        ChassisSpeeds speeds;
        if (fieldRelative) {
            // Ensure 'rot' is definitely passed here
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentHeading);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, -rot);
        }

        // 3. Convert to states
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        // 4. DESATURATION CHECK
        // If kMaxSpeed is 4.5 but rot is high, this prevents wheels from stopping
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        for (int i = 0; i < 4; i++) {
            if (SwerveOn[i]) swerveModules[i].setState(swerveModuleStates[i]);
        }
        
        // swerveModules[0].setState(swerveModuleStates[0]);


        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.currentHeading = currentHeading;

        // poseEstimator.update(currentHeading, getModulePositions());

    }

    public void driveRelative(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            if (SwerveOn[i]) swerveModules[i].setState(states[i]);
        }
        
    }

    /** Helper to get all module positions for odometry */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    /** Reset the gyro heading (Field Oriented "Zero") */
    public void zeroHeading() {
        gyro.zeroYaw();
    }

    // public void resetOrdometry(Pose2d pose) {
    //     poseEstimator.resetPose(pose);
        
    // }

    @Override
    public void periodic() { // Ticks every 20ms
        // Update the odometry with current gyro and wheel positions
        
        odometry.update(gyro.getAngle(), getModulePositions());

    // Send module data to Shuffleboard
        swerveModules[0].updateTelemetry("FL");
        swerveModules[1].updateTelemetry("FR");
        swerveModules[2].updateTelemetry("BL");
        swerveModules[3].updateTelemetry("BR");
    
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle().getDegrees()); // Range of 0 - 360 (placed in shuffleboard)
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, currentHeading);
    }

    public Pose2d getPose2d() {
        return limeLight.getPose2d();
    }

}