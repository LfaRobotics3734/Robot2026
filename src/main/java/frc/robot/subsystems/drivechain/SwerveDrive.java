package frc.robot.subsystems.drivechain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Gyroscope gyro; 
    private final SwerveModule[] swerveModules;
    
    // Define a max speed for the robot (m/s). Kraken X60s are fast!
    public static final double kMaxSpeed = 1; 

    public SwerveDrive() {
        gyro = new Gyroscope();

        // 1. Initialize real SwerveModules (Drive ID, Steer ID, Analog Encoder ID)
        // CHECK YOUR CAN IDs AND ANALOG PORTS!
        swerveModules = new SwerveModule[] {
            new SwerveModule(1, 2, 0, false, true, .9879), // Front Left
            new SwerveModule(3, 4, 3, false, false, .8629), // Front Right
            new SwerveModule(7, 8, 1, true, false, .8519),  // Back Left
            new SwerveModule(5, 6, 2, false, false, .8869), // Back Right
            
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
    }

    /**
     * Main drive method.
     * @param xSpeed Forward/Backward speed (m/s)
     * @param ySpeed Left/Right speed (m/s)
     * @param rot Angular speed (rad/s)
     * @param fieldRelative Whether inputs are relative to the field or robot
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // 1. Get the rotation. Use getRotation2d() if your gyro class has it!
        Rotation2d currentHeading = gyro.getAngle(); 

        // 2. Create ChassisSpeeds
        ChassisSpeeds speeds;
        if (fieldRelative) {
            // Ensure 'rot' is definitely passed here
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentHeading);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // 3. Convert to states
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        // 4. DESATURATION CHECK
        // If kMaxSpeed is 4.5 but rot is high, this prevents wheels from stopping
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        for (int i = 0; i < 4; i++) {
            swerveModules[i].setState(swerveModuleStates[i]);
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

    @Override
    public void periodic() {
        // Update the odometry with current gyro and wheel positions
        odometry.update(gyro.getAngle(), getModulePositions());

    // Send module data to Shuffleboard
        swerveModules[0].updateTelemetry("FL");
        swerveModules[1].updateTelemetry("FR");
        swerveModules[2].updateTelemetry("BL");
        swerveModules[3].updateTelemetry("BR");
    
    // Also log the Gyro
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle().getDegrees());
    }
}