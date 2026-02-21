package frc.robot.subsystems.drivechain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope {

    private final AHRS navX;
    

    public Gyroscope() {
        // Initialize the navX on the MXP port
        navX = new AHRS(NavXComType.kMXP_SPI);
        
        // Reset the gyro so the current orientation is "0"
        zeroYaw();
    }

    /**
     * Returns the heading of the robot as a Rotation2d.
     * This handles the conversion from navX degrees to WPILib geometry.
     */
    public Rotation2d getAngle() {
        // We use getRotation2d() to ensure counter-clockwise is positive
        return navX.getRotation2d();

        // return new Rotation2d();
    }

    /**
     * Resets the gyro yaw to 0. 
     * Useful for field-relative driving when you want to reset "Forward".
     */
    public void zeroYaw() {
        navX.reset();
    }

    /**
     * Returns the rate of rotation (degrees per second).
     * Useful if you want to implement anti-tip or specialized stabilization.
     */
    public double getRotationRate() {
        return navX.getRate();

        // return 0.0;
    }
}