package frc.robot.subsystems.drivechain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope {

    private final AHRS navX;
    

public Gyroscope() {
    // Stick with the Studica-specific enum since the constructor requires it
    navX = new AHRS(NavXComType.kMXP_SPI);
    
    edu.wpi.first.wpilibj.Timer.delay(0.1); 

    if (navX != null) {
        zeroYaw();
    } else {
        System.out.println("ERROR: NavX could not be initialized!");
    }
    }

    /**
     * Returns the heading of the robot as a Rotation2d.
     */
    public Rotation2d getAngle() {
        Rotation2d angle = navX.getRotation2d();
        if (angle == null) {
         System.out.println("Received null angle!");
         return new Rotation2d(0); 
         }
        angle = angle.plus(Rotation2d.fromDegrees(90));        
        return angle;

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

    }
}