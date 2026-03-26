package frc.robot.subsystems.drivechain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants;

public class Gyroscope {

    private final AHRS navX;
    /** Added to raw NavX+field offset so "zero" does not call navX.reset() (preserves integration). */
    private Rotation2d yawZeroOffset = Rotation2d.kZero;

    public Gyroscope() {
        // Stick with the Studica-specific enum since the constructor requires it
        navX = new AHRS(NavXComType.kMXP_SPI);

        edu.wpi.first.wpilibj.Timer.delay(0.1);

        if (navX != null) {
            navX.reset();
            zeroYaw();
        } else {
            System.out.println("ERROR: NavX could not be initialized!");
        }
    }

    /** NavX cumulative yaw (deg) since reset — monotonic, good for rotation deltas. */
    public double getYawDegreesCumulative() {
        return navX.getAngle();
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
        Rotation2d rRaw =
                angle.plus(Rotation2d.fromDegrees(Constants.DriveConstants.GYRO_FIELD_OFFSET_DEG));
        return rRaw.plus(yawZeroOffset);
    }

    /** Sets current heading to field 0 without resetting NavX (integration stays continuous). */
    public void zeroYaw() {
        Rotation2d rRaw =
                navX.getRotation2d()
                        .plus(Rotation2d.fromDegrees(Constants.DriveConstants.GYRO_FIELD_OFFSET_DEG));
        yawZeroOffset = Rotation2d.kZero.minus(rRaw);
    }

    /**
     * Returns the rate of rotation (degrees per second).
     * Useful if you want to implement anti-tip or specialized stabilization.
     */
    public double getRotationRate() {
        return navX.getRate();

    }
}