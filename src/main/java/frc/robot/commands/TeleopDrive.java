package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivechain.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopDrive extends Command {
    private static final double kPovOmegaRadPerSec = 1.0;
    /** Stop when within this many degrees of cap (avoids never quite reaching target). */
    private static final double kPovCapToleranceDeg = 1.0;
    /** POV up: align to field heading 0 (rad error → ω). */
    private static final double kPovUpAlignKp = 2.0;
    private static final double kPovUpAlignToleranceRad = Math.toRadians(2.0);

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier vX, vY, vRot;
    private final IntSupplier povSupplier;
    private static double rotMultiplier;

    private int lastPov = -1;
    /** NavX cumulative yaw (deg) at POV press; monotonic so caps work at any starting heading. */
    private double povStartYawDeg = Double.NaN;
    /** Latched after reaching cap until POV is released (stops vision/pose threshold flutter). */
    private boolean povCapped = false;

    /**
     * @param swerveDrive The subsystem
     * @param vX Supplier for forward/backwar;d (e.g., joystick.getLeftY)
     * @param vY Supplier for left/right (e.g., joystick.getLeftX)
     * @param vRot Supplier for rotation (e.g., joystick.getRightX)
     * @param povSupplier POV hat angle in degrees, or -1 if none (e.g. joystick.getHID().getPOV())
     */
    public TeleopDrive(
            SwerveDrive swerveDrive,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier vRot,
            IntSupplier povSupplier) {
        this.swerveDrive = swerveDrive;
        this.vX = vX;
        this.vY = vY;
        this.vRot = vRot;
        this.povSupplier = povSupplier;
        addRequirements(swerveDrive);
        rotMultiplier = 0.0;
    }

    public static void SetRotMultiplier(double rotMulti) {
        rotMultiplier = rotMulti;
    }

    @Override
    public void execute() {
        // 1. Apply Deadband (ignores small inputs under 10%)
        // 2. Scale by Max Speed
        double x = MathUtil.applyDeadband(vX.getAsDouble(), 0.075) * SwerveDrive.kMaxSpeed;
        double y = MathUtil.applyDeadband(vY.getAsDouble(), 0.075) * SwerveDrive.kMaxSpeed;

        // Rotation usually feels better a bit slower (e.g., 2 rotations per second)
        double rot = MathUtil.applyDeadband(vRot.getAsDouble(), 0.3) * (Math.PI * 2);
        rot *= rotMultiplier;
        rot += povOmegaRadPerSec();

        // 3. Send to Subsystem (true = Field Relative)
        swerveDrive.drive(x * 0.5, y * 0.5, rot, true);
    }

    /**
     * POV spin: uses NavX cumulative yaw (not pose) so Limelight fusion cannot toggle the “angle so
     * far.” Latch freezes ω after cap until the hat is released.
     */
    private double povOmegaRadPerSec() {
        int pov = povSupplier.getAsInt();
        if (pov == -1) {
            lastPov = -1;
            povStartYawDeg = Double.NaN;
            povCapped = false;
            return 0.0;
        }
        if (pov == 0) {
            lastPov = 0;
            povStartYawDeg = Double.NaN;
            povCapped = false;
            return povUpAlignOmegaRadPerSec();
        }

        if (pov != lastPov) {
            povStartYawDeg = Double.NaN;
            povCapped = false;
            lastPov = pov;
        }
        if (povCapped) {
            return 0.0;
        }
        if (Double.isNaN(povStartYawDeg)) {
            povStartYawDeg = swerveDrive.getYawDegreesCumulative();
        }

        double currentYawDeg = swerveDrive.getYawDegreesCumulative();
        // NavX getAngle(): positive yaw = clockwise. CCW spin decreases this value.
        double deltaDeg = currentYawDeg - povStartYawDeg;

        double maxDeg;
        int directionSign;
        switch (pov) {
            case 90:
                maxDeg = 90.0;
                directionSign = -1;
                break;
            case 270:
                maxDeg = 90.0;
                directionSign = 1;
                break;
            case 180:
                maxDeg = 180.0;
                directionSign = 1;
                break;
            default:
                return 0.0;
        }

        if (maxDeg >= 180.0) {
            if (Math.abs(deltaDeg) >= maxDeg - kPovCapToleranceDeg) {
                povCapped = true;
                return 0.0;
            }
        } else if (directionSign > 0) {
            // Left (CCW): omega > 0 → NavX angle decreases → delta negative
            if (deltaDeg <= -(maxDeg - kPovCapToleranceDeg)) {
                povCapped = true;
                return 0.0;
            }
        } else {
            // Right (CW): omega < 0 → NavX angle increases → delta positive
            if (deltaDeg >= maxDeg - kPovCapToleranceDeg) {
                povCapped = true;
                return 0.0;
            }
        }

        return directionSign * kPovOmegaRadPerSec;
    }

    /** Rotate toward field heading 0 (same frame as {@code SwerveDrive.getFieldHeading()}). */
    private double povUpAlignOmegaRadPerSec() {
        double theta = swerveDrive.getFieldHeading().getRadians();
        double err = MathUtil.angleModulus(-theta);
        if (Math.abs(err) <= kPovUpAlignToleranceRad) {
            return 0.0;
        }
        return MathUtil.clamp(kPovUpAlignKp * err, -kPovOmegaRadPerSec, kPovOmegaRadPerSec);
    }
}