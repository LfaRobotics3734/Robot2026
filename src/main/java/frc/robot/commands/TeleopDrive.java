package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivechain.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopDrive extends Command {
    private static final double kPovOmegaRadPerSec = 1.0;

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier vX, vY, vRot;
    private final IntSupplier povSupplier;
    private static double rotMultiplier;

    private int lastPov = -1;
    private double povStartRad = Double.NaN;

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
     * POV spin in field-relative mode: same {@code drive()} call as translation so you can move and
     * rotate together. 90° left/right use signed delta; 180° uses |delta| so ±π both count as full flip.
     */
    private double povOmegaRadPerSec() {
        int pov = povSupplier.getAsInt();
        if (pov == -1) {
            lastPov = -1;
            povStartRad = Double.NaN;
            return 0.0;
        }
        if (pov == 0) {
            lastPov = 0;
            povStartRad = Double.NaN;
            return 0.0;
        }

        if (pov != lastPov) {
            povStartRad = Double.NaN;
            lastPov = pov;
        }
        if (Double.isNaN(povStartRad)) {
            povStartRad = swerveDrive.getPose2d().getRotation().getRadians();
        }

        double current = swerveDrive.getPose2d().getRotation().getRadians();
        double delta = MathUtil.angleModulus(current - povStartRad);

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

        double maxRad = Math.toRadians(maxDeg);
        if (maxDeg >= 180.0) {
            if (Math.abs(delta) >= maxRad - 1e-3) {
                return 0.0;
            }
        } else if (directionSign > 0) {
            if (delta >= maxRad) {
                return 0.0;
            }
        } else {
            if (delta <= -maxRad) {
                return 0.0;
            }
        }

        return directionSign * kPovOmegaRadPerSec;
    }
}