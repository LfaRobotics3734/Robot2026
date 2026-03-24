package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivechain.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopDrive extends Command {
    private static final double kPovOmegaRadPerSec = 2.25;
    /** P on heading error (rad) for all POV “snap to heading” directions. */
    private static final double kPovAlignKp = 4.0;
    private static final double kPovAlignToleranceRad = Math.toRadians(2.0);

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier vX, vY, vRot;
    private final IntSupplier povSupplier;
    private static double rotMultiplier;

    /** Field heading captured once when the POV first leaves center; cleared on release. */
    private Rotation2d povHeadingAtPress = null;

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
     * POV up snaps to field 0°. Other directions snap to ±90° / 180° relative to heading stored at
     * first press (no NavX reset involved).
     */
    private double povOmegaRadPerSec() {
        int pov = povSupplier.getAsInt();
        if (pov == -1) {
            povHeadingAtPress = null;
            return 0.0;
        }
        if (povHeadingAtPress == null) {
            povHeadingAtPress = swerveDrive.getFieldHeading();
        }

        Rotation2d target;
        if (pov == 0) {
            target = new Rotation2d();
        } else {
            switch (pov) {
                case 90:
                    target = povHeadingAtPress.plus(Rotation2d.fromDegrees(-90));
                    break;
                case 270:
                    target = povHeadingAtPress.plus(Rotation2d.fromDegrees(90));
                    break;
                case 180:
                    target = povHeadingAtPress.plus(Rotation2d.fromDegrees(180));
                    break;
                default:
                    return 0.0;
            }
        }
        return omegaTowardHeading(target);
    }

    private double omegaTowardHeading(Rotation2d target) {
        double err =
                MathUtil.angleModulus(
                        target.getRadians() - swerveDrive.getFieldHeading().getRadians());
        if (Math.abs(err) <= kPovAlignToleranceRad) {
            return 0.0;
        }
        return MathUtil.clamp(kPovAlignKp * err, -kPovOmegaRadPerSec, kPovOmegaRadPerSec);
    }
}
