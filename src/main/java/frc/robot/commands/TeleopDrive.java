package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivechain.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopDrive extends Command {

    private static final double kSnapMaxOmega = 3.0;
    private static final double kSnapKp = 4.0;
    private static final double kSnapKd = 0.08;

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier vX, vY, vRot;
    private final IntSupplier povSupplier;
    private static double rotMultiplier;
    private static boolean headingInverted = false;

    private final PIDController snapPid;

    /**
     * @param swerveDrive The subsystem
     * @param vX Supplier for forward/backward (e.g., joystick axis 1, negated)
     * @param vY Supplier for left/right (e.g., joystick axis 0, negated)
     * @param vRot Supplier for rotation (e.g., joystick axis 2)
     * @param povSupplier POV hat angle in degrees, or -1 if released
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

        snapPid = new PIDController(kSnapKp, 0.0, kSnapKd);
        snapPid.enableContinuousInput(-Math.PI, Math.PI);
        snapPid.setTolerance(Math.toRadians(2.0));
    }

    public static void SetRotMultiplier(double rotMulti) {
        rotMultiplier = rotMulti;
    }

    public static void ToggleHeadingInversion() {
        headingInverted = !headingInverted;
    }

    public static boolean IsHeadingInverted() {
        return headingInverted;
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(vX.getAsDouble(), 0.075) * SwerveDrive.kMaxSpeed;
        double y = MathUtil.applyDeadband(vY.getAsDouble(), 0.075) * SwerveDrive.kMaxSpeed;

        if (headingInverted) {
            x = -x;
            y = -y;
        }

        double rot;
        Rotation2d povTarget = getPovTarget();

        if (povTarget != null) {
            if (headingInverted) {
                povTarget = povTarget.plus(Rotation2d.fromDegrees(270));
            }
            snapPid.setSetpoint(povTarget.getRadians());
            double currentRad = swerveDrive.getFieldHeading().getRadians();
            rot = MathUtil.clamp(snapPid.calculate(currentRad), -kSnapMaxOmega, kSnapMaxOmega);
        } else {
            rot = MathUtil.applyDeadband(vRot.getAsDouble(), 0.3) * (Math.PI * 2);
            rot *= rotMultiplier;
            snapPid.reset();
        }

        swerveDrive.drive(x * 0.5, y * 0.5, rot, true);
    }

    /**
     * Maps the POV hat to fixed field headings (relative to the zeroed heading).
     * Returns null when no cardinal direction is pressed.
     *
     *   Up    (0)   → 0°    forward
     *   Right (90)  → −90°  clockwise
     *   Left  (270) → +90°  counter-clockwise
     *   Down  (180) → 180°  about-face
     */
    private Rotation2d getPovTarget() {
        switch (povSupplier.getAsInt()) {
            case 0:   return Rotation2d.fromDegrees(0);
            case 90:  return Rotation2d.fromDegrees(-90);
            case 270: return Rotation2d.fromDegrees(90);
            case 180: return Rotation2d.fromDegrees(180);
            default:  return null;
        }
    }
}
