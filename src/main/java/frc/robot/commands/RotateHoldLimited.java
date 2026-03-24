package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivechain.SwerveDrive;

/** Hold-to-spin with a cap; pair with {@code Trigger.whileTrue}. */
public class RotateHoldLimited extends Command {
    private static final double kOmegaRadPerSec = 1.0;

    private final SwerveDrive drive;
    /** +1 = CCW, -1 = CW (WPILib omega sign). */
    private final double directionSign;
    private final double maxDegrees;

    private double startRad;

    public RotateHoldLimited(SwerveDrive drive, double directionSign, double maxDegrees) {
        this.drive = drive;
        this.directionSign = directionSign < 0 ? -1.0 : 1.0;
        this.maxDegrees = maxDegrees;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startRad = drive.getPose2d().getRotation().getRadians();
    }

    @Override
    public void execute() {
        double current = drive.getPose2d().getRotation().getRadians();
        double delta = MathUtil.angleModulus(current - startRad);
        double maxRad = Math.toRadians(maxDegrees);

        double omega;
        if (directionSign > 0) {
            omega = (delta >= maxRad) ? 0.0 : kOmegaRadPerSec;
        } else {
            omega = (delta <= -maxRad) ? 0.0 : -kOmegaRadPerSec;
        }
        drive.driveRelative(new ChassisSpeeds(0.0, 0.0, omega));
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
