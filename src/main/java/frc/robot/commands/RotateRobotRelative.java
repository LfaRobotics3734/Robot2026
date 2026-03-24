package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivechain.SwerveDrive;

/**
 * Turns the robot in place by a fixed angle (degrees) relative to current field heading.
 * Positive delta = counterclockwise when viewed from above (WPILib convention).
 */
public class RotateRobotRelative extends Command {
    private static final double kP = 1.2;
    private static final double kD = 0.06;
    private static final double kMaxOmegaRadPerSec = 1.0;

    private final SwerveDrive drive;
    private final double deltaDeg;
    private final PIDController pid = new PIDController(kP, 0.0, kD);
    private Rotation2d target = new Rotation2d();

    public RotateRobotRelative(SwerveDrive drive, double deltaDegrees) {
        this.drive = drive;
        this.deltaDeg = deltaDegrees;
        pid.enableContinuousInput(-Math.PI, Math.PI);
        pid.setTolerance(Math.toRadians(2.0));
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Rotation2d current = drive.getPose2d().getRotation();
        target = current.plus(Rotation2d.fromDegrees(deltaDeg));
        pid.reset();
        pid.setSetpoint(target.getRadians());
    }

    @Override
    public void execute() {
        double meas = drive.getPose2d().getRotation().getRadians();
        double omega = pid.calculate(meas);
        omega = MathUtil.clamp(omega, -kMaxOmegaRadPerSec, kMaxOmegaRadPerSec);
        drive.driveRelative(new ChassisSpeeds(0.0, 0.0, omega));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
