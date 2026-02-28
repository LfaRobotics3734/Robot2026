package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivechain.SwerveDrive;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier vX, vY, vRot;
    private static double rotMultiplier;

    /**
     * @param swerveDrive The subsystem
     * @param vX Supplier for forward/backwar;d (e.g., joystick.getLeftY)
     * @param vY Supplier for left/right (e.g., joystick.getLeftX)
     * @param vRot Supplier for rotation (e.g., joystick.getRightX)
     */
    public TeleopDrive(SwerveDrive swerveDrive, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRot) {
        this.swerveDrive = swerveDrive;
        this.vX = vX;
        this.vY = vY;
        this.vRot = vRot;
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

        // 3. Send to Subsystem (true = Field Relative)
        swerveDrive.drive(x * 0.1, y * 0.1, rot * rotMultiplier, true);
    }
}