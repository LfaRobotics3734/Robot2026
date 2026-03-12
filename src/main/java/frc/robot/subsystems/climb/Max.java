package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class Max extends Command {
    private TalonFX motor;
    private DutyCycleOut cycleOut = new DutyCycleOut(0);
    private double speed = 0.3;

    public Max(TalonFX tMotor) {
        this.motor = tMotor;
    }

    @Override
    public void execute() {
        motor.setControl(cycleOut.withOutput(speed));

    }

    public boolean isFinished() {
        
        return Math.abs(motor.getPosition().getValueAsDouble()) >= 383;
    }


    public void end(boolean interuppted) {
        motor.setControl(cycleOut.withOutput(0));
    }
}
