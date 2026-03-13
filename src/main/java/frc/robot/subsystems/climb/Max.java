package frc.robot.subsystems.climb;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;

public class Max extends Command {
    private TalonFX motor;
    private DutyCycleOut cycleOut = new DutyCycleOut(0);
    private StatusSignal<Current> currentSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private double current = 0;
    private double velocity = 0;
    private double speed = 0.3;

    public Max(TalonFX tMotor) {
        this.motor = tMotor;
    }
    
    @Override
    public void initialize() {
        currentSignal = motor.getStatorCurrent();
        velocitySignal = motor.getVelocity();
    }

    @Override
    public void execute() {
        motor.setControl(cycleOut.withOutput(-speed));
        currentSignal.refresh();
        velocitySignal.refresh();
    }

    public boolean isFinished() {
        current = currentSignal.getValueAsDouble();
        velocity = velocitySignal.getValueAsDouble();


        SmartDashboard.putNumber("Climb Motor Rotations", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb Motor Velocity", velocity);
        SmartDashboard.putNumber("Climb Motor Current", current);

        return Math.abs(motor.getPosition().getValueAsDouble()) >= 383;
    }


    public void end(boolean interuppted) {
        motor.setControl(cycleOut.withOutput(0));
    }
}
