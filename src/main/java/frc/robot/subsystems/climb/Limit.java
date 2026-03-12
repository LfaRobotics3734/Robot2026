package frc.robot.subsystems.climb;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;


public class Limit extends Command {
    private TalonFX motor;
    private DutyCycleOut cycleOut = new DutyCycleOut(0);
    private StatusSignal<Current> currentSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private double current = 0;
    private double velocity = 0;

    public Limit(TalonFX talonMotor) {
        this.motor = talonMotor;
    }
    
    @Override
    public void execute() {
        motor.setControl(cycleOut.withOutput(-0.025));
        currentSignal = motor.getStatorCurrent();
        velocitySignal = motor.getVelocity();
    }

    public boolean isFinished() {

        current = currentSignal.refresh().getValueAsDouble();
        velocity = velocitySignal.refresh().getValueAsDouble();


        SmartDashboard.putNumber("Climb Motor Rotations", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb Motor Velocity", velocity);
        SmartDashboard.putNumber("Climb Motor Current", current);

        return current > 10 && Math.abs(velocity) < 0.1;
    }

    public void end(boolean interuppted) {

    }




}
