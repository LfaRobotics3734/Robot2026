package frc.robot.subsystems.climb;
import edu.wpi.first.wpilibj.Preferences;

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
    private double speed = -0.35; // pos = up clockwise - neg = down counter
    private double liftingSpeed = -0.9;
    public static int callCount = 0; // Called once at beginning, second to lift, third to lower (i need to hardcode this bc timecrunch)



    public Limit(TalonFX talonMotor) {
        this.motor = talonMotor;
        Limit.callCount++;
    }  


    @Override
    public void initialize() {
        currentSignal = motor.getStatorCurrent();
        velocitySignal = motor.getVelocity();
        Preferences.setBoolean("ClimbAtZero", false);

    }

    @Override
    public void execute() {
        if(callCount == 2) {
            motor.setControl(cycleOut.withOutput(-liftingSpeed));
        } else {
            motor.setControl(cycleOut.withOutput(-speed));
        }
        currentSignal.refresh();
        velocitySignal.refresh();
    }

    public boolean isFinished() {
        current = currentSignal.getValueAsDouble();
        velocity = velocitySignal.getValueAsDouble();


        // SmartDashboard.putNumber("Climb Motor Rotations", motor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Climb Motor Velocity", velocity);
        // SmartDashboard.putNumber("Climb Motor Current", current);
        if(callCount == 2) {
            return (Math.abs(current) > 10);
        } else {
        return (Math.abs(current) > 2.3); // && Math.abs(velocity) < (speed * 90.0); // OKAY SO NORMAL VELOCITY IS THE SPEED * 100
        }
    }

    public void end(boolean interuppted) {
        motor.setControl(cycleOut.withOutput(0));
        
        
        int motorResets = 0;

        while(motorResets < 3) {
            motorResets++;
            motor.setPosition(0);  // reset encoder at bottom; don't block (getPosition can be stale and freeze the thread)
        }
        if(!interuppted) {
            Preferences.setBoolean("ClimbAtZero", true);
        }
    }




}
