package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.StatusSignal;


public class Climb {
    private TalonFX motor;
    private StatusSignal<Current> currentSignal;
    private StatusSignal<AngularVelocity> velocitySignal;

    double motorRot = 0;
    double maxMotorRot = 0;
    private boolean foundMin = false;

    private final DutyCycleOut voltageOut = new DutyCycleOut(0);

    public Climb(int motorID) {
        motor = new TalonFX(motorID);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(motorConfig);
            
        currentSignal = motor.getStatorCurrent();
        velocitySignal = motor.getVelocity();
    }


    public double getCurrent() {
        currentSignal.refresh();
        return currentSignal.getValueAsDouble();
    }

    public double getVelocity() {
        velocitySignal.refresh();
        return velocitySignal.getValueAsDouble();
    }
        

    public void enableSpin() {
        motorRot = motor.getPosition().getValueAsDouble();
        if(motorRot >= 0.0 && motorRot < 10.0) {
            motor.setControl(voltageOut.withOutput(0.02));
        };
    }

    public void disableSpin() {
        motor.setControl(voltageOut.withOutput(0));
    }



    public TalonFX getMotor() {
        return motor;
    }
}
