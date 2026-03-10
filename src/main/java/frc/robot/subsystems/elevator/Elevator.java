package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class Elevator {
    private TalonFX motor;
    double motorRot = 0;
    double maxMotorRot = 0;

    private final DutyCycleOut voltageOut = new DutyCycleOut(0);

    public Elevator(int motorID) {
        motor = new TalonFX(motorID);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(motorConfig);

        motor.setPosition(0);
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



}
