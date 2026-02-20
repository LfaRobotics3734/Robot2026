package frc.robot.subsystems.drivechain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class DriveTrain{

    public DriveTrain() {
        return;
    }

        TalonFX motor = new TalonFX(0);
        DutyCycleOut request = new DutyCycleOut(0);

    public void drive(double voltPercent) {
        motor.setControl(request.withOutput(voltPercent));
    }

    
}
