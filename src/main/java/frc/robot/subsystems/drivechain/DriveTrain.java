package frc.robot.subsystems.drivechain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class DriveTrain{

    public DriveTrain() {
        return;
    }

        private TalonFX motor = new TalonFX(0);
        private DutyCycleOut request = new DutyCycleOut(0);

    public void drive(double voltPercent) {
        motor.setControl(request.withOutput(voltPercent));
    }

    
}
