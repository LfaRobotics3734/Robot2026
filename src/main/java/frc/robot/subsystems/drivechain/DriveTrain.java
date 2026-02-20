package frc.robot.subsystems.drivechain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.AnalogInput;


public class DriveTrain{
    private TalonFX motor = new TalonFX(1);
    private DutyCycleOut request = new DutyCycleOut(0);

    public DriveTrain() {
        return;
    }

    public void drive(double voltPercent, AnalogInput encoder) {
        motor.setControl(request.withOutput(voltPercent));
        System.out.println(encoder.getVoltage());
    }

    
}
