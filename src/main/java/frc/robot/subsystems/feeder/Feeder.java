package frc.robot.subsystems.feeder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.MathUtil;

public class Feeder {
    private TalonFX feederMotor;
    private final DutyCycleOut feederCycleOut = new DutyCycleOut(0); // Percent voltage out (0-1)
    private boolean isSpinning = false;
        
    // One motor to control the indexer speed
    public Feeder(int feederMotorID) {
        feederMotor = new TalonFX(feederMotorID);
   }

   
    // Acts as a switch (spin is either on or off) 
    public void configureFeed() {
        if(isSpinning) {
            isSpinning = false;
            disableSpin();
        } else {
            isSpinning = true;
            enableSpin();
        }

    }

   public void enableSpin() {
    feederMotor.setControl(feederCycleOut.withOutput(-0.3));
   }

   public void disableSpin() {
    feederMotor.setControl(feederCycleOut.withOutput(0));
   }

}

    
