package frc.robot.subsystems.feeder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.MathUtil;

public class Feeder {
    private TalonFX feederMotor;
    private TalonFX idlerMotor; 
    private final DutyCycleOut feederCycleOut = new DutyCycleOut(0); // Percent voltage out (0-1)
    private final DutyCycleOut indexerCycleOut = new DutyCycleOut(0);
    private boolean isSpinning = false;
    

        
    // One motor to control the indexer speed
    public Feeder(int feederMotorID, int idlerMotorID) {
        feederMotor = new TalonFX(feederMotorID);
        idlerMotor = new TalonFX(idlerMotorID);
        // idlerMotor = new TalonFX()
   }

   
    // Acts as a switch (spin is either on or off) 
    public void configureFeedIdle(int diddy) {
        if(isSpinning) {
            isSpinning = false;
            disableSpin();
        } else {
            isSpinning = true;
            enableSpin(diddy);
        }

    }

   public void enableSpin(int reverse) {
    feederMotor.setControl(feederCycleOut.withOutput(-0.3 * reverse));
    idlerMotor.setControl(indexerCycleOut.withOutput(0.6 * reverse));
   }

   public void disableSpin() {
    feederMotor.setControl(feederCycleOut.withOutput(0));
    idlerMotor.setControl(indexerCycleOut.withOutput(0));
   }

}

    
