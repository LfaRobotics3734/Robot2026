package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class Intake {
    private TalonFX spinMotor;
    private TalonFX positionMotor;
    private final DutyCycleOut spinCycleOut = new DutyCycleOut(0); 
    private final DutyCycleOut positionCycleOut = new DutyCycleOut(0);
        
    // Two motors total -> positionMotor for up/down, and spinMotor for powering the axles. 
    public Intake(int spinMotorID, int positionMotorID) {
        spinMotor = new TalonFX(spinMotorID);
        positionMotor = new TalonFX(positionMotorID);
   }

   // Used to lift or lower the intake 
   public void adjustPosition(int ySpeed, boolean down) {
    if(down) {
        TalonFXConfiguration positionMotorConfig = new TalonFXConfiguration();
        positionMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        positionMotor.getConfigurator().apply(positionMotorConfig);
    }
    positionMotor.setControl(positionCycleOut.withOutput(ySpeed));
   }
   
   public void enableSpin() {
    // spinMotor.setControl(spinCycleOut.withOutput(0)); (TODO)
   }

   public void disableSpin() {
    spinMotor.setControl(spinCycleOut.withOutput(0));
   }

}
