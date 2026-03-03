package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.MathUtil;


public class Intake {
    private TalonFX spinMotor;
    private TalonFX positionMotor;
    private final DutyCycleOut spinCycleOut = new DutyCycleOut(0); 
    private final DutyCycleOut positionCycleOut = new DutyCycleOut(0);
    private boolean isSpinning = false;
        
    // Two motors total -> positionMotor for up/down, and spinMotor for powering the axles. 
    public Intake(int spinMotorID, int positionMotorID) {
        spinMotor = new TalonFX(spinMotorID);
        positionMotor = new TalonFX(positionMotorID);
   }

   // Used to lift or lower the intake 
   public void adjustPosition(int speed) {
    speed = MathUtil.clamp(speed, -1, 1);

    positionMotor.setControl(positionCycleOut.withOutput(speed));
   }

   public void stopPosition() {
    positionMotor.setControl(positionCycleOut.withOutput(0));
   }
   
    public void configureSpin() {
        if(isSpinning) {
            isSpinning = false;
            disableSpin();
        } else {
            isSpinning = true;
            enableSpin();
        }

    }

   public void enableSpin() {
    spinMotor.setControl(spinCycleOut.withOutput(0.15));
   }

   public void disableSpin() {
    spinMotor.setControl(spinCycleOut.withOutput(0));
   }

}
