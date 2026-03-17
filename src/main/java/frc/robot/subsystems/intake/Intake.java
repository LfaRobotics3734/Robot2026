package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake {
    private TalonFX spinMotor;
    private TalonFX positionMotor;
    private final DutyCycleOut spinCycleOut = new DutyCycleOut(0); // Percent voltage out (0-1)
    private final DutyCycleOut positionCycleOut = new DutyCycleOut(0);
    private boolean isSpinning = false;
    private double rotations = 0.0;
        
    // Two motors total -> positionMotor for up/down, and spinMotor for powering the axles. 
    public Intake(int spinMotorID, int positionMotorID) {
        spinMotor = new TalonFX(spinMotorID);
        positionMotor = new TalonFX(positionMotorID);

        // Makes sure the motors lock
        TalonFXConfiguration positionConfig = new TalonFXConfiguration();
        positionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        positionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; // This is the down limit
        positionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -15.1;

        positionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        positionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;

        positionMotor.getConfigurator().apply(positionConfig);
        positionMotor.setPosition(0);
        
        while(Math.abs(getMotorRotations()) > 0.005) {
            positionMotor.setPosition(0);
        }
        SmartDashboard.putNumber("Position Motor", getMotorRotations());

   }

   public double getMotorRotations() {
    return positionMotor.getPosition().getValueAsDouble();
   }

   // Used to lift or lower the intake 
   public void adjustPosition(double speed) {
    
    
    speed = MathUtil.clamp(speed, -1, 1); // Even if speed is any number , it exceed the range of -1 to 1
    if(getMotorRotations() > -15.1 && speed < 0) { // For going down
        positionMotor.setControl(positionCycleOut.withOutput(speed));
    } else if (getMotorRotations() < 0 && speed > 0) { // for going up
        positionMotor.setControl(positionCycleOut.withOutput(speed));
    }
    

    SmartDashboard.putNumber("Position Motor", getMotorRotations());

   }

   public void stopPosition() {
    positionMotor.setControl(positionCycleOut.withOutput(0));
   }
    // Acts as a switch (spin is either on or off) 
    public void configureSpin(int diddy) {
        if(isSpinning) {
            isSpinning = false;
            disableSpin();
        } else {
            isSpinning = true;
            enableSpin(diddy);
        }

    }

   public void enableSpin(int reverse) {
    spinMotor.setControl(spinCycleOut.withOutput(-0.7 * reverse));
   }

   public void disableSpin() {
    spinMotor.setControl(spinCycleOut.withOutput(0));
   }

}
