package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Shooter {
    private TalonFX primaryMotor; 
    private TalonFX secondaryMotor;
    private TalonFX angleMotor1;
    private TalonFX angleMotor2;
    private final DutyCycleOut shooterCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut angleCycleOut = new DutyCycleOut(0);
    private boolean isSpinning = false;
    public int targetAngle = 0;

    
    //Motor variables 
    public Shooter(int primaryMotorID, int secondaryMotorID, int angleMotor1ID, int angleMotor2ID) {
        //H: shooter definition
        
        primaryMotor = new TalonFX(primaryMotorID);
        secondaryMotor = new TalonFX(secondaryMotorID);

        angleMotor1 = new TalonFX(angleMotor1ID);
        angleMotor2 = new TalonFX(angleMotor2ID);

        // Create Break Mode Config so motors lock after dutyCycleOut
        TalonFXConfiguration angleConfig1 = new TalonFXConfiguration();
        angleConfig1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleConfig1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleConfig1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.95; // max height in rot
        angleConfig1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleConfig1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        TalonFXConfiguration angleConfig2 = new TalonFXConfiguration();
        angleConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleConfig2.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleConfig2.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        angleConfig2.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleConfig2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.0; // Define peak
        

        //H: Angle motors lock
        angleMotor1.getConfigurator().apply(angleConfig1);
        angleMotor2.getConfigurator().apply(angleConfig2);


        SmartDashboard.putNumber("Angle Motor 1 (For = up)", getAngle(angleMotor1));
        SmartDashboard.putNumber("Angle Motor 2 (Rev = up)", getAngle(angleMotor2));

        int setPosCalls = 0;
        while(setPosCalls < 3) {
            angleMotor1.setPosition(0);
            angleMotor2.setPosition(0);
            setPosCalls++;
        }
    }



    public double getAngle(TalonFX motor) {
        return motor.getPosition().getValueAsDouble();
    }

    /** Set target angle by position: 0 = lowest (and set new 0 when reached), 1 = mid, 2 = highest. */
    public void adjustAngle(int position) {
        targetAngle = position;


        
    }


    public void moveAngle(double speed, CommandXboxController controller) {
        speed = MathUtil.clamp(speed, -1, 1); // Even if speed is any number , it can't exceed the range of -1 to 1
        if(getAngle(angleMotor1) < 0.95 && getAngle(angleMotor1) > -0.007) { // IF we havent hit max rotations
            angleMotor1.setControl(angleCycleOut.withOutput(speed * 2.25));
            
        }

        if (getAngle(angleMotor1) > .85 || getAngle(angleMotor1) < 0) {
            controller.setRumble(RumbleType.kBothRumble,1);
        } 
        if (getAngle(angleMotor1) <= .85 && getAngle(angleMotor1) >= 0) {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }

        if(getAngle(angleMotor2) > -1.05 && getAngle(angleMotor2) < 0.007) { // the 0.007 is to account for a little bit of rotation on stop causing it to go over the value
            angleMotor2.setControl(angleCycleOut.withOutput(-speed * 2));
        }
        SmartDashboard.putNumber("Angle Motor 1 (For = up)", getAngle(angleMotor1));
        SmartDashboard.putNumber("Angle Motor 2 (Rev = up)", getAngle(angleMotor2));
       }


    public TalonFX getAngleMotor1() {
        return angleMotor1;
    }

    public TalonFX getAngleMotor2() {
        return angleMotor2;
    }


    public void stopAngle() {
        double kG = ShooterConstants.ANGLE_HOLD_KG;
        angleMotor1.setControl(angleCycleOut.withOutput(kG + 0.005));
        angleMotor2.setControl(angleCycleOut.withOutput(-kG));
    }


    // Acts as a switch (spin is either on or off) 
    public void configureShoot(int diddy, boolean light) {
        if(isSpinning) {
            isSpinning = false;
            disableShoot();
        } else if (light) {
            isSpinning = true;
            enableShoot(.45, .295);
        } else {
            isSpinning = true;
            enableShoot(.7 * diddy, .3 * diddy);
        }

    }



//Enable and disable spin on the shooting motors
//H: I disabled the for loops since there needs to be OPPOSITE spin on shooter motors
    public void enableShoot(double shooter, double idler) {
        
            primaryMotor.setControl(shooterCycleOut.withOutput(-shooter));
            secondaryMotor.setControl(shooterCycleOut.withOutput(shooter));
        
    }
   

    public void disableShoot() {
        primaryMotor.setControl(shooterCycleOut.withOutput(0));
        secondaryMotor.setControl(shooterCycleOut.withOutput(0));
    }
    


}


// import com.ctre.phoenix6.configs.TalonFXConfiguration;


// public class Shooter {

//     private TalonFX[] motors;
//     private DutyCycleOut[] cycleOuts;

//     public Shooter() {

//         motors = new TalonFX[] {
//             new TalonFX(0), //Top Spinner
//             new TalonFX(0), //Sandwiched Spinner
//             new TalonFX(0), //Bottom Spinner
//             new TalonFX(0), //Elevator
//             new TalonFX(0), //Elevator
//         };
        
//         for (int i = 0; i <= 4; i++) {
//             TalonFXConfiguration motorConfig = new TalonFXConfiguration();
//             motors[i].getConfigurator().apply(motorConfig);
//         }

//         cycleOuts = new DutyCycleOut[] {
//             new DutyCycleOut(0), //Top Spinner
//             new DutyCycleOut(0), //Sandwiched Spinner
//             new DutyCycleOut(0), //Bottom Spinner
//             new DutyCycleOut(0), //Elevator
//         };

//     }

//     public void elevateShooter(double speed) {
//         motors[3].setControl(cycleOuts[3].withOutput(speed));
//         motors[4].setControl(cycleOuts[3].withOutput(-speed));
//     }
    


    
// }
