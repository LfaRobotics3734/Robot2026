package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        angleConfig1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.95;
        angleConfig1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleConfig1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        TalonFXConfiguration angleConfig2 = new TalonFXConfiguration();
        angleConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleConfig2.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleConfig2.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        angleConfig2.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleConfig2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -4.0; // Define peak
        

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
    /** Call periodically (e.g. from teleopPeriodic or subsystem periodic) to run position control. */
    public void runAnglePositionControl() {
        
    }

    private double targetRotationsMotor1(int position) {
        return switch (position) {
            case 0 -> ShooterConstants.ANGLE_POS_0_M1;
            case 1 -> ShooterConstants.ANGLE_POS_1_M1;
            default -> ShooterConstants.ANGLE_POS_2_M1;
        };
    }

    private double targetRotationsMotor2(int position) {
        return switch (position) {
            case 0 -> ShooterConstants.ANGLE_POS_0_M2;
            case 1 -> ShooterConstants.ANGLE_POS_1_M2;
            default -> ShooterConstants.ANGLE_POS_2_M2;
        };
    }

    public void stopAngle() {
        double kG = ShooterConstants.ANGLE_HOLD_KG;
        angleMotor1.setControl(angleCycleOut.withOutput(kG));
        angleMotor2.setControl(angleCycleOut.withOutput(-kG));
    }


    // Acts as a switch (spin is either on or off) 
    public void configureShoot(int diddy) {
        if(isSpinning) {
            isSpinning = false;
            disableShoot();
        } else {
            isSpinning = true;
            enableShoot(.8 * diddy, .4 * diddy);
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
