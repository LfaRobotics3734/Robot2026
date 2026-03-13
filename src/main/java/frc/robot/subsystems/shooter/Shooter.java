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
    /** Target angle position: 0 = lowest, 1 = mid, 2 = highest. */
    private int targetAnglePosition = 0;
    private boolean hasZeroedAtLow = false;
    
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
    }

    /** Set current physical position as 0 (no motor movement). Call when teleop starts so wherever the hood is becomes the reference. */
    public void zeroAngleEncoders() {
        angleMotor1.setPosition(0);
        angleMotor2.setPosition(0);
        targetAnglePosition = 0;
        hasZeroedAtLow = true;  // already at "0" so don't re-zero when we reach target 0
    }

    public double getAngle(TalonFX motor) {
        return motor.getPosition().getValueAsDouble();
    }

    /** Set target angle by position: 0 = lowest (and set new 0 when reached), 1 = mid, 2 = highest. */
    public void adjustAngle(int position) {
        targetAnglePosition = MathUtil.clamp(position, 0, 2);
        if (targetAnglePosition != 0) hasZeroedAtLow = false;
    }

    /** Call periodically (e.g. from teleopPeriodic or subsystem periodic) to run position control. */
    public void runAnglePositionControl() {
        double t1 = targetRotationsMotor1(targetAnglePosition);
        double t2 = targetRotationsMotor2(targetAnglePosition);
        double p1 = getAngle(angleMotor1);
        double p2 = getAngle(angleMotor2);
        double err1 = t1 - p1;
        double err2 = t2 - p2;
        boolean atTarget = Math.abs(err1) <= ShooterConstants.ANGLE_POSITION_TOLERANCE
            && Math.abs(err2) <= ShooterConstants.ANGLE_POSITION_TOLERANCE;

        if (atTarget) {
            stopAngle();
            if (targetAnglePosition == 0 && !hasZeroedAtLow) {
                angleMotor1.setPosition(0);
                angleMotor2.setPosition(0);
                hasZeroedAtLow = true;
            }
        } else {
            double maxOut = ShooterConstants.ANGLE_POSITION_MAX_OUTPUT;
            double minOut = ShooterConstants.ANGLE_POSITION_MIN_OUTPUT;
            double raw1 = ShooterConstants.ANGLE_POSITION_KP * err1;
            double raw2 = -ShooterConstants.ANGLE_POSITION_KP * err2;
            // Apply minimum output when moving so motors overcome static friction
            double out1 = MathUtil.clamp(raw1, -maxOut, maxOut);
            double out2 = MathUtil.clamp(raw2, -maxOut, maxOut);
            if (out1 > 0 && out1 < minOut) out1 = minOut;
            else if (out1 < 0 && out1 > -minOut) out1 = -minOut;
            if (out2 > 0 && out2 < minOut) out2 = minOut;
            else if (out2 < 0 && out2 > -minOut) out2 = -minOut;
            angleMotor1.setControl(angleCycleOut.withOutput(out1));
            angleMotor2.setControl(angleCycleOut.withOutput(out2));
        }
        SmartDashboard.putNumber("Angle Motor 1 (For = up)", p1);
        SmartDashboard.putNumber("Angle Motor 2 (Rev = up)", p2);
        SmartDashboard.putNumber("Shooter Angle Target", targetAnglePosition);
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
