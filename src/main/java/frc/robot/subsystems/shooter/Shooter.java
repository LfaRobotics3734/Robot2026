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
    private TalonFX idlerMotor;
    private final DutyCycleOut shooterCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut angleCycleOut = new DutyCycleOut(0); 
    private boolean isSpinning = false;
    
    //Motor variables 
    public Shooter(int primaryMotorID, int secondaryMotorID, int angleMotor1ID, int angleMotor2ID, int idlerMotorID) {
        //H: shooter definition
        
        primaryMotor = new TalonFX(primaryMotorID);
        secondaryMotor = new TalonFX(secondaryMotorID);

        angleMotor1 = new TalonFX(angleMotor1ID);
        angleMotor2 = new TalonFX(angleMotor2ID);

        idlerMotor = new TalonFX(idlerMotorID);
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
        angleConfig2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -2.0;

        //H: Angle motors lock
        angleMotor1.getConfigurator().apply(angleConfig1);
        angleMotor2.getConfigurator().apply(angleConfig2);


        angleMotor1.setPosition(0);
        angleMotor2.setPosition(0);
        while(Math.abs(getAngle(angleMotor1)) > 0.005) {
            angleMotor1.setPosition(0);
        }

        while(Math.abs(getAngle(angleMotor2)) > 0.005) {
            angleMotor2.setPosition(0);
        }

        SmartDashboard.putNumber("Angle Motor 1 (For = up)", getAngle(angleMotor1));
        SmartDashboard.putNumber("Angle Motor 2 (Rev = up)", getAngle(angleMotor2));
    }

    public double getAngle(TalonFX motor) {
        return motor.getPosition().getValueAsDouble();
    }

//H: adjusting the position of the shooter
    public void adjustAngle(double speed) {
    speed = MathUtil.clamp(speed, -1, 1); // Even if speed is any number , it can't exceed the range of -1 to 1
    if(!(getAngle(angleMotor1) > 0.95)) { // IF we havent hit max rotations
        angleMotor1.setControl(angleCycleOut.withOutput(speed));
    }
    if(!(Math.abs(getAngle(angleMotor2)) > 2)) {
        angleMotor2.setControl(angleCycleOut.withOutput(-speed * 2));
    }
    SmartDashboard.putNumber("Angle Motor 1 (For = up)", getAngle(angleMotor1));
    SmartDashboard.putNumber("Angle Motor 2 (Rev = up)", getAngle(angleMotor2));
   }

    public void stopAngle() {
        // Replace '0.05' with the minimum power needed to keep your arm from falling
        
        //after encoder is added make it os only use motors if it is not in resting position otherwise motors will smoke if too long
        
        double kG = 0.020; 
        angleMotor1.setControl(angleCycleOut.withOutput(kG));
        angleMotor2.setControl(angleCycleOut.withOutput(-kG));
    }


    // Acts as a switch (spin is either on or off) 
    public void configureShoot() {
        if(isSpinning) {
            isSpinning = false;
            disableShoot();
        } else {
            isSpinning = true;
            enableShoot(.5, .3);
        }

    }


//Enable and disable spin on the shooting motors
//H: I disabled the for loops since there needs to be OPPOSITE spin on shooter motors
    public void enableShoot(double shooter, double idler) {
        
        primaryMotor.setControl(shooterCycleOut.withOutput(-shooter));
        secondaryMotor.setControl(shooterCycleOut.withOutput(shooter));
        idlerMotor.setControl(shooterCycleOut.withOutput(idler));
        
    }
   

    public void disableShoot() {
        primaryMotor.setControl(shooterCycleOut.withOutput(0));
        secondaryMotor.setControl(shooterCycleOut.withOutput(0));
        idlerMotor.setControl(shooterCycleOut.withOutput(0));
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
