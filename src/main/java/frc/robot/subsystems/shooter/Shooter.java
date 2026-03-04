package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Shooter {
    private TalonFX[] shooterMotors; 
    private TalonFX[] angleMotors;
    private final DutyCycleOut spinCycleOut = new DutyCycleOut(0); 
    
    // Unnecessary comments left here for Hari
    public Shooter() {
        // Create Break Mode Config so motors lock after dutyCycleOut
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterMotors = new TalonFX[] {
            new TalonFX(ShooterConstants.MotorID.PRIMARY_SHOOTER), // INDEX 0 = PRIMARY AXLE MOTOR
            new TalonFX(ShooterConstants.MotorID.SECONDARY_SHOOTER), // INDEX 1 = SECONDARY AXLE MOTOR
        };

        angleMotors = new TalonFX[] {
            new TalonFX(ShooterConstants.MotorID.ANGLE_MOTOR_ONE), 
            new TalonFX(ShooterConstants.MotorID.ANGLE_MOTOR_TWO)
        };


        
    }



    public void enableSpin() {
        for (TalonFX shootMotor : shooterMotors) {
            shootMotor.setControl(spinCycleOut.withOutput(0.15));
        }
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
