package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter {
    private TalonFX[] motors; // I believe there are three (and elevator has 2 -- move to another file??)
    private final DutyCycleOut spinCycleOut = new DutyCycleOut(0); 

    public Shooter() {
        motors = new TalonFX[] {
            new TalonFX(0), //Top Spinner
            new TalonFX(0), //Sandwiched Spinner
            new TalonFX(0), //Bottom Spinner
            new TalonFX(0), //Elevator
            new TalonFX(0), //Elevator
        };
    }

    public void enableSpin() {
        for (TalonFX motor : motors) {
            motor.setControl(spinCycleOut.withOutput(0.15));
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
