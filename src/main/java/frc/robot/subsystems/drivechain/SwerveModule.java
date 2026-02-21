package frc.robot.subsystems.drivechain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final TalonFX driveMotor, steerMotor;
    private final AnalogEncoder absoluteEncoder;
    
    private final PIDController steerPID;
    // Using DutyCycleOut for simplicity; use VelocityVoltage if you have a closed-loop drive
    private final DutyCycleOut driveControl = new DutyCycleOut(0); 
    private final DutyCycleOut steerControl = new DutyCycleOut(0);

    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID) {
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        absoluteEncoder = new AnalogEncoder(new AnalogInput(encoderID));

        // Configure Kraken X60s (Talon FX)
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        driveMotor.getConfigurator().apply(config);
        steerMotor.getConfigurator().apply(config);

        // Steer PID: Telling it that 0 and 1 (rotations) are the same point
        steerPID = new PIDController(0.5, 0, 0); // Tune these values!
        steerPID.enableContinuousInput(0, 1); 
        
    }

    public void setState(SwerveModuleState state) {
        // 1. Get current angle from the analog encoder (returns 0.0 to 1.0)
        Rotation2d currentAngle = Rotation2d.fromRotations(absoluteEncoder.get());

        // 2. Optimize the state (prevents the wheel from spinning > 90 degrees)
        state.optimize(currentAngle);

        // 3. Calculate Steering Output
        double steerOutput = steerPID.calculate(absoluteEncoder.get(), state.angle.getRotations());
        steerMotor.setControl(steerControl.withOutput(steerOutput));

        // 4. Set Drive Output (Converting m/s to a % percentage for now)
        // Note: Replace 4.5 with your robot's actual max speed
        double driveOutput = state.speedMetersPerSecond / 4.5; 
        driveMotor.setControl(driveControl.withOutput(driveOutput));
    }

    // Inside SwerveModule.java

/**
 * Returns the current position and rotation of the module.
 * Useful for Odometry.
 */
    public SwerveModulePosition getPosition() {
        // 1. Get distance from the Kraken (Drive Motor)
        // Formula: (Rotations / GearRatio) * Circumference
        double motorRotations = driveMotor.getPosition().getValueAsDouble();
        double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
        double gearRatio = 6.75; // Example ratio for MK4i L2 - Change to yours!
        
        double distanceMeters = (motorRotations / gearRatio) * wheelCircumference;

        // 2. Get angle from Analog Encoder (0 to 1 rotations)
        Rotation2d angle = Rotation2d.fromRotations(absoluteEncoder.get());

        return new SwerveModulePosition(distanceMeters, angle);
    }

    public void updateTelemetry(String name) {
    // Current angle in rotations (0 to 1)
        SmartDashboard.putNumber(name + " Encoder", absoluteEncoder.get());
    
    // The angle the PID is trying to reach
        SmartDashboard.putNumber(name + " Target", steerPID.getSetpoint());
    
    // The current error (how far off the wheel is)
        SmartDashboard.putNumber(name + " Error", steerPID.getPositionError());
    }
}