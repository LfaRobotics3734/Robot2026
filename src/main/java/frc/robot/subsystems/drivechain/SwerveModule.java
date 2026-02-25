package frc.robot.subsystems.drivechain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
    private final double angleOffset;

    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID, boolean driveInverted, boolean steerInverted, double offset) {
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        absoluteEncoder = new AnalogEncoder(new AnalogInput(encoderID));

        this.angleOffset = offset;

        absoluteEncoder.setInverted(false);
        // Configure Kraken X60s (Talon FX) (Drive)
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        if (driveInverted) driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        driveMotor.getConfigurator().apply(driveConfig);
        
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        if (steerInverted) steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        steerMotor.getConfigurator().apply(steerConfig);

        // Steer PID: Telling it that 0 and 1 (rotations) are the same point
        steerPID = new PIDController(0.4, 0, 0);
        steerPID.enableContinuousInput(0, 1); 

        
    }

    public void setState(SwerveModuleState state) {
        // 1. Get current angle (This method MUST subtract your offset!)
        Rotation2d currentAngle = getAngle();

        // 2. Optimize the state 
        // This modifies 'state' in place (speed and angle)
        state.optimize(currentAngle); 

        // 3. Calculate Steering Output
        // IMPORTANT: Use currentAngle.getRotations() so the PID 
        // sees the same 'corrected' zero as the target.
        double steerTarget = state.angle.getRotations();
        double steerCurrent = currentAngle.getRotations();
        
        double steerOutput = steerPID.calculate(steerCurrent, steerTarget);
        
        // Send output to the steer motor
        steerMotor.setControl(steerControl.withOutput(steerOutput));

        // 4. Set Drive Output
        // state.speedMetersPerSecond is now optimized (it may be negative)
        double driveOutput = state.speedMetersPerSecond / SwerveDrive.kMaxSpeed; 
        driveMotor.setControl(driveControl.withOutput(driveOutput));

        // Debugging: lowered threshold to see slower rotation speeds
        if (Math.abs(state.speedMetersPerSecond) > 0.01) {
            System.out.println("Module Speed: " + state.speedMetersPerSecond + " Target Angle: " + state.angle.getDegrees());
        }
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
        // Rotation2d angle = Rotation2d.fromRotations(absoluteEncoder.get());

        return new SwerveModulePosition(distanceMeters, getAngle());
    }

    public void updateTelemetry(String name) {
    // Current angle in rotations (0 to 1)
        SmartDashboard.putNumber(name + " Encoder", absoluteEncoder.get());
    
    // The angle the PID is trying to reach
        SmartDashboard.putNumber(name + " Target", steerPID.getSetpoint());
    
    // The current error (how far off the wheel is)
        SmartDashboard.putNumber(name + " Error", steerPID.getPositionError());
    }

    public Rotation2d getAngle() {
    // Subtract the offset from the raw reading
        double correctedRotations = absoluteEncoder.get() - angleOffset;
        correctedRotations = ((correctedRotations % 1.0) + 1.0) % 1.0;
        return Rotation2d.fromRotations(correctedRotations);
    }
}