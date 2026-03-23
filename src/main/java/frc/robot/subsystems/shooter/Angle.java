package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.MathUtil;


public class Angle {
    private TalonFX angleMotor1;
    private TalonFX angleMotor2;
    private double angleMotor1MaxRot = 0.95;
    private double angleMotor2MaxRot = -3;
    private double angleMotor1Speed = 0.01;
    private double angleMotor2Speed= angleMotor1Speed * 3;
    private DutyCycleOut cycleOutMotor1 = new DutyCycleOut(0);
    private DutyCycleOut cycleOutMotor2 = new DutyCycleOut(0);

    private double[][] expectedRotations = { {0.0, 0.0}, {0.4, 1.2}, {0.8, 2.4} }; // Low, Medium, High

    private int level = 0; // 0: Low (base), 1: Med, 2: High
    private int angleMotor1Index = 0;
    private int angleMotor2Index = 1;

    public Angle(TalonFX motor1, TalonFX motor2) {
        this.angleMotor1 = motor1;
        this.angleMotor2 = motor2;
    }

    public void adjustLevel(int dir) {
        level = MathUtil.clamp(level + dir, 0,2); // Sets min of 0, max of 2

        this.new AdjustPosition().schedule();
    }

    public int getLevel() {
        return level;
    }

    public class AdjustPosition extends Command {

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            angleMotor1.setControl(cycleOutMotor1.withOutput(angleMotor1Speed));
            angleMotor2.setControl(cycleOutMotor2.withOutput(angleMotor2Speed));
        }


        @Override
        public boolean isFinished() {
            return (angleMotor2.getPosition().getValueAsDouble() > Math.abs(expectedRotations[level][angleMotor2Index]));
    }
    }

    public class Limit extends Command {
        
        @Override
        public void execute() {
            angleMotor1.setControl(cycleOutMotor1.withOutput(0.02));
            angleMotor2.setControl(cycleOutMotor2.withOutput(0.06));
        }

        @Override
        public boolean isFinished() {
            return Math.abs(angleMotor2.getPosition().getValueAsDouble()) > 1.5;
        }

        @Override
        public void end(boolean interrupted) {
            angleMotor1.setControl(cycleOutMotor1.withOutput(0));
            angleMotor2.setControl(cycleOutMotor2.withOutput(0));
            int motorResets = 0;
    
            while(motorResets < 3) {
                motorResets++;
                angleMotor1.setPosition(0);  // reset encoder at bottom; don't block (getPosition can be stale and freeze the thread)
                angleMotor2.setPosition(0);
            }
        }
    }

}

