// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class IntakeConstants {
    public static final class MotorID{
      public static final int POSITION_MOTOR = 10;
      public static final int SPIN_MOTOR = 12;
    }
  }

  public static final class ShooterConstants {
    public static final class MotorID {
      public static final int ANGLE_MOTOR_ONE = 5;
      public static final int ANGLE_MOTOR_TWO = 11;
      public static final int PRIMARY_SHOOTER = 7;
      public static final int SECONDARY_SHOOTER = 6;
    }
    /** Angle positions: 0 = lowest, 1 = mid, 2 = highest. Motor1: forward = up (0..0.95). Motor2: reverse = up (0..-4). */
    public static final double ANGLE_POS_0_M1 = 0.0;
    public static final double ANGLE_POS_0_M2 = 0.0;
    public static final double ANGLE_POS_1_M1 = 0.475;
    public static final double ANGLE_POS_1_M2 = -2.0;
    public static final double ANGLE_POS_2_M1 = 0.95;
    public static final double ANGLE_POS_2_M2 = -4.0;
    public static final double ANGLE_POSITION_KP = 0.6;
    /** Min duty when moving (overcome static friction). Lower = gentler, may not move. */
    public static final double ANGLE_POSITION_MIN_OUTPUT = 0.08;
    /** Max duty for angle motors when moving. */
    public static final double ANGLE_POSITION_MAX_OUTPUT = 0.20;
    public static final double ANGLE_POSITION_TOLERANCE = 0.02;
    public static final double ANGLE_HOLD_KG = 0.020;
  }

  public static final class FeederConstants {
    public static final class MotorID{
      public static final int FEEDER_MOTOR = 4;
      public static final int SHOOTER_INTAKE = 8;   
    }
  }

    public static final class ClimbConstants {
    public static final class MotorID {
      public static final int CLIMB_MOTOR = 9;
    }
  }


  public static final class WheelConstants {
    public static final class MotorID {
      public static final int FRONT_LEFT_STEER = 15; // 15
      public static final int FRONT_LEFT_DRIVE = 14; // 14

      public static final int FRONT_RIGHT_STEER = 17; // 17
      public static final int FRONT_RIGHT_DRIVE = 16; // 16

      public static final int BACK_RIGHT_STEER = 2; // 2
      public static final int BACK_RIGHT_DRIVE = 1; // 1

      public static final int BACK_LEFT_STEER = 13; // 13
      public static final int BACK_LEFT_DRIVE = 3; // 3
    }

    public static final class EncoderID {
      public static final int FRONT_LEFT_ENCODER = 0;
      

    }
  }

}
