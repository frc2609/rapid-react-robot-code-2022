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
  public final class Drive {
    public static final double LONGITUDINAL_SPEED_MULTIPLIER = 1.0;
    public static final double TURNING_SPEED_MULTIPLIER = 0.3; // 0.7;
    public static final double OVERALL_SPEED_MULTIPLIER = 0.8;
  }

  public final class Intake {
    public static final double INTAKE_SPEED = 1;
    public static final double INTAKE_LIFT_SPEED = 0.4;
    public static final double BELT_SPEED = 1;
    public static final double LIFT_TIME = 0.5; // seconds
  }

  public static final class LED {
    public static final int PWM_PORT = 4;
    public static final double RED = 0.61;
    public static final double GREEN = 0.77;
    public static final double BLUE = 0.87;
    public static final double PURPLE = 0.91;
  }

  public final class MotorID {
    public final class CAN {
      public static final int DRIVE_LEFT_FRONT = 2;
      public static final int DRIVE_LEFT_REAR = 3;
      public static final int DRIVE_RIGHT_FRONT = 5;
      public static final int DRIVE_RIGHT_REAR = 4;
      public static final int SHOOTER_LEFT = 8;
      public static final int SHOOTER_LEFT_HOOD = 12;
      public static final int SHOOTER_RIGHT = 9;
      public static final int SHOOTER_RIGHT_HOOD = 11;
      public static final int SHOOTER_ROTATE = 10;
    }

    public final class PWM {
      public static final int INTAKE_BALL = 3;
      public static final int INTAKE_LIFT = 2;
      public static final int LOWER_BELT = 1;
      public static final int UPPER_BELT = 0;
    }
  }

  public final class PID {
    public final class Rotate {
      public static final double PROPORTIONAL = 0.2;
      public static final double INTEGRAL = 0.0;
      public static final double DERIVATIVE = 0.0;
      public static final double INTEGRAL_ZONE = 1.0;
      public static final double FEED_FORWARD = 0.0025;
      public static final double MAX_OUTPUT = 1.0;
      public static final double MIN_OUTPUT = -1.0;
      public static final int MAX_POS = 20;
      public static final int MIN_POS = -20;
    }

    public final class Flywheel {
      public static final double PROPORTIONAL = 0.00005;
      public static final double INTEGRAL = 0.0000003;
      public static final double DERIVATIVE = 0.00001;
      public static final double INTEGRAL_ZONE = 200;
      public static final double FEED_FORWARD = 0.000195;
      public static final double MAX_OUTPUT = 1.0;
      public static final double MIN_OUTPUT = 0.0;
    }

    public final class Hood {
      public static final double PROPORTIONAL = 0.0001;
      public static final double INTEGRAL = 0.0000005;
      public static final double DERIVATIVE = 0.00001;
      public static final double INTEGRAL_ZONE = 300;
      public static final double FEED_FORWARD = 0.00018;
      public static final double MAX_OUTPUT = 1.0;
      public static final double MIN_OUTPUT = 0;
      public static final double LOW_GOAL_RPM = 0;
    }
  }

  public final class Shooter {
    public static final double FLYWHEEL_OVERDRIVE = 0.9;
    public static final int FLYWHEEL_RPM_ADJUSTMENT = 200;
    public static final double HOOD_OVERDRIVE = 1.1;
    public static final double ROTATION_SENSITIVITY_MULTIPLIER = 0.25;
  }

  public final class Xbox {
    public static final double DEADBAND = 0.1;
    public static final int CONTROLLER_PORT = 0;
  }
}
