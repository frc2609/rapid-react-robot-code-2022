// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // right click in shuffleboard, select Show as... -> Toggle Button
    public static final String INTAKE_OVERRIDE_STRING = "Intake OV";
    public static final String FEEDER_OVERRIDE_STRING = "Feed Lock";

    public final class Xbox {
        public static final double JOYSTICK_DRIFT_TOLERANCE = 0.1;
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int LEFT_STICK_X_AXIS = 0;
        public static final int LEFT_STICK_Y_AXIS = 1;
        public static final int RIGHT_STICK_X_AXIS = 4;
        public static final int RIGHT_STICK_Y_AXIS = 5;
        public static final int LEFT_TRIGGER_AXIS = 2;
        public static final int RIGHT_TRIGGER_AXIS = 3;
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int START_BUTTON = 8;
        public static final int BACK_BUTTON = 7;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;
        public static final int POV_UP_BUTTON = 0;
        public static final int POV_DOWN_BUTTON = 180;
        public static final int POV_RIGHT_BUTTON = 90;
        public static final int POV_LEFT_BUTTON = 270;
    }

    public final class PwmMotorId {
        public static final int LOWER_BELT_MOTOR = 1;
        public static final int UPPER_BELT_MOTOR = 0;
        public static final int INTAKE_BALL_MOTOR = 3;
        public static final int INTAKE_LIFT_MOTOR = 2;
    }

    public final class CanMotorId {
        public static final int HOOD_MOTOR = 11;
        public static final int SHOOTER_ROTATE_MOTOR = 10;
        public static final int SHOOTER_RIGHT_MOTOR = 9;
        public static final int SHOOTER_LEFT_MOTOR = 8;
        public static final int HOOK_MOTOR = 7;
        public static final int BAR_MOTOR = 6;
        public static final int LEFT_FRONT_MOTOR = 2;
        public static final int LEFT_REAR_MOTOR = 3;
        public static final int RIGHT_FRONT_MOTOR = 5;
        public static final int RIGHT_REAR_MOTOR = 4;
    }

    public final class ArmValue {
        public static final int MAX_ARM_POS = 98;
        public static final int MIN_ARM_POS = 0;
        public static final double ARM_SPEED_MULTIPLIER = -2.0;
        public static final double HOOK_SPEED_MULTIPLIER = 0.75;
    }

    public final class SweetSpot {
        public static final double MIN = 4; // PLACEHOLDER
        public static final double MAX = 14; // PLACEHOLDER
    }

    public final class Rotate {
        public static final double PROPORTIONAL = 0.2; //0.11
        public static final double INTEGRAL = 0.0; //0.0000000005;
        public static final double DERIVATIVE = 0.0;
        public static final double INTEGRAL_ZONE = 1.0;
        public static final double FEED_FORWARD = 0.0025;
        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = -1.0;
        public static final int MAX_POS = 20;
        public static final int MIN_POS = -20;
        public static final double TOLERANCE = 1;  // How much the limelight can be off in x-direction (degrees)
    }

    public final class Flywheel {
        public static final double PROPORTIONAL = 0; //0.00008;
        public static final double INTEGRAL = 0; //0.0000007;
        public static final double DERIVATIVE = 0; //0.0003;
        public static final double INTEGRAL_ZONE = 0; //300.0;
        public static final double FEED_FORWARD = 0; //0.000229;
        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = 0.0;
        public static final double LOW_GOAL_RPM = 1600; //= 1600;
    }

    public final class Hood {
        public static final double PROPORTIONAL = 0;
        public static final double INTEGRAL = 0;
        public static final double DERIVATIVE = 0;
        public static final double INTEGRAL_ZONE = 0;
        public static final double FEED_FORWARD = 0;
        public static final double MAX_OUTPUT = 1.0;
        public static final double MIN_OUTPUT = 0;
        public static final double LOW_GOAL_RPM = 0;
    }

    public final class Motors {
        public static final double INTAKE_SPEED = 1;
        public static final double INTAKE_LIFT_SPEED = 0.4;
        public static final double BELT_SPEED = 1;
    }

    public static final class LED {
        public static final int PWM_PORT = 4;
        public static final double RED = 0.61;
        public static final double GREEN = 0.77;
        public static final double BLUE = 0.87;
        public static final double PURPLE = 0.91;
    }

    public static final class Drive {
        public static final double isDrivingForwardDeadzone = -250;
    }

    public static final class DriveKin {
        public static final double ksVolts = 0.24533; //0.24364;
        public static final double kvVoltSecondsPerMeter = 2.8011; //2.8238;
        public static final double kaVoltSecondsSquaredPerMeter = 0.40306; //0.46022;
        public static final double kPDriveVel = 4.945; //5.7579; 
        public static final double kTrackwidthMeters = 0.56;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final int proxThreshold = 100;
    
        public static boolean isReversed = false;
        public static double rpmTolerance = 20;
        public static double commandTimer = 3;
    }
}
