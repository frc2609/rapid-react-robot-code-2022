// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public final class Logitech {
        public static final double JOYSTICK_DRIFT_TOLERANCE = 0.1;
        public static final int JOYSTICK_PORT = 0;
        public static final int LEFT_STICK_X_AXIS = 0;
        public static final int LEFT_STICK_Y_AXIS = 1;
        public static final int RIGHT_STICK_X_AXIS = 2; // 4
        public static final int RIGHT_STICK_Y_AXIS = 3; // 5
        public static final int LEFT_TRIGGER_AXIS = 2;
        public static final int RIGHT_TRIGGER_AXIS = 3;
        public static final int BUTTON_2 = 2; // 1
        public static final int BUTTON_3 = 3; // 2
        public static final int BUTTON_1 = 1; // 3
        public static final int BUTTON_4 = 4; // 4
        public static final int START_BUTTON = 10; // 8
        public static final int BACK_BUTTON = 7;
        public static final int LEFT_TOP_BUMPER = 5;
        public static final int RIGHT_TOP_BUMPER = 6;
        public static final int LEFT_BOTTOM_BUMPER = 7;
        public static final int RIGHT_BOTTOM_BUMPER = 8;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;
        public static final int POV_UP_BUTTON = 0;
        public static final int POV_DOWN_BUTTON = 180;
        public static final int POV_RIGHT_BUTTON = 90;
        public static final int POV_LEFT_BUTTON = 270;
    }

    public final class PwmMotorId {
        public static final int LOWER_BELT_MOTOR = 0;
        public static final int UPPER_BELT_MOTOR = 1;
        public static final int INTAKE_BALL_MOTOR = 2;
        public static final int INTAKE_LIFT_MOTOR = 3;
    }

    public final class CanMotorId {
        public static final int SHOOTER_HOOD_MOTOR = 11;
        public static final int SHOOTER_ROTATE_MOTOR = 10;
        public static final int SHOOTER_RIGHT_MOTOR = 9;
        public static final int SHOOTER_LEFT_MOTOR = 8;
        public static final int HOOK_MOTOR = 7;
        public static final int BAR_MOTOR = 6;
        public static final int LEFT_FRONT_MOTOR = 4;
        public static final int LEFT_REAR_MOTOR = 5;
        public static final int RIGHT_FRONT_MOTOR = 3;
        public static final int RIGHT_REAR_MOTOR = 2;
    }

    public final class ArmValue {
        public static final int MAX_ARM_POS = 98;
        public static final int MIN_ARM_POS = 0;
        public static final double ARM_SPEED_MULTIPLIER = -2.0;
    }

    public final class Rotate {
        public static final double proportialPIDConstant = 0.2; //0.11
        public static final double integralPIDConstant = 0.0; //0.0000000005;
        public static final double derivativePIDConstant = 0.0;
        public static final double integralPIDZone = 1.0;
        public static final double feedForwardPIDConstant = 0.0025;
        public static final double maxPIDOutput = 1.0;
        public static final double minPIDOutput = -1.0;
        public static final int MAX_POS = 20;
        public static final int MIN_POS = -20;
        public static final double TOLERANCE = 1;  // How much the limelight can be off in x-direction (degrees)
    }

    public final class Hood {
        public static final double proportialPIDConstant = 1;
        public static final double integralPIDConstant = 0.0002;
        public static final double derivativePIDConstant = 0.0;
        public static final double integralPIDZone = 0.0;
        public static final double feedForwardPIDConstant = 0.0;
        public static final double maxPIDOutput = 1.0;
        public static final double minPIDOutput = -1.0;
        public static final double MAX_POS = 2.7;
        public static final double MIN_POS = 0.0;
    }

    public final class Flywheel {
        public static final double proportialPIDConstant = 0.00006; // 0.00004;
        public static final double integralPIDConstant = 0.0; //0.0002;
        public static final double derivativePIDConstant = 0.0;
        public static final double integralPIDZone = 5.0;
        public static final double FeedForwardPIDConstant = 0.000195; //0.000183;
        public static final double maxPIDOutput = 1.0;
        public static final double minPIDOutput = 0.0;
    }
}
