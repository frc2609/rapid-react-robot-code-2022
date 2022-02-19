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
    // XBox controller
    public static final double JOYSTICK_DRIFT_TOLERANCE = 0.1;
    public static final int JOYSTICK_PORT = 0;
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

    // PWM Motors
    public static final int LOWER_BELT_MOTOR = 9;
    public static final int UPPER_BELT_MOTOR = 1;
    public static final int INTAKE_BALL_MOTOR = 2;
    public static final int INTAKE_LIFT_MOTOR = 3;
    
    // CAN Motors
    public static final int SHOOTER_RIGHT_MOTOR = 9;
    public static final int SHOOTER_LEFT_MOTOR = 8;
    public static final int HOOK_MOTOR = 7;
    public static final int BAR_MOTOR = 6;
    public static final int LEFT_FRONT_MOTOR = 4;
    public static final int LEFT_REAR_MOTOR = 5;
    public static final int RIGHT_FRONT_MOTOR = 3;
    public static final int RIGHT_REAR_MOTOR = 2;

    // Positions
    public static final int MAX_ARM_POS = 98;
    public static final int MIN_ARM_POS = 0;
    public static final double ARM_SPEED_MULTIPLIER = -2.0;
}
