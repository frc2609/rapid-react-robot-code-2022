// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // selecting auto modes
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax; // motors
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // specifying brushless motor type
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import com.kauailabs.navx.frc.AHRS; // for NavX Gyro
import edu.wpi.first.wpilibj.SPI; // SPI port, NavX gyro
import edu.wpi.first.wpilibj.XboxController; // controller
import com.revrobotics.ColorMatch;

public class Robot extends TimedRobot {
  // selecting auto modes
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
    // motors
  private final CANSparkMax frontLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rearRightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax hookMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax barMotor = new CANSparkMax(5, MotorType.kBrushless);
  SparkMaxPIDController barPID;
  // joysticks
  private final XboxController joystick = new XboxController(0);
  // gyro/navx
  private AHRS navx;
  /*
  // climbing
  // do we need to declare these here (can't they be in the climb() function?)
  private boolean currentlyClimbing; // might have to declare here
  private double maxPitchForwardDegrees = 15; // placeholder value
  private double maxPitchBackwardDegrees = -15; // placeholder value
  // assuming that pitch forward is positive and pitch backward is negative
  */

  private final I2C.Port i2cPortDef = I2C.Port.kOnboard;
  // private final I2C.Port i2cPortMXP = I2C.Port.kMXP;

  private final ColorSensorV3 colorSensor1 = new ColorSensorV3(i2cPortDef);
  // private final ColorSensorV3 colorSensor2 = new ColorSensorV3(i2cPortMXP);

  private final ColorMatch colorMatcher = new ColorMatch();

  // Adjustable colour values
  private final Color kBlueTarget = new Color(0.200, 0.450, 0.300);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.332, 0.100);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  private static boolean ballIsRed = false;
  private static boolean ballIsBlue = false;
  private static boolean dumpBall = false;

  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    hookMotor.setIdleMode(IdleMode.kBrake);
    try { navx = new AHRS(SPI.Port.kMXP); }
    catch (RuntimeException ex) 
    { 
      System.out.println("Failed to initialize NAVX "); 
    }

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void robotPeriodic() 
  {
    //SmartDashboard.putBoolean("Climbing Enabled ", currentlyClimbing);
    //SmartDashboard.putNumber("NavX Pitch:", navx.getPitch());
    // smart dashboard.put stats (navx connected bool, yaw, pitch, roll, climbing bool)

    //Detects the colour, outputs it into the dashboard, and sets the ball colour ("ballIsBlue", "ballIsRed")
    Color detectedColor = colorSensor1.getColor();

    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      ballIsBlue = true;
      ballIsRed = false;
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      ballIsBlue = false;
      ballIsRed = true;
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      ballIsBlue = false;
      ballIsRed = false;
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      ballIsBlue = false;
      ballIsRed = false;
    } else {
      colorString = "Unknown";
      ballIsBlue = false;
      ballIsRed = false;
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  // auto code
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    barMotor.getEncoder().setPosition(0);
    barPID = barMotor.getPIDController();
    barPID.setP(0.034173);
    barPID.setI(0.0001);
    barPID.setOutputRange(-1, 0.1);
    hookMotor.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("arm setpoint", -19.8);
  }

  @Override
  public void teleopPeriodic() 
  {
    if (joystick.getYButtonPressed()) { navx.zeroYaw(); }
    // driving
    double driveX = Math.pow(joystick.getRawAxis(0), 3);
    double driveY = Math.pow(joystick.getRawAxis(1), 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;
    frontLeftMotor.set(leftMotors);
    rearLeftMotor.set(-leftMotors);
    frontRightMotor.set(-rightMotors);
    rearRightMotor.set(rightMotors);
    // climbing
    // 4rx, 5ry joystick axis
    hookMotor.set(Math.pow(joystick.getRawAxis(5), 3));
    double armSetp = SmartDashboard.getNumber("arm setpoint", -6);
    if (joystick.getXButton()) { // move climbing bar into place }
      barPID.setReference(armSetp, ControlType.kPosition);
      System.out.println("SETTING");
    }else{
      barMotor.set(0);
    }
    // barMotor.fsset(Math.pow(joystick.getRawAxis(4), 3));
    SmartDashboard.putNumber("bar enc", barMotor.getEncoder().getPosition());

    // Emergency dump button in case the colour sensor senses the wrong colour
    if (joystick.getRawButton(8)) {
      ballIsBlue = false;
      ballIsRed = false;
      dumpBall = true;
    }
  }

  @Override
  public void disabledInit() {
    hookMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("bar enc", barMotor.getEncoder().getPosition());}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
  
  /*
  public void climb(){
    // hook motors
    if (joystick.getLeftBumperPressed())
    hookMotor.set(-0.2);
    if (joystick.getRightBumperPressed())
    hookMotor.set(0.2);
    if (joystick.getLeftBumperReleased())
    hookMotor.set(0);
    if (joystick.getRightBumperReleased())
    hookMotor.set(0);
    
    if (navx.getRoll() >= maxPitchForwardDegrees) 
      climbingAngleMotor.set(0.2);
    else if (navx.getRoll() <= maxPitchBackwardDegrees)
      climbingAngleMotor.set(-0.2); // figure out which direction goes back and forward
    else
      climbingAngleMotor.set(0);
      // above code is for balancing
    // from back of robot:
    //* roll = tilt left/right
    //* yaw = turn left/right
    //* pitch = tilt forward/back <- this is what we need
    //
    // probably want a delay so that it doesn't try to readjust when the motor is trying to move the robot
  }
  */
}
