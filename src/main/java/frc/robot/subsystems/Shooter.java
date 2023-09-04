package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.PID;
import frc.robot.Constants.MotorID.CAN;

public class Shooter extends SubsystemBase {
  // left motors follow right motors, so any operations on the right motors are copied by the left ones
  private final CANSparkMax leftFlywheelMotor = new CANSparkMax(CAN.SHOOTER_LEFT, MotorType.kBrushless);
  private final CANSparkMax rightFlywheelMotor = new CANSparkMax(CAN.SHOOTER_RIGHT, MotorType.kBrushless);
  private final CANSparkMax leftHoodMotor = new CANSparkMax(CAN.SHOOTER_LEFT_HOOD, MotorType.kBrushless);
  private final CANSparkMax rightHoodMotor = new CANSparkMax(CAN.SHOOTER_RIGHT_HOOD, MotorType.kBrushless);

  private SparkMaxPIDController flywheelPIDController;
  private SparkMaxPIDController hoodPIDController;
  private double manualFlywheelRpm = 0;
  private boolean isEnabled = false;

  // public final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  // public final DigitalInput stagingSensor = new DigitalInput(0);
  // public final DigitalInput shooterSensor = new DigitalInput(1);

  public Shooter() {
    rightHoodMotor.setInverted(true);
    rightFlywheelMotor.setInverted(false);
    leftFlywheelMotor.follow(rightFlywheelMotor, true);
    leftHoodMotor.follow(rightHoodMotor, true);

    rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    rightHoodMotor.setIdleMode(IdleMode.kCoast);

    flywheelPIDController = rightFlywheelMotor.getPIDController();
    hoodPIDController = rightHoodMotor.getPIDController();

    setPidValues();
    
    SmartDashboard.putNumber("Desired Flywheel RPM", 0);
    SmartDashboard.putNumber("Desired Hood RPM", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Actual Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Actual Hood RPM", getHoodRPM());
    SmartDashboard.putNumber("Left Flywheel Current", leftFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Flywheel Current", rightFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Hood Current", rightHoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left Hood Current", leftHoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Manual Flywheel RPM", manualFlywheelRpm);
    SmartDashboard.putBoolean("Shooter Enabled", isEnabled);
    // SmartDashboard.putNumber("Intake Sensor Proximity", intakeSensor.getProximity());
    // SmartDashboard.putBoolean("Intake Sensor", getIntakeSensor());
    // SmartDashboard.putBoolean("Staging Sensor", stagingSensor.get());
    // SmartDashboard.putBoolean("Shooter Sensor", shooterSensor.get());
  }

  // public boolean getIntakeSensor() {
  //   return intakeSensor.getProximity() > Constants.AutoConstants.proxThreshold;
  // }

  public double getFlywheelRPM() {
    return rightFlywheelMotor.getEncoder().getVelocity();
  }

  public double getHoodRPM() {
    return rightHoodMotor.getEncoder().getVelocity();
  }

  public void increaseFlywheelRPM() {
    manualFlywheelRpm += Constants.Shooter.FLYWHEEL_RPM_ADJUSTMENT;
    if (isEnabled) setPidReferences();
  }

  public void decreaseFlywheelRPM() {
    manualFlywheelRpm = Math.max(manualFlywheelRpm -= Constants.Shooter.FLYWHEEL_RPM_ADJUSTMENT, 0);
    if (isEnabled) setPidReferences();
  }

  public void runMotors() {
    isEnabled = true;
    setPidReferences();
  }

  private void setPidReferences() {
    final double desiredFlywheelRPM = manualFlywheelRpm * Constants.Shooter.FLYWHEEL_OVERDRIVE;
    final double desiredHoodRPM = manualFlywheelRpm * Constants.Shooter.HOOD_OVERDRIVE;
    
    SmartDashboard.putNumber("Desired Flywheel RPM", desiredFlywheelRPM);
    SmartDashboard.putNumber("Desired Hood RPM", desiredHoodRPM);
    
    flywheelPIDController.setReference(desiredFlywheelRPM, ControlType.kVelocity);
    hoodPIDController.setReference(desiredHoodRPM, ControlType.kVelocity);
  }

  private void setPidValues() {
    flywheelPIDController.setP(PID.Flywheel.PROPORTIONAL);
    flywheelPIDController.setI(PID.Flywheel.INTEGRAL);
    flywheelPIDController.setD(PID.Flywheel.DERIVATIVE);
    flywheelPIDController.setIZone(PID.Flywheel.INTEGRAL_ZONE);
    flywheelPIDController.setFF(PID.Flywheel.FEED_FORWARD);
    flywheelPIDController.setOutputRange(PID.Flywheel.MIN_OUTPUT,
        PID.Flywheel.MAX_OUTPUT);

    hoodPIDController.setP(PID.Hood.PROPORTIONAL);
    hoodPIDController.setI(PID.Hood.INTEGRAL);
    hoodPIDController.setD(PID.Hood.DERIVATIVE);
    hoodPIDController.setIZone(PID.Hood.INTEGRAL_ZONE);
    hoodPIDController.setFF(PID.Hood.FEED_FORWARD);
    hoodPIDController.setOutputRange(PID.Hood.MIN_OUTPUT,
        PID.Hood.MAX_OUTPUT);
  }

  public void stopMotors() {
    isEnabled = false;
    rightFlywheelMotor.stopMotor();
    rightHoodMotor.stopMotor();
  }
}
