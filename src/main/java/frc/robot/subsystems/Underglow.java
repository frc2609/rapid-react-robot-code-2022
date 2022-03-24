package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Underglow extends SubsystemBase {
    // led controller pretends to be a PWM motor controller
    private final Spark LEDController = new Spark(Constants.LED.PWM_PORT);
    private final double set;

    public Underglow() {
       if (SmartDashboard.getBoolean("isRedTeam", true)) {
           set = Constants.LED.RED;
       } else {
           set = Constants.LED.BLUE;
       }
    }

    @Override
    public void periodic() {
        LEDController.set(set); // changes colour of LED
        SmartDashboard.putNumber("LED Controller Colour", set);
    }
}