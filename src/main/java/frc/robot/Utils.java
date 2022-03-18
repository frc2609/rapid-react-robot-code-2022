package frc.robot;

public class Utils {
    public static double clamp(double input, double min, double max) {
        return Math.max(Math.min(input, max), min);
    }

    public static double degToRad(double degrees) {
        return degrees * Math.PI / 180;
    }
}