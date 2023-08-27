package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Date;
import java.util.Scanner;

public class Utils {
  public static double degToRad(double degrees) {
    return degrees * Math.PI / 180;
  }

  public void printVersion() {
    Date dt = new Date();
    SmartDashboard.putString("VERSION DATE:", dt.toString());
    File deployDir = Filesystem.getDeployDirectory();
    String pathToFile = deployDir.getPath() + File.separator + "hash.txt";
    File file = new File(pathToFile);
    Scanner reader;

    try {
      reader = new Scanner(file);
      String versionHash = reader.nextLine();
      SmartDashboard.putString("VERSION HASH:", versionHash);
      reader.close();
    } catch (FileNotFoundException e) {
      SmartDashboard.putString("VERSION HASH:", "FNF ERROR");
    }
  }
}
