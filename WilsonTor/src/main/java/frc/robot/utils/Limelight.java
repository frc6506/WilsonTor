package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  public static NetworkTable limelightTable =
      NetworkTableInstance.getDefault().getTable("limelight-bazicrg");

  public static double returnHorizontalOffset() {
    return limelightTable.getEntry("tx").getDouble(0.00);
  }

  public static double returnVerticalOffset() {
    return limelightTable.getEntry("ty").getDouble(0.00);
  }

  public static double returnAngle() {
    return limelightTable.getEntry("").getDouble(0.00);
  }
}
