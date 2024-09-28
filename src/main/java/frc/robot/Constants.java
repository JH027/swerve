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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveConstants{
    public static final double distance = 0.3429;
    public static final int turnEncoderId = 0;
    public static final double driveRadius = Math.hypot(0.5461/2,  0.635/2);
    public static final double trackWidthX = 0.5461/2;
    public static final double trackWidthY = 0.635/2;
    public static final double driveConversionFactor = 1 / 6.12 * 4 * Math.PI * 25.4 / 1000;
    public static final double turnConversionFactor = 2 * Math.PI * 150.0 / 7;
  }
  public static class IndexerConstants{
    public static final int beamBreakId = 7;
  }
}
