package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final int kDriveXboxPort = 0;
  public static final double kDefaultPeriod = 0.01;

  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.kReal : Mode.kSim;
  public static enum Mode {
    kReal,
    kSim,
    kReplay;
  }
  
  public static RobotMode kRobotMode = RobotMode.kDisable;
  public static enum RobotMode {
    kManual,
    kAssist,
    kAuto,
    kDisable
  }

  public static final class FieldConstants {
    public static final double kTargetX = 7.5;
    public static final double kLeftTargetY = 2.9;
    public static final double kMidTargetY = 2.0;
    public static final double kRightTargetY = 1.1;
    public static final double kLowTargetHeight = 1.2;
    public static final double kMidTargetHeight = 1.8;
    public static final double kHighTargetHeight = 2.4;
    public static final double kSpecialTargetHeight = 2.9;
  }

}