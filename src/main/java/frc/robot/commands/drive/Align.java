package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.maths.Maths;
import frc.FSLib.ros.laser.RPLidarA1.LaserScan;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanEntry;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanMath;
import frc.robot.subsystems.drive.Drive;

public class Align extends Command {
  private final Drive drive;

  private final Timer dontSeeTimer = new Timer();

  private double targetPixel;
  private double detectedPixel, lastDetectedPixel;

  private final LaserScanEntry laserScanEntry = new LaserScanEntry("LaserScan");
  private final DoubleArraySubscriber poses = NetworkTableInstance.getDefault()
    .getTable("Vision").getDoubleArrayTopic("poses").subscribe(new double[] {});

  public Align(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    LaserScan scan = laserScanEntry.get();
    double distance = LaserScanMath.calculateTargetDistance(scan);
    targetPixel = -(Math.atan(0.15 / distance) * 203.718327);
    double detectionResults[] = poses.get();
    lastDetectedPixel = Maths.findNearestLinear(detectionResults, 0);
  }

  @Override
  public void execute() {
    LaserScan scan = laserScanEntry.get();
    double distance = LaserScanMath.calculateTargetDistance(scan);
    targetPixel = -(Math.atan(0.15 / distance) * 203.718327);
    double detectionResults[] = poses.get();
    detectedPixel = Maths.findNearestLinear(detectionResults, 0);
    if (detectedPixel != Double.NaN && lastDetectedPixel != Double.NaN && Math.abs(detectedPixel - lastDetectedPixel) < 30) {
      double speed = Math.copySign(0.15, targetPixel - detectedPixel);
      drive.setVelocity(speed, speed);
      dontSeeTimer.reset();
      lastDetectedPixel = detectedPixel;
      SmartDashboard.putNumber("targetPixel", targetPixel);
    } else {
      drive.setVelocity(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0); 
  }

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(targetPixel, detectedPixel, 1.5) || dontSeeTimer.hasElapsed(0.3);
  }
}
