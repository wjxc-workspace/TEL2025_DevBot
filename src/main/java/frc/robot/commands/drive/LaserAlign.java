package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.ros.laser.RPLidarA1.LaserScan;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanEntry;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanMath;
import frc.robot.subsystems.drive.Drive;

public class LaserAlign extends Command {
  private final Drive drive;

  double nearestValue = 0;

  private final LaserScanEntry entry = new LaserScanEntry("LaserScan");

  private final Timer failedTimer = new Timer();

  private Angle yaw;

  public LaserAlign(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    failedTimer.restart();
  }

  @Override
  public void execute() {
    LaserScan scan = entry.get();
    yaw = LaserScanMath.calculateYaw(scan);
    if (yaw != null) {
      WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(0, Math.copySign(0.1, 1.57 - yaw.in(Radians)), false);
      drive.setVelocity(speeds.left, speeds.right);
      failedTimer.reset();
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
    return yaw == null || MathUtil.isNear(Math.PI / 2, yaw.in(Radians), 0.095) || failedTimer.hasElapsed(0.2);
  }
}
