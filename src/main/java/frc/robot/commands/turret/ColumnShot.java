package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.ros.laser.RPLidarA1.LaserScan;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanEntry;
import frc.FSLib.ros.laser.RPLidarA1.LaserScanMath;
import frc.FSLib.util.Util;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class ColumnShot extends Command {
  private final Turret turret;
  private final double targetHeightMeters;
  private final int targetFiredCount;

  private final Timer reloadTimer = new Timer();
  private int tureFiredCount = 0;

  private double targetVelocity = TurretConstants.kMaxSpeedRadPerSec;
  private double pitch = TurretConstants.kMinAngleRad;

  private final LaserScanEntry entry = new LaserScanEntry("LaserScan");

  public ColumnShot(Turret turret, double targetHeightMeters, int targetFireCount) {
    this.turret = turret;
    this.targetHeightMeters = targetHeightMeters;
    this.targetFiredCount = targetFireCount;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    reloadTimer.reset();
    tureFiredCount = 0;
  }

  @Override
  public void execute() {
    LaserScan scan = entry.get();
    double distance = LaserScanMath.calculateTargetDistance(scan);
    pitch = Util.clamp(
      Math.PI / 2 - Math.atan2(targetHeightMeters - TurretConstants.kTurretToRobotCenter.getZ() + Math.pow(distance, 2) * 0.019358, distance),
      TurretConstants.kMinAngleRad,
      TurretConstants.kMaxAngleRad
    );
    turret.setVelocity(targetVelocity, targetVelocity * 0.8);
    turret.setPitch(pitch);

    if (turret.isNearTargetSpeeds(targetVelocity, targetVelocity * 0.8)) {
      reloadTimer.start();
    }

    if (reloadTimer.isRunning()) {
      if (reloadTimer.get() < 0.2) {
        turret.setTrigger(12);
      } else if (reloadTimer.get() < 1.2) {
        turret.setTrigger(0);
      } else {
        reloadTimer.reset();
        tureFiredCount++;
      };
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
    reloadTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return tureFiredCount == targetFiredCount;
  }
}
