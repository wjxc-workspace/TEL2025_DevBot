package frc.robot.commands.drive;

import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeedMetersPerSec;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class ArcadeDrive extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier, zSupplier;
  private final SlewRateLimiter limiter = new SlewRateLimiter(1);

  public ArcadeDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.zSupplier = zSupplier;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double x = limiter.calculate(MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.02));
    double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), 0.02);
    var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);
    drive.setVelocity(speeds.left * kMaxSpeedMetersPerSec, speeds.right * kMaxSpeedMetersPerSec);
  }
}
