package frc.robot.gamepiece;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.gamepiece.CargoOnField.TEL_10_CARGO_INFO;

import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.turret.TurretConstants;

public class CargoOnFly extends GamePieceProjectile {
  public CargoOnFly(
    Pose2d robotPosition,
    Angle shooterAngle,
    Translation3d targetPose
  ) {
    super(
      TEL_10_CARGO_INFO,
      robotPosition.getTranslation(),
      TurretConstants.kTurretToRobotCenter.toTranslation2d().rotateBy(Rotation2d.kCCW_90deg),
      new ChassisSpeeds(),
      robotPosition.getRotation().plus(Rotation2d.fromDegrees(-90)),
      Meters.of(TurretConstants.kTurretToRobotCenter.getZ()),
      MetersPerSecond.of(22),
      shooterAngle
    );
    super
      .withProjectileTrajectoryDisplayCallBack(
        (pose3ds) -> Logger.recordOutput("GamePiece/SuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
        (pose3ds) -> Logger.recordOutput("GamePiece/UnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
      )
      .withTargetPosition(() -> targetPose)
      .withTargetTolerance(new Translation3d(0.2, 0.2, 0.2))
      .withHitTargetCallBack(() -> System.out.println("Hit special net, + 10 points!"));
  }
}
