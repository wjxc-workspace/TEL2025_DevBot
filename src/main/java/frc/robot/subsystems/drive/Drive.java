package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final DriveIO driveIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics kinematics =
    new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry odometry = 
    new DifferentialDriveOdometry(Rotation2d.kZero, 0.0, 0.0);

  private double gyroYawRad = 0.0;
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  public Drive (DriveIO driveIO, GyroIO gyroIO) {
    this.driveIO = driveIO;
    this.gyroIO = gyroIO;

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException();
    }

    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      this::getTwist,
      speeds -> {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveIO.setVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
      },
      new PPLTVController(Constants.kDefaultPeriod),
      robotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      this
    );

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
      path -> Logger.recordOutput("Drive/ActivatePath", path.toArray(new Pose2d[path.size()]))
    );
    PathPlannerLogging.setLogTargetPoseCallback(
      pose -> Logger.recordOutput("Drive/TargetPose", pose)
    );
  }

  @Override
  public void periodic() {
    driveIO.updateInputs(driveInputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", driveInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    if (gyroInputs.connected) {
      gyroYawRad = gyroInputs.yawRad;
    } else {
      Twist2d twist = kinematics.toTwist2d(
        driveInputs.leftPositionMeters - lastLeftPositionMeters,
        driveInputs.rightPositionMeters - lastRightPositionMeters
      );
      gyroYawRad = gyroYawRad + twist.dtheta;
      lastLeftPositionMeters = driveInputs.leftPositionMeters;
      lastRightPositionMeters = driveInputs.rightPositionMeters;
    }

    odometry.update(Rotation2d.fromRadians(gyroYawRad), driveInputs.leftPositionMeters, driveInputs.rightPositionMeters);

    // posePublisher.set(getPose());
    // twistPublisher.set(getTwist());
    // driveNTInst.flush();
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    odometry.resetPose(pose);
  }

  @AutoLogOutput
  public ChassisSpeeds getTwist() {
    return kinematics.toChassisSpeeds(
      new DifferentialDriveWheelSpeeds(driveInputs.leftVelocityMetersPerSec, driveInputs.rightVelocityMetersPerSec)
    );
  }

  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    driveIO.setVelocity(leftMetersPerSec, rightMetersPerSec);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    driveIO.setVoltage(leftVolts, rightVolts);
  }
}
