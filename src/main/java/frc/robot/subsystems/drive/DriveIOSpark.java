package frc.robot.subsystems.drive;

import static frc.FSLib.util.SparkUtil.assertOk;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.FSLib.util.SparkUtil;

import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveIOSpark implements DriveIO {
  private final SparkMax frontLeftMotor = new SparkMax(kFrontLeftMotorID, MotorType.kBrushless);
  private final SparkMax backLeftMotor = new SparkMax(kBackLeftMotorID, MotorType.kBrushless);
  private final SparkMax frontRightMotor = new SparkMax(kFrontRightMotorID, MotorType.kBrushless);
  private final SparkMax backRightMotor = new SparkMax(kBackRightMotorID, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = frontLeftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = frontRightMotor.getEncoder();

  private final SparkClosedLoopController leftController = frontLeftMotor.getClosedLoopController();
  private final SparkClosedLoopController rightController = frontRightMotor.getClosedLoopController();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  public DriveIOSpark() {
    SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig();
    frontLeftMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true) 
      .smartCurrentLimit(30);
    frontLeftMotorConfig.encoder
      .positionConversionFactor(kPositionConversionFactor)
      .velocityConversionFactor(kVelocityConversionFactor);
    frontLeftMotorConfig.signals.apply(SparkUtil.optimizedSignalConfig())
      .appliedOutputPeriodMs(5)
      .primaryEncoderPositionAlwaysOn(true)
      .primaryEncoderPositionPeriodMs(5)
      .primaryEncoderVelocityAlwaysOn(true)
      .primaryEncoderVelocityPeriodMs(5);
    frontLeftMotorConfig.closedLoop
      .pid(kP, kI, kD);
    assertOk(frontLeftMotor, () -> frontLeftMotor.configure(frontLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig backLeftMotorConfig = new SparkMaxConfig();
    backLeftMotorConfig
      .idleMode(IdleMode.kBrake)
      .follow(frontLeftMotor, false)
      .smartCurrentLimit(30);
    backLeftMotorConfig.signals.apply(SparkUtil.optimizedAllSignalConfig());
    assertOk(backLeftMotor, () -> backLeftMotor.configure(backLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig();
    frontRightMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(30);
    frontRightMotorConfig.encoder
      .positionConversionFactor(kPositionConversionFactor)
      .velocityConversionFactor(kVelocityConversionFactor);
    frontRightMotorConfig.signals.apply(SparkUtil.optimizedSignalConfig())
      .appliedOutputPeriodMs(5)
      .primaryEncoderPositionAlwaysOn(true)
      .primaryEncoderPositionPeriodMs(5)
      .primaryEncoderVelocityAlwaysOn(true)
      .primaryEncoderVelocityPeriodMs(5);
    frontRightMotorConfig.closedLoop
      .pid(kP, kI, kD);
    assertOk(frontRightMotor, () -> frontRightMotor.configure(frontRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig backRightMotorConfig = new SparkMaxConfig();
    backRightMotorConfig
      .idleMode(IdleMode.kBrake)
      .follow(frontRightMotor, false)
      .smartCurrentLimit(30);
    backRightMotorConfig.signals.apply(SparkUtil.optimizedAllSignalConfig());
    assertOk(backRightMotor, () -> backRightMotor.configure(backRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionMeters = leftEncoder.getPosition();
    inputs.leftVelocityMetersPerSec = leftEncoder.getVelocity();
    inputs.leftAppliedVolts = frontLeftMotor.getAppliedOutput() * frontLeftMotor.getBusVoltage();
    inputs.leftCurrentAmps = frontLeftMotor.getOutputCurrent();

    inputs.rightPositionMeters = rightEncoder.getPosition();
    inputs.rightVelocityMetersPerSec = rightEncoder.getVelocity();
    inputs.rightAppliedVolts = frontRightMotor.getAppliedOutput() * frontRightMotor.getBusVoltage();
    inputs.rightCurrentAmps = frontRightMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    frontLeftMotor.setVoltage(leftVolts);
    frontRightMotor.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    double leftFFVolts = feedforward.calculate(leftMetersPerSec);
    double rightFFVolts = feedforward.calculate(rightMetersPerSec);
    leftController.setReference(leftMetersPerSec , ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
    rightController.setReference(rightMetersPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
  }
}
