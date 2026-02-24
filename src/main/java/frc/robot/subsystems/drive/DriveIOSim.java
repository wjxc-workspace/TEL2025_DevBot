package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;

public class DriveIOSim implements DriveIO {
  private DifferentialDrivetrainSim driveSim = 
    new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      kGearRatio,
      2.5,
      25.0,
      kWheelRadius,
      kTrackWidth,
      null
    );

  private boolean closedLoop = false;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private PIDController leftPID = new PIDController(kSimkP, kSimkI, kSimkD);
  private PIDController rightPID = new PIDController(kSimkP, kSimkI, kSimkD);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kSimkS, kSimkV, kSimkA);

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts = leftFFVolts + leftPID.calculate(driveSim.getLeftVelocityMetersPerSecond());
      rightAppliedVolts = rightFFVolts + rightPID.calculate(driveSim.getRightVelocityMetersPerSecond());
    }

    driveSim.setInputs(leftAppliedVolts, rightAppliedVolts);
    driveSim.update(Constants.kDefaultPeriod);

    inputs.leftPositionMeters = driveSim.getLeftPositionMeters();
    inputs.leftVelocityMetersPerSec = driveSim.getLeftVelocityMetersPerSecond();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = driveSim.getLeftCurrentDrawAmps();

    inputs.rightPositionMeters = driveSim.getRightPositionMeters();
    inputs.rightVelocityMetersPerSec = driveSim.getRightVelocityMetersPerSecond();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = driveSim.getRightCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    closedLoop = true;
    leftFFVolts = feedforward.calculate(leftMetersPerSec);
    rightFFVolts = feedforward.calculate(rightMetersPerSec);
    leftPID.setSetpoint(leftMetersPerSec);
    rightPID.setSetpoint(rightMetersPerSec);
  }
}
