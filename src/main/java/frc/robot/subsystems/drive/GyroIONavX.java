package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);
  
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawRad = Units.degreesToRadians(-navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
  }
}
