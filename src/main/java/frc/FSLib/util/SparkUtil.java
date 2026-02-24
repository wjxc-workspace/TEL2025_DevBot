package frc.FSLib.util;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SignalsConfig;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkUtil {
  public static void assertOk(SparkMax sparkMax, Supplier<REVLibError> command) {
    REVLibError error = command.get();
    if (error != REVLibError.kOk) {
      DriverStation.reportError("can't apply configuration to SparkMax with ID " + sparkMax.getDeviceId(), true);
    }
  }

  public static enum StatusFrames{
    kAbslouteEncoder,
    kAnalogSensor,
    kAlternateEncoder,
    kPosition,
    kVelocity;
  }

  private static int kOptimizedFrameRateMs = 500;

  public static SignalsConfig optimizedAllSignalConfig() {
    return optimizedSignalConfig(StatusFrames.values());
  }

  public static SignalsConfig optimizedSignalConfig() {
    return optimizedSignalConfig(StatusFrames.kAbslouteEncoder, StatusFrames.kAlternateEncoder, StatusFrames.kAnalogSensor);
  }

  public static SignalsConfig optimizedSignalConfig(StatusFrames ...frames) {
    SignalsConfig config = new SignalsConfig();
    for (StatusFrames frame : frames) {
      switch (frame) {
        case kAbslouteEncoder:
          config
            .absoluteEncoderPositionPeriodMs(kOptimizedFrameRateMs)
            .absoluteEncoderVelocityPeriodMs(kOptimizedFrameRateMs);
          break;
        case kAlternateEncoder:
          config
            .externalOrAltEncoderPosition(kOptimizedFrameRateMs)
            .externalOrAltEncoderVelocity(kOptimizedFrameRateMs);
          break;
        case kAnalogSensor:
          config
            .analogPositionPeriodMs(kOptimizedFrameRateMs)
            .analogVelocityPeriodMs(kOptimizedFrameRateMs)
            .analogVoltagePeriodMs(kOptimizedFrameRateMs);
          break;
        case kPosition:
          config
            .primaryEncoderVelocityPeriodMs(kOptimizedFrameRateMs);
          break;
        case kVelocity:
          config
            .primaryEncoderVelocityPeriodMs(kOptimizedFrameRateMs);
          break;
      }
    }
    return config;
  }
}
