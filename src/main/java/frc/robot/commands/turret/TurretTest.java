package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class TurretTest extends Command {
  private final Turret turret;
  private final XboxController xbox;

  private final Timer timer = new Timer();
  
  private double pitch = TurretConstants.kMinAngleRad;

  public TurretTest(Turret turret, XboxController xbox) {
    this.turret = turret;
    this.xbox = xbox;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (xbox.getPOV() == 0) {
      pitch = pitch > TurretConstants.kMaxAngleRad ? pitch : pitch + 0.001;
    } else if (xbox.getPOV() == 180) {
      pitch = pitch < TurretConstants.kMinAngleRad ? pitch : pitch - 0.001;
    }

    if (xbox.getXButton()) {
      turret.setVelocity(TurretConstants.kMaxSpeedRadPerSec, TurretConstants.kMaxSpeedRadPerSec * 0.8);
    } else {
      turret.setVelocity(0);
    }

    if (xbox.getRightBumperButton()) {
      timer.restart();
    }

    if (timer.isRunning()) {
      if (timer.get() < 0.2) {
        turret.setTrigger(12);
      } else {
        turret.setTrigger(0);
      }
    }

    if (xbox.getYButtonPressed()) {
      System.out.println("pitch: " + pitch);
    }

    turret.setPitch(pitch);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0);
    turret.setTrigger(0);
  }

  @Override
  public boolean isFinished() {
    return xbox.getPOV() == 270;
  }
}
