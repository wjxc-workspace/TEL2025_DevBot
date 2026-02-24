package frc.robot.commands.storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.storage.Stroage;

public class StorageCommands {
  public static Command reload(Stroage stroage) {
    return new Reload(stroage);
  }
}
