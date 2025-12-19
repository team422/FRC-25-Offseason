package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public enum RobotAction {
    kAutoDefault,
    kTeleopDefault
  }

  private Drive m_drive;
  private SubsystemProfiles<RobotAction> m_profiles;
  private static RobotState m_instance;

  public RobotState(Drive drive) {
    m_drive = drive;

    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kAutoDefault, () -> {});
    hash.put(RobotAction.kTeleopDefault, () -> {});

    m_profiles = new SubsystemProfiles<>(hash, RobotAction.kTeleopDefault);
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void updateRobotAction(RobotAction action) {
    DriveProfiles newDriveState = DriveProfiles.kDefault;

    switch (action) {
      case kAutoDefault:
        newDriveState = DriveProfiles.kAutoAlign;
        break;
      case kTeleopDefault:
        newDriveState = DriveProfiles.kDefault;
        break;
      default:
        break;
    }

    if (newDriveState != m_drive.getCurrentProfile()) {
      m_drive.updateProfile(newDriveState);
    }

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(Drive drive) {
    if (m_instance == null) {
      m_instance = new RobotState(drive);
    }
    return m_instance;
  }

  public RobotAction getCurrAction() {
    return m_profiles.getCurrentProfile();
  }
}
