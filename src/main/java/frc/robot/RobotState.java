package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.util.SetpointGenerator;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private Drive m_drive;
  private Turret m_turret;
  private static RobotState m_instance;
  private SubsystemProfiles<RobotAction> m_profiles;

  public enum RobotAction {
    kDefault,
    kAutoAim,
    kAutoShooting,
    kDefaultShooting
  }

  public static void startInstance(Drive drive, Turret turret) {
    m_instance = new RobotState(drive, turret);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  private RobotState(Drive drive, Turret turret) {
    m_drive = drive;
    m_turret = turret;

    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kDefault, () -> {});
    hash.put(RobotAction.kDefaultShooting, () -> {});
    hash.put(RobotAction.kAutoAim, this::autoAimingPeriodic);
    hash.put(RobotAction.kAutoShooting, this::shootingPeriodic);

    m_profiles = new SubsystemProfiles<RobotState.RobotAction>(hash, RobotAction.kDefault);
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void autoAimingPeriodic() {
    m_turret.setPivotPosition(SetpointGenerator.getShooterPosition(m_drive));
    m_turret.setHoodPosition(SetpointGenerator.getHoodPosition(m_drive));
  }

  public void shootingPeriodic() {
    autoAimingPeriodic();

    var speeds = SetpointGenerator.getShooterSpeeds(m_drive);
    m_turret.setSpeed(speeds.getFirst(), speeds.getSecond());
  }

  public void updateRobotAction(RobotAction action) {
    DriveProfiles newDriveState = DriveProfiles.kDefault;
    TurretState newTurretState = TurretState.kIdle;

    switch (action) {
      case kDefault:
        break;
      case kAutoShooting:
        newTurretState = TurretState.kAutoShooting;
        break;
      case kAutoAim:
        newTurretState = TurretState.kAiming;
        break;
      case kDefaultShooting:
        newTurretState = TurretState.kDefaultShooting;
      default:
        break;
    }

    if (newDriveState != m_drive.getCurrentProfile()) {
      m_drive.updateProfile(newDriveState);
    }

    if (newTurretState != m_turret.getState()) {
      m_turret.updateState(newTurretState);
    }

    m_profiles.setCurrentProfile(action);
  }

  public RobotAction getAction() {
    return m_profiles.getCurrentProfile();
  }
}
