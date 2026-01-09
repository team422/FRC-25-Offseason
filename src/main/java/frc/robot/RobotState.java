package frc.robot;

import frc.robot.subsystems.custom.Custom;
import frc.robot.subsystems.custom.Custom.CustomState;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private Custom m_custom;

  private static RobotState m_instance;
  private SubsystemProfiles<RobotAction> m_profiles;

  public enum RobotAction {
    kDefault,
    kVoltageControl,
    kVelocityControl
  }

  public static void startInstance(Custom custom) {
    m_instance = new RobotState(custom);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  private RobotState(Custom custom) {
    m_custom = custom;

    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kDefault, () -> {});
    hash.put(RobotAction.kVoltageControl, () -> {});
    hash.put(RobotAction.kVelocityControl, () -> {});
    m_profiles = new SubsystemProfiles<RobotState.RobotAction>(hash, RobotAction.kDefault);
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void updateRobotAction(RobotAction action) {
    CustomState newState = CustomState.kIdle;
    switch (action) {
      case kDefault:
        break;
      case kVelocityControl:
        newState = CustomState.kVelocityControl;
        break;
      case kVoltageControl:
        newState = CustomState.kVoltageControl;
        break;
      default:
        break;
    }

    if (newState != m_custom.getState()) {
      m_custom.updateState(newState);
    }

    m_profiles.setCurrentProfile(action);
  }

  public RobotAction getAction() {
    return m_profiles.getCurrentProfile();
  }
}
