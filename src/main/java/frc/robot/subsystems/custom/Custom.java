package frc.robot.subsystems.custom;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.CustomConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Custom extends SubsystemBase {
  private CustomIO m_io;
  private MotorInputsAutoLogged m_inputs;

  public enum CustomState {
    kIdle,
    kVoltageControl,
    kVelocityControl
  }

  private SubsystemProfiles<CustomState> m_profiles;

  public Custom(CustomIO io) {
    m_io = io;

    m_inputs = new MotorInputsAutoLogged();

    HashMap<CustomState, Runnable> hash = new HashMap<>();
    hash.put(CustomState.kIdle, this::idlePeriodic);
    hash.put(CustomState.kVelocityControl, this::velocityControlPeriodic);
    hash.put(CustomState.kVoltageControl, this::voltageControlPeriodic);
    m_profiles = new SubsystemProfiles<Custom.CustomState>(hash, CustomState.kIdle);

    if (RobotBase.isReal()) {
      m_io.setPID(
          CustomConstants.kP.get(),
          CustomConstants.kI.get(),
          CustomConstants.kD.get(),
          CustomConstants.kKV.get());
    } else {
      m_io.setPID(
          CustomConstants.kSimP.get(),
          CustomConstants.kSimI.get(),
          CustomConstants.kSimD.get(),
          CustomConstants.kSimKV.get());
    }
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_io.setPID(
                CustomConstants.kP.get(),
                CustomConstants.kI.get(),
                CustomConstants.kD.get(),
                CustomConstants.kKV.get());
          }
        },
        CustomConstants.kP,
        CustomConstants.kI,
        CustomConstants.kD,
        CustomConstants.kKV);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isSimulation()) {
            m_io.setPID(
                CustomConstants.kSimP.get(),
                CustomConstants.kSimI.get(),
                CustomConstants.kSimD.get(),
                CustomConstants.kSimKV.get());
          }
        },
        CustomConstants.kSimP,
        CustomConstants.kSimI,
        CustomConstants.kSimD,
        CustomConstants.kSimKV);

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Custom", m_inputs);
    Logger.recordOutput("Custom/state", m_profiles.getCurrentProfile());

    m_profiles.getPeriodicFunctionTimed().run();
  }

  public void idlePeriodic() {
    m_io.setVoltage(CustomConstants.kIdleVoltage);
  }

  public void velocityControlPeriodic() {
    m_io.setVelocity(CustomConstants.kVelocityRPS.getAsDouble());
  }

  public void voltageControlPeriodic() {
    m_io.setVoltage(CustomConstants.kVoltage.getAsDouble());
  }

  public void updateState(CustomState state) {
    m_profiles.setCurrentProfile(state);
  }

  public CustomState getState() {
    return m_profiles.getCurrentProfile();
  }
}
