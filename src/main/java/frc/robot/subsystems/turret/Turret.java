package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.TurretConstants.FlywheelConstants;
import frc.robot.Constants.TurretConstants.HoodConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;
import frc.robot.subsystems.turret.flywheel.FlywheelIO;
import frc.robot.subsystems.turret.flywheel.FlywheelInputsAutoLogged;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.hood.HoodInputsAutoLogged;
import frc.robot.subsystems.turret.pivot.PivotIO;
import frc.robot.subsystems.turret.pivot.PivotInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private PivotIO m_pivot;
  private FlywheelIO m_flywheel;
  private HoodIO m_hood;
  private PivotInputsAutoLogged m_pivotInputs;
  private FlywheelInputsAutoLogged m_flywheelInputs;
  private HoodInputsAutoLogged m_hoodInputs;

  private SubsystemProfiles<TurretState> m_profiles;

  private Rotation2d m_desiredPivotPosition = new Rotation2d();
  private Rotation2d m_desiredHoodPosition = new Rotation2d();

  private double m_desiredTopSpeed = 0;
  private double m_desiredBottomSpeed = 0;

  public enum TurretState {
    kIdle,
    kAiming,
    kAutoShooting,
    kDefaultShooting,
  }

  public Turret(PivotIO pivot, FlywheelIO flywheel, HoodIO hood) {
    m_pivot = pivot;
    m_flywheel = flywheel;
    m_hood = hood;

    m_pivotInputs = new PivotInputsAutoLogged();
    m_flywheelInputs = new FlywheelInputsAutoLogged();
    m_hoodInputs = new HoodInputsAutoLogged();

    HashMap<TurretState, Runnable> map = new HashMap<>();
    map.put(TurretState.kIdle, this::idlePeriodic);
    map.put(TurretState.kDefaultShooting, this::defaultShootingPeriodic);
    map.put(TurretState.kAiming, this::aimingPeriodic);
    map.put(TurretState.kAutoShooting, this::shootingPeriodic);

    m_profiles = new SubsystemProfiles<Turret.TurretState>(map, TurretState.kIdle);

    if (RobotBase.isReal()) {
      m_flywheel.setTopPID(
          0,
          FlywheelConstants.kTopP.getAsDouble(),
          FlywheelConstants.kTopI.getAsDouble(),
          FlywheelConstants.kTopD.getAsDouble());
      m_flywheel.setBottomPID(
          0,
          FlywheelConstants.kBottomP.getAsDouble(),
          FlywheelConstants.kBottomI.getAsDouble(),
          FlywheelConstants.kBottomD.getAsDouble());
      m_pivot.setPID(
          0,
          PivotConstants.kP.getAsDouble(),
          PivotConstants.kI.getAsDouble(),
          PivotConstants.kD.getAsDouble());
    } else {
      m_flywheel.setTopPID(
          0,
          FlywheelConstants.kSimTopP.getAsDouble(),
          FlywheelConstants.kSimTopI.getAsDouble(),
          FlywheelConstants.kSimTopD.getAsDouble());
      m_flywheel.setBottomPID(
          0,
          FlywheelConstants.kSimBottomP.getAsDouble(),
          FlywheelConstants.kSimBottomI.getAsDouble(),
          FlywheelConstants.kSimBottomD.getAsDouble());
      m_pivot.setPID(
          0,
          PivotConstants.kSimP.getAsDouble(),
          PivotConstants.kSimI.getAsDouble(),
          PivotConstants.kSimD.getAsDouble());
    }
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_flywheel.setTopPID(
                0,
                FlywheelConstants.kTopP.getAsDouble(),
                FlywheelConstants.kTopI.getAsDouble(),
                FlywheelConstants.kTopD.getAsDouble());
          }
        },
        FlywheelConstants.kTopP,
        FlywheelConstants.kTopI,
        FlywheelConstants.kTopD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_flywheel.setBottomPID(
                0,
                FlywheelConstants.kBottomP.getAsDouble(),
                FlywheelConstants.kBottomI.getAsDouble(),
                FlywheelConstants.kBottomD.getAsDouble());
          }
        },
        FlywheelConstants.kBottomP,
        FlywheelConstants.kBottomI,
        FlywheelConstants.kBottomD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_pivot.setPID(
                0,
                PivotConstants.kP.getAsDouble(),
                PivotConstants.kI.getAsDouble(),
                PivotConstants.kD.getAsDouble());
          }
        },
        PivotConstants.kP,
        PivotConstants.kI,
        PivotConstants.kD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (!RobotBase.isReal()) {
            m_flywheel.setTopPID(
                0,
                FlywheelConstants.kSimTopP.getAsDouble(),
                FlywheelConstants.kSimTopI.getAsDouble(),
                FlywheelConstants.kSimTopD.getAsDouble());
          }
        },
        FlywheelConstants.kSimTopP,
        FlywheelConstants.kSimTopI,
        FlywheelConstants.kSimTopD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (!RobotBase.isReal()) {
            m_flywheel.setBottomPID(
                0,
                FlywheelConstants.kSimBottomP.getAsDouble(),
                FlywheelConstants.kSimBottomI.getAsDouble(),
                FlywheelConstants.kSimBottomD.getAsDouble());
          }
        },
        FlywheelConstants.kSimBottomP,
        FlywheelConstants.kSimBottomI,
        FlywheelConstants.kSimBottomD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (!RobotBase.isReal()) {
            m_pivot.setPID(
                0,
                PivotConstants.kSimP.getAsDouble(),
                PivotConstants.kSimI.getAsDouble(),
                PivotConstants.kSimD.getAsDouble());
          }
        },
        PivotConstants.kSimP,
        PivotConstants.kSimI,
        PivotConstants.kSimD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_hood.setPID(
                0, HoodConstants.kP.get(), HoodConstants.kI.get(), HoodConstants.kD.get());
          }
        },
        HoodConstants.kP,
        HoodConstants.kI,
        HoodConstants.kD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (!RobotBase.isReal()) {
            m_hood.setPID(
                0, HoodConstants.kSimP.get(), HoodConstants.kSimI.get(), HoodConstants.kSimD.get());
          }
        },
        HoodConstants.kSimP,
        HoodConstants.kSimI,
        HoodConstants.kSimD);

    Logger.processInputs("Turret/Flywheel", m_flywheelInputs);
    Logger.processInputs("Turret/Pivot", m_pivotInputs);
    Logger.processInputs("Turret/Hood", m_hoodInputs);
    Logger.recordOutput("Turret/state", m_profiles.getCurrentProfile());

    m_flywheel.updateInputs(m_flywheelInputs);
    m_pivot.updateInputs(m_pivotInputs);
    m_hood.updateInputs(m_hoodInputs);

    m_profiles.getPeriodicFunctionTimed().run();
  }

  public void idlePeriodic() {
    m_desiredTopSpeed = 0;
    m_desiredBottomSpeed = 0;
    m_flywheel.setVelocity(m_desiredTopSpeed, m_desiredBottomSpeed);
  }

  public void aimingPeriodic() {
    autoAim();
    m_flywheel.setVelocity(0, 0);
  }

  public void shootingPeriodic() {
    autoAim();
    m_flywheel.setVelocity(m_desiredTopSpeed, m_desiredBottomSpeed);
  }

  public void defaultShootingPeriodic() {
    m_pivot.setAngle(new Rotation2d());
    m_flywheel.setVelocity(
        FlywheelConstants.kTopDefaultShootingSpeed.getAsDouble(),
        FlywheelConstants.kBottomDefaultShootingSpeed.getAsDouble());
    m_hood.setPosition(Rotation2d.fromRotations(HoodConstants.kDefaultShooting.get()));
  }

  public void autoAim() {
    m_pivot.setAngle(m_desiredPivotPosition);
    m_hood.setPosition(m_desiredHoodPosition);
  }

  public void updateState(TurretState state) {
    m_profiles.setCurrentProfile(state);
  }

  public void setPivotPosition(Rotation2d angle) {
    m_desiredPivotPosition = angle;
  }

  public void setHoodPosition(Rotation2d position) {
    m_desiredHoodPosition = position;
  }

  public void setSpeed(double topSpeedRPS, double bottomSpeedRPS) {
    m_desiredTopSpeed = topSpeedRPS;
    m_desiredBottomSpeed = bottomSpeedRPS;
  }

  public Rotation2d getPivotPosition() {
    return Rotation2d.fromDegrees(m_pivotInputs.position);
  }

  public Rotation2d getHoodPosition() {
    return Rotation2d.fromDegrees(m_hoodInputs.position);
  }

  public TurretState getState() {
    return m_profiles.getCurrentProfile();
  }
}
