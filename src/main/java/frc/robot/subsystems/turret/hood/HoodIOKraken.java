package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.TurretConstants.HoodConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;

public class HoodIOKraken implements HoodIO {
  private TalonFX m_motor;
  private PositionVoltage m_voltage = new PositionVoltage(0).withEnableFOC(true);
  private TalonFXConfiguration m_configs;
  private Rotation2d m_desired = new Rotation2d();

  private StatusSignal<Angle> m_position;
  private StatusSignal<Voltage> m_voltageSignal;
  private StatusSignal<AngularVelocity> m_velocity;
  private StatusSignal<ConnectedMotorValue> m_connected;
  private StatusSignal<Current> m_supplyCurrent;
  private StatusSignal<Current> m_statorCurrent;
  private StatusSignal<Temperature> m_temperature;

  public HoodIOKraken(int port, String bus) {
    m_motor = new TalonFX(port, bus);

    var current =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kHoodDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kHoodDefaultStatorCurrent);
    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    var feedback = new FeedbackConfigs().withSensorToMechanismRatio(HoodConstants.kGearRatio);
    m_configs =
        new TalonFXConfiguration()
            .withCurrentLimits(current)
            .withMotorOutput(motorOutput)
            .withFeedback(feedback);
    m_motor.getConfigurator().apply(m_configs);

    m_position = m_motor.getPosition();
    m_voltageSignal = m_motor.getMotorVoltage();
    m_velocity = m_motor.getVelocity();
    m_connected = m_motor.getConnectedMotor();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_temperature = m_motor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        100,
        m_position,
        m_voltageSignal,
        m_velocity,
        m_connected,
        m_supplyCurrent,
        m_statorCurrent,
        m_temperature);
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    inputs.connected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    inputs.desired = m_desired.getDegrees();
    inputs.position = m_position.getValue().in(Degrees);
    inputs.statorCurrent = m_statorCurrent.getValue().in(Amps);
    inputs.supplyCurrent = m_supplyCurrent.getValue().in(Amps);
    inputs.temperature = m_temperature.getValue().in(Fahrenheit);
    inputs.velocity = m_velocity.getValue().in(RotationsPerSecond);
    inputs.voltage = m_voltageSignal.getValue().in(Volts);
    inputs.atSetpoint = Math.abs(inputs.position - inputs.desired) < PivotConstants.kTolerance;
  }

  @Override
  public void setPosition(Rotation2d angle) {
    m_desired = angle;
    m_motor.setControl(m_voltage.withPosition(angle.getRadians()));
  }

  @Override
  public void setPID(int slot, double p, double i, double d) {
    var configs = new SlotConfigs().withKP(p).withKI(i).withKD(d);
    configs.SlotNumber = slot;

    m_motor.getConfigurator().apply(configs);
  }
}
