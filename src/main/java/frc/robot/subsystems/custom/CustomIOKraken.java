package frc.robot.subsystems.custom;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.CustomConstants;

public class CustomIOKraken implements CustomIO {
  private TalonFX m_motor;
  private VelocityVoltage m_voltage = new VelocityVoltage(0).withEnableFOC(false);

  private TalonFXConfiguration m_configs;
  private double m_desired = 0;

  private StatusSignal<Voltage> m_voltageSignal;
  private StatusSignal<AngularVelocity> m_velocity;
  private StatusSignal<ConnectedMotorValue> m_connected;
  private StatusSignal<Current> m_supplyCurrent;
  private StatusSignal<Current> m_statorCurrent;
  private StatusSignal<Temperature> m_temperature;

  public CustomIOKraken(int port, String bus) {
    m_motor = new TalonFX(port, bus);

    var current =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kMotorDefaultStatorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kMotorDefaultSupplyCurrentLimit);
    var output = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    var feedback = new FeedbackConfigs().withSensorToMechanismRatio(CustomConstants.kGearRatio);

    m_configs =
        new TalonFXConfiguration()
            .withCurrentLimits(current)
            .withMotorOutput(output)
            .withFeedback(feedback);

    m_motor.getConfigurator().apply(m_configs);

    m_voltageSignal = m_motor.getMotorVoltage();
    m_velocity = m_motor.getVelocity();
    m_connected = m_motor.getConnectedMotor();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_temperature = m_motor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        100,
        m_voltageSignal,
        m_velocity,
        m_connected,
        m_supplyCurrent,
        m_statorCurrent,
        m_temperature);
  }

  @Override
  public void updateInputs(MotorInputs inputs) {

    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_voltageSignal,
          m_velocity,
          m_connected,
          m_supplyCurrent,
          m_statorCurrent,
          m_temperature);
    }
    inputs.connected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    inputs.desired = m_desired;
    inputs.statorCurrent = m_statorCurrent.getValue().in(Amps);
    inputs.supplyCurrent = m_supplyCurrent.getValue().in(Amps);
    inputs.temperature = m_temperature.getValue().in(Fahrenheit);
    inputs.velocity = m_velocity.getValue().in(RotationsPerSecond);
    inputs.voltage = m_voltageSignal.getValue().in(Volts);
    inputs.atSetpoint = Math.abs(inputs.velocity - inputs.desired) < CustomConstants.kTolerance;
  }

  @Override
  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
    m_desired = 0;
  }

  @Override
  public void setVelocity(double speedRPS) {
    m_motor.setControl(m_voltage.withVelocity(speedRPS));
    m_desired = speedRPS;
  }

  @Override
  public void setPID(double p, double i, double d, double kV) {
    m_motor.getConfigurator().apply(new SlotConfigs().withKP(p).withKI(i).withKD(d).withKV(kV));
  }
}
