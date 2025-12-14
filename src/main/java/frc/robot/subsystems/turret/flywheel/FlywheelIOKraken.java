package frc.robot.subsystems.turret.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.TurretConstants.FlywheelConstants;

public class FlywheelIOKraken implements FlywheelIO {
  private TalonFX m_topMotor;
  private TalonFX m_bottomMotor;

  private VelocityVoltage m_voltage = new VelocityVoltage(0).withEnableFOC(true);
  private double m_topDesiredRPS;
  private double m_bottomDesiredRPS;

  private TalonFXConfiguration m_configs;

  private StatusSignal<Voltage> m_topVoltageSignal;
  private StatusSignal<AngularVelocity> m_topVelocity;
  private StatusSignal<ConnectedMotorValue> m_topConnected;
  private StatusSignal<Current> m_topSupplyCurrent;
  private StatusSignal<Current> m_topStatorCurrent;
  private StatusSignal<Temperature> m_topTemperature;
  private StatusSignal<Voltage> m_bottomVoltageSignal;
  private StatusSignal<AngularVelocity> m_bottomVelocity;
  private StatusSignal<ConnectedMotorValue> m_bottomConnected;
  private StatusSignal<Current> m_bottomSupplyCurrent;
  private StatusSignal<Current> m_bottomStatorCurrent;
  private StatusSignal<Temperature> m_bottomTemperature;

  public FlywheelIOKraken(int top, int bottom, String bus) {
    m_topMotor = new TalonFX(top, bus);
    m_bottomMotor = new TalonFX(bottom, bus);

    m_topDesiredRPS = 0;
    m_bottomDesiredRPS = 0;

    var current =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kFlywheelDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kFlywheelDefaultStatorLimit);

    var output = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);

    var feedback = new FeedbackConfigs().withSensorToMechanismRatio(FlywheelConstants.kGearRatio);

    m_configs =
        new TalonFXConfiguration()
            .withCurrentLimits(current)
            .withMotorOutput(output)
            .withFeedback(feedback);

    m_topMotor.getConfigurator().apply(m_configs);
    m_bottomMotor
        .getConfigurator()
        .apply(m_configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    m_topVoltageSignal = m_topMotor.getMotorVoltage();
    m_topVelocity = m_topMotor.getRotorVelocity();
    m_topConnected = m_topMotor.getConnectedMotor();
    m_topSupplyCurrent = m_topMotor.getSupplyCurrent();
    m_topStatorCurrent = m_topMotor.getStatorCurrent();
    m_topTemperature = m_topMotor.getDeviceTemp();

    m_bottomVoltageSignal = m_bottomMotor.getMotorVoltage();
    m_bottomVelocity = m_bottomMotor.getRotorVelocity();
    m_bottomConnected = m_bottomMotor.getConnectedMotor();
    m_bottomSupplyCurrent = m_bottomMotor.getSupplyCurrent();
    m_bottomStatorCurrent = m_bottomMotor.getStatorCurrent();
    m_bottomTemperature = m_bottomMotor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        100,
        m_topVoltageSignal,
        m_topVelocity,
        m_topConnected,
        m_topSupplyCurrent,
        m_topStatorCurrent,
        m_topTemperature,
        m_bottomVoltageSignal,
        m_bottomVelocity,
        m_bottomConnected,
        m_bottomSupplyCurrent,
        m_bottomStatorCurrent,
        m_bottomTemperature);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.topVoltage = m_topVoltageSignal.getValue().in(Volts);
    inputs.topVelocityRPS = m_topVelocity.getValue().in(RotationsPerSecond);
    inputs.topConnected = m_topConnected.getValue() != ConnectedMotorValue.Unknown;
    inputs.topSupplyCurrent = m_topSupplyCurrent.getValue().in(Amps);
    inputs.topStatorCurrent = m_topStatorCurrent.getValue().in(Amps);
    inputs.topTemperature = m_topTemperature.getValue().in(Fahrenheit);
    inputs.topDesiredRPS = m_topDesiredRPS;
    inputs.atTopSetpoint =
        Math.abs(inputs.topVelocityRPS - inputs.topDesiredRPS) < FlywheelConstants.kTolerance;

    inputs.bottomVoltage = m_bottomVoltageSignal.getValue().in(Volts);
    inputs.bottomVelocityRPS = m_bottomVelocity.getValue().in(RotationsPerSecond);
    inputs.bottomConnected = m_bottomConnected.getValue() != ConnectedMotorValue.Unknown;
    inputs.bottomSupplyCurrent = m_bottomSupplyCurrent.getValue().in(Amps);
    inputs.bottomStatorCurrent = m_bottomStatorCurrent.getValue().in(Amps);
    inputs.bottomTemperature = m_bottomTemperature.getValue().in(Fahrenheit);
    inputs.bottomDesiredRPS = m_bottomDesiredRPS;
    inputs.atBottomSetpoint =
        Math.abs(inputs.bottomDesiredRPS - inputs.bottomVelocityRPS) < FlywheelConstants.kTolerance;
  }

  @Override
  public void setVelocity(double topRPS, double bottomRPS) {
    m_topMotor.setControl(m_voltage.withVelocity(topRPS));
    m_bottomMotor.setControl(m_voltage.withVelocity(bottomRPS));
    m_topDesiredRPS = topRPS;
    m_bottomDesiredRPS = bottomRPS;
  }

  @Override
  public void setTopPID(int slot, double p, double i, double d) {
    var configs = new SlotConfigs().withKP(p).withKI(i).withKD(d);
    configs.SlotNumber = slot;

    m_topMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setBottomPID(int slot, double p, double i, double d) {
    var configs = new SlotConfigs().withKP(p).withKI(i).withKD(d);
    configs.SlotNumber = slot;

    m_bottomMotor.getConfigurator().apply(configs);
  }
}
