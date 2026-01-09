package frc.robot.subsystems.custom;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CustomConstants;

public class CustomIOSim implements CustomIO {
  private DCMotorSim m_sim;
  private PIDController m_controller = new PIDController(0, 0, 0);
  private double m_voltage = 0;
  private double m_KV = 0.0;

  public CustomIOSim() {
    m_sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                CustomConstants.kSimGearbox, CustomConstants.kSimMOI, CustomConstants.kGearRatio),
            CustomConstants.kSimGearbox);

    m_controller.setTolerance(CustomConstants.kTolerance);
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.connected = true;
    inputs.desired = m_controller.getSetpoint();
    inputs.statorCurrent = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;
    inputs.velocity = m_sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.atSetpoint = m_controller.atSetpoint();
  }

  @Override
  public void setVoltage(double volts) {
    m_voltage = volts;
    m_controller.setSetpoint(0);
  }

  @Override
  public void setVelocity(double speedRPS) {
    m_controller.setSetpoint(speedRPS);

    m_voltage =
        m_controller.calculate(m_sim.getAngularVelocityRPM() / 60)
            + m_controller.getSetpoint() * m_KV;
  }

  @Override
  public void setPID(double p, double i, double d, double kV) {
    m_controller.setPID(p, i, d);
    m_KV = kV;
  }
}
