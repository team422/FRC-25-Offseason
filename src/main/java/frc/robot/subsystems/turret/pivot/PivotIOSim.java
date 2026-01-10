package frc.robot.subsystems.turret.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants.PivotConstants;

public class PivotIOSim implements PivotIO {
  private DCMotorSim m_sim;
  private PIDController m_controller = new PIDController(0, 0, 0);

  private boolean m_positionControl = true;
  private double m_voltage = 0.0;

  public PivotIOSim() {
    m_sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                PivotConstants.kSimGearbox, PivotConstants.kSimMOI, PivotConstants.kGearRatio),
            PivotConstants.kSimGearbox);
    m_controller.setTolerance(PivotConstants.kTolerance);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    if (m_positionControl) {
      m_voltage = m_controller.calculate(m_sim.getAngularPosition().in(Degrees));
    }
    
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.connected = true;
    inputs.position = m_sim.getAngularPosition().in(Degrees);
    inputs.desired = m_controller.getSetpoint();
    inputs.statorCurrent = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;
    inputs.velocity = m_sim.getAngularVelocity().in(DegreesPerSecond);
    inputs.atSetpoint = m_controller.atSetpoint();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    m_positionControl = true;
    m_controller.setSetpoint(angle.getDegrees());
  }

  @Override
  public void setPID(int slot, double p, double i, double d) {
    m_controller.setPID(p, i, d);
  }

  @Override
  public void setVoltage(double volts) {
    m_positionControl = false;
    m_voltage = volts;
  }

  @Override
  public void zero() {
    m_sim.setAngle(Rotation2d.fromDegrees(PivotConstants.kMinNegativeAngle).getRadians());
  }
}
