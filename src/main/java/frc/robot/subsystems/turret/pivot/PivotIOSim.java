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
    double voltage = m_controller.calculate(m_sim.getAngularPosition().in(Degrees));
    m_sim.setInputVoltage(voltage);
    m_sim.update(.02);

    inputs.connected = true;
    inputs.position = m_sim.getAngularPosition().in(Degrees);
    inputs.desired = m_controller.getSetpoint();
    inputs.statorCurrent = m_sim.getCurrentDrawAmps();
    inputs.voltage = voltage;
    inputs.velocity = m_sim.getAngularVelocity().in(DegreesPerSecond);
    inputs.atSetpoint = m_controller.atSetpoint();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    m_controller.setSetpoint(angle.getDegrees());
  }

  @Override
  public void setPID(int slot, double p, double i, double d) {
    m_controller.setPID(p, i, d);
  }
}
