package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.TurretConstants.HoodConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;

public class HoodIOSim implements HoodIO {
  private SingleJointedArmSim m_sim;
  private PIDController m_controller;

  public HoodIOSim() {
    m_sim =
        new SingleJointedArmSim(
            HoodConstants.kSimGearbox,
            HoodConstants.kGearRatio,
            HoodConstants.kSimMOI,
            HoodConstants.kArmLength,
            HoodConstants.kMinAngle,
            HoodConstants.kMaxAngle,
            false,
            HoodConstants.kMinAngle);
    m_controller = new PIDController(0, 0, 0);
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    double currPosition = Rotation2d.fromRadians(m_sim.getAngleRads()).getDegrees();
    double voltage = m_controller.calculate(currPosition);

    m_sim.setInputVoltage(voltage);
    m_sim.update(.02);

    inputs.connected = true;
    inputs.position = currPosition;
    inputs.desired = m_controller.getSetpoint();
    inputs.statorCurrent = m_sim.getCurrentDrawAmps();
    inputs.voltage = voltage;
    inputs.velocity = Units.radiansPerSecondToRotationsPerMinute(m_sim.getVelocityRadPerSec()) / 60;
    inputs.atSetpoint = Math.abs(inputs.position - inputs.desired) < PivotConstants.kTolerance;
  }

  @Override
  public void setPosition(Rotation2d angle) {
    m_controller.setSetpoint(angle.getDegrees());
  }

  @Override
  public void setPID(int slot, double p, double i, double d) {
    m_controller.setPID(p, i, d);
  }
}
