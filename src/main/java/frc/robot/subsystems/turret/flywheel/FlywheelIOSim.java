package frc.robot.subsystems.turret.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  private DCMotorSim m_topSim;
  private DCMotorSim m_bottomSim;
  private PIDController m_topController = new PIDController(0, 0, 0);
  private PIDController m_bottomController = new PIDController(0, 0, 0);

  public FlywheelIOSim() {
    m_topSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FlywheelConstants.kSimGearbox,
                FlywheelConstants.kSimMOI,
                FlywheelConstants.kGearRatio),
            FlywheelConstants.kSimGearbox);
    m_bottomSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FlywheelConstants.kSimGearbox,
                FlywheelConstants.kSimMOI,
                FlywheelConstants.kGearRatio),
            FlywheelConstants.kSimGearbox);
    m_topController.setTolerance(FlywheelConstants.kTolerance);
    m_bottomController.setTolerance(FlywheelConstants.kTolerance);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    double topVoltage =
        m_topController.calculate(m_topSim.getAngularVelocityRPM() / 60)
            + FlywheelConstants.kSimKV.getAsDouble() * m_topController.getSetpoint();
    double bottomVoltage =
        m_bottomController.calculate(m_bottomSim.getAngularVelocityRPM() / 60)
            + FlywheelConstants.kSimKV.getAsDouble() * m_bottomController.getSetpoint();

    m_topSim.setInputVoltage(topVoltage);
    m_bottomSim.setInputVoltage(bottomVoltage);

    m_topSim.update(.02);
    m_bottomSim.update(.02);

    inputs.topVelocityRPS = m_topSim.getAngularVelocityRPM() / 60;
    inputs.topDesiredRPS = m_topController.getSetpoint();
    inputs.topVoltage = topVoltage;
    inputs.topConnected = true;
    inputs.topSupplyCurrent = m_topSim.getCurrentDrawAmps();

    inputs.bottomVelocityRPS = m_bottomSim.getAngularVelocityRPM() / 60;
    inputs.bottomDesiredRPS = m_bottomController.getSetpoint();
    inputs.bottomVoltage = bottomVoltage;
    inputs.bottomConnected = true;
    inputs.bottomSupplyCurrent = m_bottomSim.getCurrentDrawAmps();

    inputs.atTopSetpoint = m_topController.atSetpoint();
    inputs.atBottomSetpoint = m_bottomController.atSetpoint();
  }

  @Override
  public void setVelocity(double topRPS, double bottomRPS) {
    m_topController.setSetpoint(topRPS);
    m_bottomController.setSetpoint(bottomRPS);
  }

  @Override
  public void setTopPID(int slot, double p, double i, double d) {
    m_topController.setPID(p, i, d);
  }

  @Override
  public void setBottomPID(int slot, double p, double i, double d) {
    m_bottomController.setPID(p, i, d);
  }
}
