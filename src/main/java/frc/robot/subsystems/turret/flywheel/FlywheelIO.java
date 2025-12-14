package frc.robot.subsystems.turret.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelInputs {
    public boolean atTopSetpoint;
    public double topVelocityRPS;
    public double topDesiredRPS;
    public double topVoltage;
    public boolean topConnected;
    public double topSupplyCurrent;
    public double topStatorCurrent;
    public double topTemperature;
    public boolean atBottomSetpoint;
    public double bottomVelocityRPS;
    public double bottomDesiredRPS;
    public double bottomVoltage;
    public boolean bottomConnected;
    public double bottomSupplyCurrent;
    public double bottomStatorCurrent;
    public double bottomTemperature;
  }

  public void updateInputs(FlywheelInputs inputs);

  public void setVelocity(double topRPS, double bottomRPS);

  public void setTopPID(int slot, double p, double i, double d);

  public void setBottomPID(int slot, double p, double i, double d);
}
