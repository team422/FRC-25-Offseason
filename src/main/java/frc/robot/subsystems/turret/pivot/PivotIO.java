package frc.robot.subsystems.turret.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public class PivotInputs {
    public boolean atSetpoint;
    public double position;
    public double desired;
    public double voltage;
    public double velocity;
    public boolean connected;
    public double supplyCurrent;
    public double statorCurrent;
    public double temperature;
  }

  public void updateInputs(PivotInputs inputs);

  public void setAngle(Rotation2d angle);

  public void setPID(int slot, double p, double i, double d);

  public void setVoltage(double volts);

  public void zero();
}
