package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public class HoodInputs {
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

  public void updateInputs(HoodInputs inputs);

  public void setPosition(Rotation2d angle);

  public void setPID(int slot, double p, double i, double d);
}
