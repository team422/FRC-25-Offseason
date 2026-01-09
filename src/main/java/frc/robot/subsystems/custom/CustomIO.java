package frc.robot.subsystems.custom;

import org.littletonrobotics.junction.AutoLog;

public interface CustomIO {
  @AutoLog
  public class MotorInputs {
    public boolean atSetpoint;
    public double desired;
    public double voltage;
    public double velocity;
    public boolean connected;
    public double supplyCurrent;
    public double statorCurrent;
    public double temperature;
  }

  public void updateInputs(MotorInputs inputs);

  public void setVoltage(double volts);

  public void setVelocity(double speedRPS);

  public void setPID(double p, double i, double d, double kV);
}
