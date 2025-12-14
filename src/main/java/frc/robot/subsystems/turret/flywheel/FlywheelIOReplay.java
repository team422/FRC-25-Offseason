package frc.robot.subsystems.turret.flywheel;

public class FlywheelIOReplay implements FlywheelIO {

  @Override
  public void updateInputs(FlywheelInputs inputs) {}

  @Override
  public void setVelocity(double topRPS, double bottomRPS) {}

  @Override
  public void setTopPID(int slot, double p, double i, double d) {}

  @Override
  public void setBottomPID(int slot, double p, double i, double d) {}
}
