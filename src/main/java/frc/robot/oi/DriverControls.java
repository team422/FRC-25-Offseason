package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public Trigger resetFieldCentric();

  public Trigger autoAim();

  public Trigger autoShoot();

  public Trigger defaultShoot();

  public Trigger homing();
}
