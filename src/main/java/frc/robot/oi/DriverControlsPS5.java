package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public Trigger velocityControl() {
    return m_controller.circle();
    // return m_controller.R2();
  }

  @Override
  public Trigger voltageControl() {
    return m_controller.cross();
    // return m_controller.L2();
  }
}
