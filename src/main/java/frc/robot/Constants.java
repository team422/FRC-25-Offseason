package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.littletonUtils.LoggedTunableNumber;

public final class Constants {
  public static final boolean kTuningMode = true;

  public static final Mode kRealMode = Mode.REAL;
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

  public static final boolean kUsePhoenixDiagnosticServer = false;

  // set to false to disable the base refresh manager
  public static final boolean kUseBaseRefreshManager = false;

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running on the proto board */
    PROTO,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean kUseComponents = true;

  // set to false to disable alerts
  public static final boolean kUseAlerts = true && kCurrentMode != Mode.SIM;

  public static final class CustomConstants {
    public static final double kGearRatio = 1;

    public static final double kTolerance = .5;
    public static final double kIdleVoltage = 0.;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Custom/P", 0.0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Custom/I", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Custom/D", 0.0);
    public static final LoggedTunableNumber kKV = new LoggedTunableNumber("Custom/kKS", 0.0);

    public static final LoggedTunableNumber kSimP = new LoggedTunableNumber("Custom/simP", 0.135);
    public static final LoggedTunableNumber kSimI = new LoggedTunableNumber("Custom/simI", 0.0);
    public static final LoggedTunableNumber kSimD = new LoggedTunableNumber("Custom/simD", 0.0);
    public static final LoggedTunableNumber kSimKV = new LoggedTunableNumber("Custom/simKS", 0.126);

    public static final LoggedTunableNumber kVelocityRPS =
        new LoggedTunableNumber("Custom/Velocity", 0.0);
    public static final LoggedTunableNumber kVoltage =
        new LoggedTunableNumber("Custom/Voltage", 0.0);

    // sim
    public static final double kSimMOI = .001;
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60Foc(2);
  }

  public static final class CurrentLimitConstants {
    // Drive
    public static final double kMotorDefaultSupplyCurrentLimit = 75.0;
    public static final double kMotorDefaultStatorCurrentLimit = 180.0;
  }

  public static final class Ports {
    public static final int kCustomMotor = 0;

    public static final String kMainCanivoreName = "Main";
  }
}
