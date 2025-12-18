package frc.robot.subsystems.drive;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.PoseEstimator;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
import frc.robot.util.AlertManager;
import frc.robot.util.MeshedDrivingController;
import frc.robot.util.SubsystemProfiles;

import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class Drive extends SubsystemBase {

  static final Lock m_odometryLock = new ReentrantLock();
  private final GyroIO m_gyroIO;
  public final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine m_sysId;
  private Rotation2d m_lastGyroYaw = new Rotation2d();

  private Alert m_gyroDisconnectedAlert = new Alert("Gyro Disconnected", AlertType.kError);

  public enum DriveProfiles {
    kAutoDefault,
    kTeleopDefault,
    kStop
  }

  private SubsystemProfiles<DriveProfiles> m_profiles;
  private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds m_desiredAutoChassisSpeeds = null;

  private Rotation2d m_desiredHeading = new Rotation2d();

  private PIDController m_driveController =
      new PIDController(
          DriveConstants.kDriveToPointP.get(), DriveConstants.kDriveToPointI.get(), 0.0);

  private PIDController m_autoDriveController =
      new PIDController(
          DriveConstants.kDriveToPointAutoP.get(),
          DriveConstants.kDriveToPointAutoI.get(),
          DriveConstants.kDriveToPointAutoD.get());

  private PIDController m_headingController =
      new PIDController(
          DriveConstants.kDriveToPointHeadingP.get(),
          DriveConstants.kDriveToPointHeadingI.get(),
          DriveConstants.kDriveToPointHeadingD.get());

  private Pose2d m_driveToPointTargetPose = null;

  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final PoseEstimator m_poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  private MeshedDrivingController m_meshedController =
      new MeshedDrivingController(
          new Pose2d(),
          false,
          DriveConstants.kDebounceAmount.get(),
          DriveConstants.kMeshDrivePriority.get());

  private ChassisSpeeds m_userChassisSpeeds;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, 0);
    m_modules[1] = new Module(frModuleIO, 1);
    m_modules[2] = new Module(blModuleIO, 2);
    m_modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    m_modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    Map<DriveProfiles, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(DriveProfiles.kTeleopDefault, this::teleopDefaultPeriodic);
    periodicHash.put(DriveProfiles.kAutoDefault, this::autoDefaultPeriodic);
    periodicHash.put(DriveProfiles.kStop, this::stopPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, DriveProfiles.kTeleopDefault);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController.setTolerance(Units.inchesToMeters(0.5));
    m_autoDriveController.setTolerance(Units.inchesToMeters(0.5));
    m_headingController.setTolerance(Units.degreesToRadians(1));

    AlertManager.registerAlert(m_gyroDisconnectedAlert);
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    double updateInputsStart = HALUtil.getFPGATime();
    m_odometryLock.lock(); // Prevents odometry updates while reading data
    m_gyroIO.updateInputs(m_gyroInputs);
    for (var module : m_modules) {
      module.updateInputs();
    }
    m_odometryLock.unlock();
    Logger.recordOutput(
        "PeriodicTime/DriveUpdateInputs", (HALUtil.getFPGATime() - updateInputsStart) / 1000.0);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_headingController.setP(DriveConstants.kDriveToPointHeadingP.get());
          m_headingController.setI(DriveConstants.kDriveToPointHeadingI.get());
          m_headingController.setD(DriveConstants.kDriveToPointHeadingD.get());
        },
        DriveConstants.kDriveToPointHeadingP,
        DriveConstants.kDriveToPointHeadingI,
        DriveConstants.kDriveToPointHeadingD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_driveController.setP(DriveConstants.kDriveToPointP.get());
          m_driveController.setI(DriveConstants.kDriveToPointI.get());
          m_driveController.setD(DriveConstants.kDriveToPointD.get());
        },
        DriveConstants.kDriveToPointP,
        DriveConstants.kDriveToPointI,
        DriveConstants.kDriveToPointD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_autoDriveController.setP(DriveConstants.kDriveToPointAutoP.get());
          m_autoDriveController.setI(DriveConstants.kDriveToPointAutoI.get());
          m_autoDriveController.setD(DriveConstants.kDriveToPointAutoD.get());
        },
        DriveConstants.kDriveToPointAutoP,
        DriveConstants.kDriveToPointAutoI,
        DriveConstants.kDriveToPointAutoD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_meshedController.setPIDControllers(
              DriveConstants.kMeshedXYP.get(),
              DriveConstants.kMeshedXYD.get(),
              DriveConstants.kMeshedThetaP.get(),
              DriveConstants.kMeshedThetaD.get());
        },
        DriveConstants.kMeshedXYP,
        DriveConstants.kMeshedXYD,
        DriveConstants.kMeshedThetaP,
        DriveConstants.kMeshedThetaD);

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Drive/Gyro", m_gyroInputs);

    double modulePeriodicStart = HALUtil.getFPGATime();
    for (var module : m_modules) {
      module.periodic();
    }
    Logger.recordOutput(
        "PeriodicTime/ModulePeriodic", (HALUtil.getFPGATime() - modulePeriodicStart) / 1000.0);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : m_modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double odometryUpdateStart = HALUtil.getFPGATime();
    // Update odometry
    double[] sampleTimestamps =
        m_modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    Twist2d totalTwist = new Twist2d();
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = m_modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (m_gyroInputs.connected) {
        // Use the real gyro angle
        m_rawGyroRotation = m_gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
        m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      Rotation2d deltaYaw;
      if (i == 0) {
        deltaYaw = m_rawGyroRotation.minus(m_lastGyroYaw);
      } else {
        deltaYaw = m_rawGyroRotation.minus(m_gyroInputs.odometryYawPositions[i - 1]);
      }
      var twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
      twist = new Twist2d(twist.dx, twist.dy, deltaYaw.getRadians());
      totalTwist =
          new Twist2d(
              totalTwist.dx + twist.dx, totalTwist.dy + twist.dy, totalTwist.dtheta + twist.dtheta);
    }

    if (m_gyroInputs.connected) {
      totalTwist.dtheta = m_gyroInputs.yawPosition.minus(m_lastGyroYaw).getRadians();
      m_lastGyroYaw = m_gyroInputs.yawPosition;
    } else {
      totalTwist.dtheta = m_rawGyroRotation.minus(m_lastGyroYaw).getRadians();
      m_lastGyroYaw = m_rawGyroRotation;
    }
    m_poseEstimator.addDriveData(Timer.getTimestamp(), totalTwist);

    Logger.recordOutput(
        "PeriodicTime/OdometryUpdate", (HALUtil.getFPGATime() - odometryUpdateStart) / 1000.0);

    Logger.recordOutput(
        "Drive/FreeFall",
        m_gyroInputs.zAcceleration < DriveConstants.kFreefallAccelerationThreshold);

    Logger.recordOutput("Drive/Profile", m_profiles.getCurrentProfile());

    if (Constants.kUseAlerts && !m_gyroInputs.connected) {
      m_gyroDisconnectedAlert.set(true);
    } else {
      m_gyroDisconnectedAlert.set(false);
    }

    Logger.recordOutput("PeriodicTime/Drive", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void stopPeriodic() {
    runVelocity(new ChassisSpeeds());
  }

  public void autoDefaultPeriodic() {
    runVelocity(new ChassisSpeeds());
  }

  public void teleopDefaultPeriodic() {
    runVelocity(m_desiredChassisSpeeds);

    Logger.recordOutput("Drive/DesiredHeading", m_desiredHeading.getDegrees());
    Logger.recordOutput("Drive/CurrentHeading", getPose().getRotation().getDegrees());
    Logger.recordOutput("Drive/DesiredSpeeds", m_desiredChassisSpeeds);
    Logger.recordOutput("Drive/MeasuredSpeeds", getChassisSpeeds());
  }

  @SuppressWarnings("unused")
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    // "Universal negative sign" (should not be required but i'll leave it as an option)
    if (RobotBase.isReal() && DriveConstants.kRealReversed) {
      if (DriverStation.isTeleopEnabled()) {
        discreteSpeeds.vxMetersPerSecond = -discreteSpeeds.vxMetersPerSecond;
        discreteSpeeds.vyMetersPerSecond = -discreteSpeeds.vyMetersPerSecond;
      }
      discreteSpeeds.omegaRadiansPerSecond = -discreteSpeeds.omegaRadiansPerSecond;
    } else if (RobotBase.isSimulation() && DriveConstants.kSimReversed) {
      discreteSpeeds.vxMetersPerSecond = -discreteSpeeds.vxMetersPerSecond;
      discreteSpeeds.vyMetersPerSecond = -discreteSpeeds.vyMetersPerSecond;
      discreteSpeeds.omegaRadiansPerSecond = -discreteSpeeds.omegaRadiansPerSecond;
    }

    SwerveModuleState[] setpointStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
  }

  
  public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
    Logger.recordOutput("Drive/speed", speeds);
    m_desiredChassisSpeeds = speeds;
    m_userChassisSpeeds = speeds;
  }

  public void setDesiredAutoChassisSpeeds(ChassisSpeeds speeds) {
    m_desiredAutoChassisSpeeds = speeds;
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setDesiredHeading(Rotation2d heading) {
    m_desiredHeading = heading;
  }

  public void setTargetPose(Pose2d pose) {
    m_driveToPointTargetPose = pose;
  }

  public Pose2d getTargetPose() {
    return m_driveToPointTargetPose;
  }

  /** Stops the drive. */
  public void stop() {
    updateProfile(DriveProfiles.kStop);
  }

  public void setCoast() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setBrakeMode(false);
    }
  }

  public void setBrake() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setBrakeMode(true);
    }
  }

  public void setDriveCoast() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDriveBrakeMode(false);
    }
  }

  public void setDriveBrake() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDriveBrakeMode(true);
    }
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations when the profile changes away from kStop and a new
   * ChassisSpeeds is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.kModuleTranslations[i].getAngle();
    }
    DriveConstants.kDriveKinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  public DriveProfiles getCurrentProfile() {
    return m_profiles.getCurrentProfile();
  }

  public void updateProfile(DriveProfiles newProfile) {
    m_profiles.setCurrentProfile(newProfile);
  }
}
