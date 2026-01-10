package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import frc.lib.littletonUtils.AllianceFlipUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  private static final Translation2d m_speakerPose =
      AllianceFlipUtil.apply(FieldConstants.kSpeakerLocation);

  private static final InterpolatingDoubleTreeMap m_topVelocityTree =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_bottomVelocityTree =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodTree = new InterpolatingDoubleTreeMap();

  static {
    m_topVelocityTree.put(1., 1.);
    m_topVelocityTree.put(200., 200.);
    m_bottomVelocityTree.put(1., 1.);
    m_bottomVelocityTree.put(200., 200.);
    hoodTree.put(1., 1.);
    hoodTree.put(200., 130.);
  }

  public static Rotation2d getShooterPosition(Drive drive) {
    var currPose = drive.getPose();
    double x = m_speakerPose.getX() - currPose.getX();
    double y = m_speakerPose.getY() - currPose.getY();

    var desiredHeading =
        available(Rotation2d.fromRadians(Math.atan2(y, x)).minus(drive.getRotation()));
    Logger.recordOutput("Turret/angle", desiredHeading.plus(drive.getRotation()));
    Logger.recordOutput(
        "Speaker", new Pose2d(m_speakerPose, new Rotation2d(Angle.ofBaseUnits(180, Degree))));

    return desiredHeading;
  }

  public static Rotation2d available(Rotation2d angle) {
    double available;
    double degrees = angle.getDegrees();
    if (degrees > 0) {
      if (degrees > PivotConstants.kMinAngle && degrees < PivotConstants.kMaxAngle) {
        available = degrees;
      } else {
        available = PivotConstants.kMinAngle;
      }
    } else {
      if (degrees > PivotConstants.kMaxNegativeAngle
          && degrees < PivotConstants.kMinNegativeAngle) {
        available = degrees;
      } else {
        available = PivotConstants.kMinNegativeAngle;
      }
    }

    Logger.recordOutput("Turret/setpointAchievable", degrees == available);

    return Rotation2d.fromDegrees(available);
  }

  public static Rotation2d getHoodPosition(Drive drive) {
    var currTranslation = drive.getPose().getTranslation();
    double distance = m_speakerPose.getDistance(currTranslation);
    return Rotation2d.fromDegrees(hoodTree.get(distance));
  }

  public static Pair<Double, Double> getShooterSpeeds(Drive drive) {
    var currTranslation = drive.getPose().getTranslation();
    double distance = m_speakerPose.getDistance(currTranslation);

    return Pair.of(m_topVelocityTree.get(distance), m_bottomVelocityTree.get(distance));
  }
}
