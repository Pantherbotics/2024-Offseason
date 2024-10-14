// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Vision {
//taken from https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java

  private final PhotonCamera mainCam;
  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  NetworkTable camStuff = NetworkTableInstance.getDefault().getTable("camStuff");
  StructArrayPublisher<Pose3d> camPosePub = camStuff.getStructArrayTopic("CamPose", Pose3d.struct).publish();
  StructPublisher<Pose3d> noteCamPose = camStuff.getStructTopic("notecam", Pose3d.struct).publish();

  public Vision() {

    mainCam = new PhotonCamera(VisionConstants.kMainCamName);

    photonEstimator =
            new PhotonPoseEstimator(
                    VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mainCam, VisionConstants.kRobotToMainCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public PhotonPipelineResult getLatestResult() {
      return mainCam.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = mainCam.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      var targets = getLatestResult().getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
          var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
                  tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }
      if (numTags == 0) return estStdDevs;
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
  }


  public void updatePose(CommandSwerveDrivetrain drivetrain){

    var visionEst = getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                var estPose = est.estimatedPose.toPose2d();
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = getEstimationStdDevs(estPose);

                drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
    
    camPosePub.set(new Pose3d[]{
        new Pose3d(drivetrain.getState().Pose).plus(VisionConstants.kRobotToNoteCam),
        new Pose3d(drivetrain.getState().Pose).plus(VisionConstants.kRobotToBackCam),
        new Pose3d(drivetrain.getState().Pose).plus(VisionConstants.kRobotToMainCam),
    });
    noteCamPose.set(new Pose3d(drivetrain.getState().Pose).plus(VisionConstants.kRobotToNoteCam));

    }
}
