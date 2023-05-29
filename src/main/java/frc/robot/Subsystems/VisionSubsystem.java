// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera Camera;
  PhotonPipelineResult result;

  public VisionSubsystem() {
    Camera = new PhotonCamera("photonvision");
  }

  @Override
  public void periodic() {
    result = Camera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var yaw = target.getPitch();
      var camToTarget = target.getBestCameraToTarget();
    }
    // This method will be called once per scheduler run
  }
}
