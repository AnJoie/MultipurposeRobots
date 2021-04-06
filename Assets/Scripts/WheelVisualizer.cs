/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
using UnityEngine;

namespace Isaac {

/// <summary>
/// Rotate a wheel mesh according to the wheelcolliders rotation speed
/// </summary>
public class WheelVisualizer : MonoBehaviour {
  /// <summary>
  /// The wheel collider this wheel mesh should be connected to
  /// </summary>
  public WheelCollider targetWheel;

  // Rotate the wheel mesh according to current wheel collider rotation speed
  void Update() {
    transform.Rotate(0, -AngularSpeedRpmToDeg(targetWheel.rpm) * Time.deltaTime, 0);
  }

  /// <summary>
  /// Calculate rotation speed in radian per second from rotation per minute
  /// </summary>
  static float AngularSpeedRpmToDeg(float rpm) {
    // rpm / 60 (seconds per minut) * 360 (degrees per rotation)
    return rpm * 6.0f;
  }
}

}  // namespace Isaac
