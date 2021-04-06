/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
using System;
using UnityEngine;

namespace Isaac {

/// <summary>
/// A simulated differential-based carter driver based on speed commands.
/// Use Unity Wheel Collider to model the two driving wheels and one caster wheel for carter.
/// Note: Validity of the WheelCollider parameters need to be fine tuned to match physical robot.
/// </summary>
public class UnityDifferentialBaseSimulation : MonoBehaviour {
  /// <summary>
  /// The name of the channel on which commands are received
  /// </summary>
  public string inputComponent = "input";
  public string commandChannelName = "base_command";

  /// <summary>
  /// The name of the channel on which state informations is published
  /// </summary>
  public string outputComponent = "output";
  public string stateChannelName = "base_state";

  /// <summary>
  /// The maximum allowed time duration which the robot will continue with the last sent speed
  /// command in the absence of speed commands.
  /// </summary>
  public float maximumTimeWithoutCommand = 0.2f;

  /// <summary>
  /// The maximal allowed linear and angular speed
  /// </summary>
  public Vector2 maximumSpeed = new Vector2(3.0f, 1.5f);

  /// <summary>
  /// The maximal motorTorque apply to the driving wheels.
  /// This determines the maximum acceleration.
  /// </summary>
  public float maxMotorTorque = 10.0f;

  /// <summary>
  /// Whether to use a proportional driver (true) or always apply maxMotorTorque (false)
  /// </summary>
  public bool useProprotionalDriver = true;

  /// <summary>
  /// Proportional controller gain
  /// </summary>
  public float proportionalGain = 100.0f;

  /// <summary>
  /// The two driving wheels of carter.
  /// </summary>
  public WheelCollider wheelFL, wheelFR;

  /// <summary>
  /// Rigidbody whose state (velocity, acceleration) is being published.
  /// </summary>
  public Rigidbody body;

  /// <summary>
  /// Shift the rigidbody center of mass from the default value.
  /// Use this to lower the center of mass for better stability.
  /// </summary>
  public Vector3 centerOfMassShift = Vector3.zero;

  /// <summary>
  /// brakeTorque applied when braking is requested
  /// </summary>
  public float brakeTorque = 100.0f;

  /// <summary>
  /// A smoothing factor for the estimated acceleration. Smoothing the acceleration is important
  /// as acceleration is estimated via finite differences and can be very noisy. The higher the
  /// value the more smoothing will be applied. If set to 0 smoothing will not be used.
  /// </summary>
  public float accelerationSmoothing = 1f;


  // stored latest measured speed for calculating acceleration
  Vector2 commandedSpeed;
  float lastCommandTime;
  Vector2 lastSpeed;
  bool brakeRequested;

  // The last acceleration used for acceleration smoothing.
  Vector2 lastAcceleration;

  Vector2 wheelCurrentSpeed = Vector2.zero;
  Vector2 wheelDesiredSpeed = Vector2.zero;

  // half length between FL and FR wheel
  float wheelBase;

  const float radToDeg = 57.2958f;

  void Start() {
    
    commandedSpeed = Vector2.zero;
    lastSpeed = Vector2.zero;
    brakeRequested = false;

    lastAcceleration = Vector2.zero;

    // get wheel base
    wheelBase = Vector3.Distance(wheelFL.transform.position, wheelFR.transform.position) / 2;

    // the default wheel collider forward is towards z, while carter forward is x.
    // so turn the steering angle to face wheels forward
    foreach (WheelCollider wheel in GetComponentsInChildren<WheelCollider>()) {
      wheel.steerAngle = 90;
    }

    // shift rigidbody center of mass
    body.centerOfMass += centerOfMassShift;
    wheelFL.motorTorque = 5;
    wheelFR.motorTorque = 5;
  }

  void Update() {
    
    // getWheelDesireSpeed(commandedSpeed);
    
  }

  void FixedUpdate() {
    
    // if (brakeRequested) {
    //   wheelFL.brakeTorque = brakeTorque;
    //   wheelFR.brakeTorque = brakeTorque;
    //   wheelFL.motorTorque = 0.0f;
    //   wheelFR.motorTorque = 0.0f;
    // } else {
    //   // get wheel current speed from rpm
    //   wheelCurrentSpeed[0] = AngularSpeedRpmToRad(wheelFL.rpm) * wheelFL.radius;
    //   wheelCurrentSpeed[1] = AngularSpeedRpmToRad(wheelFR.rpm) * wheelFR.radius;
    //
    //   // calculate Torque to apply based on the current speed and the desired speed from the last command
    //   wheelFL.motorTorque = getTorque(wheelCurrentSpeed[0], wheelDesiredSpeed[0]);
    //   wheelFR.motorTorque = getTorque(wheelCurrentSpeed[1], wheelDesiredSpeed[1]);
    //   wheelFL.brakeTorque = 0.0f;
    //   wheelFR.brakeTorque = 0.0f;
    // }
    //
    // // measure current speed and acceleration
    // Vector3 vecForward = body.rotation * Vector3.right;
    // Vector2 measuredSpeed = new Vector2(Vector3.Dot(body.velocity, vecForward), -body.angularVelocity.y);
    // Vector2 measuredAcceleration = (measuredSpeed - lastSpeed) / Time.deltaTime;
    // lastAcceleration += TimedSmoothingFactor(Time.deltaTime, accelerationSmoothing)
    //     * (measuredAcceleration - lastAcceleration);

    
  }

  // calculate the desired speed of the left and right wheels given linear and angular speed
  private void getWheelDesireSpeed(Vector2 commandedSpeed) {
    brakeRequested = Mathf.Approximately(commandedSpeed[0], 0.0f) && Mathf.Approximately(commandedSpeed[1], 0.0f);
    wheelDesiredSpeed[0] = (commandedSpeed[0] - commandedSpeed[1] * wheelBase);
    wheelDesiredSpeed[1] = (commandedSpeed[0] + commandedSpeed[1] * wheelBase);
  }

  // get motorTorque from current and desired velocity different for wheel
  // may need to fine tune for exact wheel dynamic, e.g., avoid oscillation
  private float getTorque(float current, float desired) {
    return useProprotionalDriver ? proportionalController(current, desired) : simpleController(current, desired);
  }

  // apply motorTorque proportional to difference, capped by maxMotorTorque
  private float proportionalController(float current, float desired) {
    return Mathf.Clamp((desired - current) * proportionalGain, -maxMotorTorque, maxMotorTorque);
  }

  // apply maxMotorTorque most of the time
  private float simpleController(float current, float desired) {
    bool sameSign = Mathf.Sign(current) * Mathf.Sign(desired) > 0;
    float diffAbs = Mathf.Abs(current) - Mathf.Abs(desired);
    if (sameSign && diffAbs >= 0 && diffAbs < 0.05) {
      return 0;
    }
    return Mathf.Sign(desired - current) * maxMotorTorque;
  }

  /// <summary>
  /// Calculate rotation speed in radian per second from rotation per minute
  /// </summary>
  static float AngularSpeedRpmToRad(float rpm) {
    return rpm * 2 * Mathf.PI / 60;
  }

  /// <summary>
  /// Limits the absolute of x by the given maximum.
  /// </summary>
  static float Limit(float x, float max) {
    return Mathf.Sign(x) * Mathf.Min(Mathf.Abs(x), max);
  }

  /// <summary>
  /// Applies the `Limit` function component-wise
  /// </summary>
  static Vector2 Limit(Vector2 x, Vector2 max) {
    return new Vector2(Limit(x[0], max[0]), Limit(x[1], max[1]));
  }

  /// <summary>
  /// A smoothing factor used for a time-based exponential decay
  /// </summary>
  static float TimedSmoothingFactor(float dt, float lambda) {
    if (lambda <= dt * 0.01f) {
      return 0f;
    } else {
      return 1.0f - Mathf.Exp(-dt / lambda);
    }
  }
}

}  // namespace Isaac

