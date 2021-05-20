/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using NUnit.Framework;
using rtaNetworking.Streaming;
using UnityEngine.UIElements;

namespace Isaac
{
    /// <summary>
    /// A simulated differential-based carter driver based on speed commands.
    /// Use Unity Wheel Collider to model the two driving wheels and one caster wheel for carter.
    /// Note: Validity of the WheelCollider parameters need to be fine tuned to match physical robot.
    /// </summary>
    public class UnityDifferentialBaseSimulation : MonoBehaviour
    {
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
        public float maxMotorTorque = 100.0f;

        /// <summary>
        /// Whether to use a proportional driver (true) or always apply maxMotorTorque (false)
        /// </summary>
        public bool useProprotionalDriver = true;

        /// <summary>
        /// Proportional controller gain
        /// </summary>
        public float proportionalGain = 200.0f;

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

        DateTime time;

        string path;
        string path1;
        List<Vector3> CoordFile;
        float second = 0;
        int i = 0;

        private Control _control;

        //Mutexes to access shared data
        private readonly object _locker = new object();
        private readonly object _locker_point = new object();
        float linear_velocity;
        float angular_velocity;

        private ScreenShot screenShoter;

        //angle to cube 
        private float _theta = 0;

        //distance to cube
        private float _distanceToCube = 0;


        string SnapShotName()
        {
            return string.Format("{0}/Snapshots/snap/snap_{1}x{2}_{3}.jpg", Application.dataPath, 256, 256,
                System.DateTime.Now.ToString("yy-MM-dd_HH-mm-ss"));
        }

        public void Receive()
        {
            UdpClient receivingUdpClient = new UdpClient(5555);

            IPEndPoint RemoteIpEndPoint = null;

            while (true)
            {
                try
                {
                    byte[] receiveBytes = receivingUdpClient.Receive(ref RemoteIpEndPoint);

                    string returnData = Encoding.UTF8.GetString(receiveBytes);
                    String[] splitted = returnData.Split(new char[] {' '}, StringSplitOptions.RemoveEmptyEntries);

                    if (splitted.Length == 2)
                    {
                        float distance, theta;
                        if (float.TryParse(splitted[0], NumberStyles.Float, CultureInfo.InvariantCulture, out distance) &&
                            float.TryParse(splitted[1], NumberStyles.Float, CultureInfo.InvariantCulture, out theta))
                        {
                            lock (_locker)
                            {
                                _theta = theta;
                                _distanceToCube = distance;
                            }
                        }
                        else
                        {
                            Debug.Log($"Cannot parse distance: {splitted[0]} ");
                            Debug.Log($"Cannot parse theta: {splitted[1]}");
                        }
                    }
                }
                catch (Exception ex)
                {
                    Debug.Log("Exception occured: " + ex.ToString() + "\n  " + ex.Message);
                }
            }
        }

        void Start()
        {
            //how to control the robot
            _control = Control.Python;


            path = @"C:\Users\ROYAL COMPUTERA\Desktop\1\Coord.txt";
            path1 = @"C:\Users\ROYAL COMPUTERA\Desktop\1\сoordUnity.txt";

            File.Delete(path1);

            time = DateTime.Now;
            // Debug.Log($"{time}");


            using (StreamReader SR = new StreamReader(path, true))
            {
                string line;
                CoordFile = new List<Vector3>();

                while ((line = SR.ReadLine()) != null)
                {
                    var words = line.Split(' ');
                    Vector3 v3 = Vector3.zero;

                    v3.x = (float) Convert.ToDouble(words[0]);
                    v3.y = (float) Convert.ToDouble(words[1]);
                    v3.z = (float) Convert.ToDouble(words[2]);

                    CoordFile.Add(v3);


                    // Debug.Log($"{line}");
                }

                // Debug.Log($"{CoordFile.Count}");
            }

            commandedSpeed = Vector2.zero;
            lastSpeed = Vector2.zero;
            brakeRequested = false;

            lastAcceleration = Vector2.zero;

            // get wheel base
            wheelBase = Vector3.Distance(wheelFL.transform.position, wheelFR.transform.position) / 2;

            // the default wheel collider forward is towards z, while carter forward is x.
            // so turn the steering angle to face wheels forward
            foreach (WheelCollider wheel in GetComponentsInChildren<WheelCollider>())
            {
                wheel.steerAngle = 90;
            }


// shift rigidbody center of mass
            body.centerOfMass += centerOfMassShift;
            // wheelFL.motorTorque = 2.5f;
            // wheelFR.motorTorque = 2.0f;
            InvokeRepeating(nameof(LaunchProjectile), 0.0f, 0.030f);

            //start UdpListener
            if (_control == Control.Python)
            {
                new Thread(Receive).Start();
            }
        }

        private void LaunchProjectile()
        {
            // Debug.Log($"Launch: {i}");
            // if (i <= CoordFile.Count-1)
            // {
            //
            //     if (time.AddMilliseconds(CoordFile[i].z) < DateTime.Now)
            //     {
            //         var cube = GameObject.Find("Cube");
            //         cube.transform.position = new Vector3(CoordFile[i].x, 1, CoordFile[i].y);
            //         
            //
            //         Debug.Log($"YES");
            //         i = i + 1;
            //         Debug.Log($"Increase: {i}");
            //     }
            // }
            // else
            // {
            //     Debug.Log($"Thats All");
            // }

            Vector3 vecForward = body.rotation * Vector3.right;
            Vector2 measuredSpeed = new Vector2(Vector3.Dot(body.velocity, vecForward), -body.angularVelocity.y);


            using (StreamWriter sw = new StreamWriter(path1, true))
            {
                sw.WriteLine($"{body.transform.position.x} {body.transform.position.y}");
                // sw.WriteLine(wheelCurrentSpeed[0]);
                // sw.Write("Motor torque: ");
                // sw.WriteLine(wheelFL.motorTorque);
            }

            //
            // using (StreamWriter sw = new StreamWriter(path2, true))
            // {
            //     sw.WriteLine($"{measuredSpeed[1]}");
            //     // sw.WriteLine(wheelCurrentSpeed[0]);
            //     // sw.Write("Motor torque: ");
            //     // sw.WriteLine(wheelFL.motorTorque);
            // }

            // var cube = GameObject.Find("Cube");
            // using (StreamWriter sw = new StreamWriter(path, true))
            // {
            //     string s = $"{cube.transform.position.x} {cube.transform.position.z} {second}";
            //         sw.WriteLine($"{s}");
            //         // sw.WriteLine(wheelCurrentSpeed[0]);
            //         // sw.Write("Motor torque: ");
            //         // sw.WriteLine(wheelFL.motorTorque);
            //     }

            // Debug.Log($"Linear{measuredSpeed[1]}");

            second = second + 0.001f;
        }

        private void controlWithMatrix()
        {
            Vector3 vecForward = body.rotation * Vector3.right;

            Vector2 measuredSpeed = new Vector2(Vector3.Dot(body.velocity, vecForward), -body.angularVelocity.y);

            var cube = GameObject.Find("Cube");

            var cubeLocal = body.transform.InverseTransformPoint(cube.transform.position);

            var theta_new = 0.0;

            if (cubeLocal.x < 0 && cubeLocal.z < 0)
            {
                theta_new = (float) ((Math.Atan(cubeLocal.z / cubeLocal.x)) - Math.PI);
            }
            else if (cubeLocal.x < 0)
            {
                theta_new = (float) ((Math.Atan(cubeLocal.z / cubeLocal.x)) + Math.PI);
            }
            else
            {
                theta_new = (float) (Math.Atan(cubeLocal.z / cubeLocal.x));
            }


            Matrix<double> v = DenseMatrix.OfArray(new double[,]
            {
                {measuredSpeed[0]}, //linear
                {measuredSpeed[1]}
            }); //angular
            Matrix<double> Kv = DenseMatrix.OfArray(new double[,]
            {
                {1, 0},
                {0, 1}
            });
            Matrix<double> Kp = DenseMatrix.OfArray(new double[,]
            {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
            });


            Matrix<double> R = DenseMatrix.OfArray(new double[,]
            {
                {Math.Cos((theta_new)), Math.Sin(theta_new), 0},
                {0, 0, 1}
            });
            Matrix<double> u = DenseMatrix.OfArray(new double[,]
            {
                {0},
                {0}
            });
            Matrix<double> xd = DenseMatrix.OfArray(new double[,]
            {
                {cubeLocal.x},
                {cubeLocal.z},
                {theta_new}
            });

            u = (-1.0) * Kv * v - R * Kp * (-xd);

            double linear_velocity = u[0, 0];
            double angular_velocity = u[1, 0];


            commandedSpeed = Limit(new Vector2((float) linear_velocity, (float) angular_velocity), maximumSpeed);
            //
            Debug.Log($"{Math.Sqrt(Math.Pow(cubeLocal.x,2)+Math.Pow(cubeLocal.z,2))}");
            getWheelDesireSpeed(commandedSpeed);
        }

        void controlWithNoMatrix()
        {
            float theta_new, k_v, k_h;

            var point_to = GameObject.Find("Cube").transform.position;

            k_v = 0.5f;
            k_h = 4.0f;

            var point_local = body.transform.InverseTransformPoint(point_to);
            //
            //
            var linear_velocity = (float) (k_v * Math.Sqrt(Math.Pow(point_local.x, 2) + Math.Pow(point_local.z, 2)));
            // //
            // // 3rd quater
            if (point_local.x < 0 && point_local.z < 0)
            {
                theta_new = (float) ((Math.Atan(point_local.z / point_local.x)) - Math.PI);
            }
            else if (point_local.x < 0)
            {
                theta_new = (float) ((Math.Atan(point_local.z / point_local.x)) + Math.PI);
            }
            else
            {
                theta_new = (float) (Math.Atan(point_local.z / point_local.x));
            }


            var angular_velocity = k_h * theta_new;
            // //
            //
            commandedSpeed = Limit(new Vector2((float) linear_velocity, (float) angular_velocity), maximumSpeed);
            //

            getWheelDesireSpeed(commandedSpeed);
        }

        void controlWithPython()
        {
            float theta_new;
            float distance_new;
            lock (_locker)
            {
                theta_new = _theta;
                distance_new = _distanceToCube;
            }


            // float k_v = 0.5f;
            // float k_h = 4.0f;
            //
            // //
            // //
            // var linear_velocity = (float) (k_v * distance_new);
            // // //
            //
            //
            // var angular_velocity = k_h * theta_new;
            // //
            //
            commandedSpeed = Limit(new Vector2((float) distance_new, (float) theta_new), maximumSpeed);
            //

            getWheelDesireSpeed(commandedSpeed);
        }

        enum Control
        {
            Matrix,
            NoMatrix,
            Python
        }

        void Update()
        {
            switch (_control)
            {
                case Control.Matrix:
                    controlWithMatrix();
                    break;
                case Control.NoMatrix:
                    controlWithNoMatrix();
                    break;
                case Control.Python:
                    controlWithPython();
                    break;
            }

            // //make screenshot
        }

        void FixedUpdate()
        {
            if (brakeRequested)
            {
                wheelFL.brakeTorque = brakeTorque;
                wheelFR.brakeTorque = brakeTorque;
                wheelFL.motorTorque = 0.0f;
                wheelFR.motorTorque = 0.0f;
            }
            else
            {
                // get wheel current speed from rpm
                wheelCurrentSpeed[0] = AngularSpeedRpmToRad(wheelFL.rpm) * wheelFL.radius;
                wheelCurrentSpeed[1] = AngularSpeedRpmToRad(wheelFR.rpm) * wheelFR.radius;

                // calculate Torque to apply based on the current speed and the desired speed from the last command
                wheelFL.motorTorque = getTorque(wheelCurrentSpeed[0], wheelDesiredSpeed[0]);
                wheelFR.motorTorque = getTorque(wheelCurrentSpeed[1], wheelDesiredSpeed[1]);
                wheelFL.brakeTorque = 0.0f;
                wheelFR.brakeTorque = 0.0f;
            }
        }

        // calculate the desired speed of the left and right wheels given linear and angular speed
        private void getWheelDesireSpeed(Vector2 commandedSpeed)
        {
            brakeRequested = Mathf.Approximately(commandedSpeed[0], 0.0f) &&
                             Mathf.Approximately(commandedSpeed[1], 0.0f);
            wheelDesiredSpeed[0] = (commandedSpeed[0] - commandedSpeed[1] * wheelBase);
            wheelDesiredSpeed[1] = (commandedSpeed[0] + commandedSpeed[1] * wheelBase);
        }

        // get motorTorque from current and desired velocity different for wheel
        // may need to fine tune for exact wheel dynamic, e.g., avoid oscillation
        private float getTorque(float current, float desired)
        {
            return useProprotionalDriver
                ? proportionalController(current, desired)
                : simpleController(current, desired);
        }

        // apply motorTorque proportional to difference, capped by maxMotorTorque
        private float proportionalController(float current, float desired)
        {
            return Mathf.Clamp((desired - current) * proportionalGain, -maxMotorTorque, maxMotorTorque);
        }

        // apply maxMotorTorque most of the time
        private float simpleController(float current, float desired)
        {
            bool sameSign = Mathf.Sign(current) * Mathf.Sign(desired) > 0;
            float diffAbs = Mathf.Abs(current) - Mathf.Abs(desired);
            if (sameSign && diffAbs >= 0 && diffAbs < 0.05)
            {
                return 0;
            }

            return Mathf.Sign(desired - current) * maxMotorTorque;
        }

        /// <summary>
        /// Calculate rotation speed in radian per second from rotation per minute
        /// </summary>
        static float AngularSpeedRpmToRad(float rpm)
        {
            return rpm * 2 * Mathf.PI / 60;
        }


        /// <summary>
        /// Limits the absolute of x by the given maximum.
        /// </summary>
        static float Limit(float x, float max)
        {
            return Mathf.Sign(x) * Mathf.Min(Mathf.Abs(x), max);
        }

        /// <summary>
        /// Applies the Limit function component-wise
        /// </summary>
        static Vector2 Limit(Vector2 x, Vector2 max)
        {
            return new Vector2(Limit(x[0], max[0]), Limit(x[1], max[1]));
        }

        /// <summary>
        /// A smoothing factor used for a time-based exponential decay
        /// </summary>
        static float TimedSmoothingFactor(float dt, float lambda)
        {
            if (lambda <= dt * 0.01f)
            {
                return 0f;
            }
            else
            {
                return 1.0f - Mathf.Exp(-dt / lambda);
            }
        }
    }
} // namespace Isaac