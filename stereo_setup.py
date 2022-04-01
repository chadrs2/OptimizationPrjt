cfg = {
    "name": "OptimizationPrjt",
    "world": "PierHarbor",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 200,
    "frames_per_sec": True,
    "octree_min": 0.02,
    "octree_max": 5.0,
    "agents":[
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor",
                    "socket": "IMUSocket"
                },
                {
                    "sensor_type": "VelocitySensor",
                    "socket": "IMUSocket"
                },
                {
                    "sensor_type": "IMUSensor",
                    "socket": "IMUSocket",
                    "Hz": 200,
                    "configuration": {
                        "AccelSigma": 0.00277,
                        "AngVelSigma": 0.00123,
                        "AccelBiasSigma": 0.00141,
                        "AngVelBiasSigma": 0.00388,
                        "ReturnBias": True
                    }
                },
                {
                    "sensor_type": "GPSSensor",
                    "socket": "IMUSocket",
                    "Hz": 5,
                    "configuration":{
                        "Sigma": 0.5,
                        "Depth": 1,
                        "DepthSigma": 0.25
                    }
                },
                {
                    "sensor_type": "RGBCamera",
                    "sensor_name": "LeftCamera",
                    "socket": "CameraLeftSocket",
                    "Hz": 5,
                    "configuration": {
                        "CaptureWidth": 512,
                        "CaptureHeight": 512
                    }
                },
                {
                    "sensor_type": "RGBCamera",
                    "sensor_name": "RightCamera",
                    "socket": "CameraLeftSocket",
                    "location": [0.0, -0.5, 0.0],
                    "Hz": 5,
                    "configuration": {
                        "CaptureWidth": 512,
                        "CaptureHeight": 512
                    }
                },
                {
                    "sensor_type": "LocationSensor",
                    "sensor_name": "MyLocation",
                    "socket": "Origin"
                }
            ],
            "control_scheme": 0,
            "location": [525.0, -660.0, -12.0],
            "rotation": [0.0, 0.0, 0.0]
        }
    ],

    "window_width":  1280,
    "window_height": 720
}