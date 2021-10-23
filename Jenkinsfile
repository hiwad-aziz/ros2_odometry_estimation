pipeline {
    agent { docker { image 'osrf/ros:foxy-desktop' } }
    stages {
        stage('Build ROS2 package') {
            steps {
                sh '''#!/bin/bash
                mkdir -p colcon_ws/src;
                mv ros2_odometry_estimation colcon_ws/src/;
                cd colcon_ws;
                source /opt/ros/foxy/setup.sh;
                colcon build --packages-select odometry_estimator;
                '''
            }
            post {
                always {
                    sh 'rm -rf colcon_ws'
                }
            }
        }
    }
}
