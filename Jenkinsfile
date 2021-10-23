pipeline {
    agent { docker { image 'osrf/ros:foxy-desktop' } }
    stages {
        stage('Build ROS2 package') {
            steps {
                sh '''#!/bin/bash
                source /opt/ros/foxy/setup.sh;
                colcon build;
                '''
            }
        }
    }
}
