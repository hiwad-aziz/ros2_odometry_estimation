pipeline {
    agent { docker { image 'osrf/ros:foxy-desktop' } }
    stages {
        stage('build') {
            steps {
                sh 'colcon build --packages-select odometry_estimator'
            }
        }
    }
}
