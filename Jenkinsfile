pipeline {
    agent { docker { image 'osrf/ros:foxy-desktop' } }
    stages {
        stage('Build ROS2 package') {
            steps {

                sh '''
                mkdir -p colcon_ws/src;
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
            options {
                checkoutToSubdirectory('colcon_ws/src')
            }
        }
    }
}
