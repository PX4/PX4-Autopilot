pipeline {
    agent { 
        label 'linux'
    }

    environment { 
        BUILD_NAME = 'PX4 Firmware'
    }

    stages {
        stage('Build') {
            steps {
                echo 'Building $BUILD_NAME..'
            }
        }
        stage('Test') {
            steps {
                echo 'Testing..'
            }
        }
        stage('Deploy') {
            when {
                expression {
                    $BUILD_NAME == 'PX4 Firmware'
                }
            }
            steps {
                echo 'Deploying....'
            }
        }
    }
}
