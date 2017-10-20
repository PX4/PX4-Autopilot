pipeline {
  agent {
    docker {
      args '2017-09-26'
      image 'px4io/px4-dev-simulation'
    }
    
  }
  stages {
    stage('Build') {
      parallel {
        stage('check_format') {
          steps {
            sh 'make check_format'
          }
        }
        stage('nuttx_px4fmu-v2_default') {
          steps {
            sh 'make px4fmu-v2_default'
          }
        }
        stage('posix_sitl_default') {
          steps {
            sh 'make posix_sitl_default'
          }
        }
      }
    }
  }
}