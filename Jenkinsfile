pipeline {
  agent {
    docker {
      args '2017-09-26'
      image 'px4io/px4-dev-simulation'
    }
    
  }
  stages {
    stage('build') {
      steps {
        sh 'make check_format'
      }
    }
  }
}