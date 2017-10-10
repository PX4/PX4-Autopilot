pipeline {
  agent {
    docker {
      image 'docker pull px4io/px4-dev-simulation'
      args '2017-09-26'
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