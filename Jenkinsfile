pipeline {
  agent {
    docker {
      image 'px4io/px4-dev-simulation:2017-09-26'
      args '--env=CCACHE_DISABLE --env=CI'
    }
    
  }
  stages {
    stage('Quick Check') {
      steps {
        sh '''make distclean;
make posix_sitl_default;'''
      }
    }
    stage('px4_metadata') {
      steps {
        sh 'make px4_metadata'
      }
    }
  }
  environment {
    CI = '1'
    CCACHE_DISABLE = '1'
  }
}