pipeline {
  agent {
    docker {
      image 'px4io/px4-dev-simulation:2017-09-26'
      args '--env=CCACHE_DISABLE=1 --env=CI=1'
    }
    
  }
  stages {
    stage('Quick Check') {
      steps {
        sh '''make distclean;
make quick_check;'''
      }
    }
  }
  environment {
    CI = '1'
    CCACHE_DISABLE = '1'
  }
}