pipeline {
  agent {
    docker {
      image 'px4io/px4-dev-simulation:2017-09-26'
      args '--env=CCACHE_DISABLE=1 --env=CI=1'
    }
    
  }
  stages {
    stage('Build') {
      parallel {
        stage('nuttx_px4fmu-v2_default') {
          steps {
            sh '''make distclean;
make px4fmu-v2_default'''
          }
        }
        stage('posix_sitl_default') {
          steps {
            sh '''make distclean;
make posix_sitl_default'''
          }
        }
        stage('nuttx_px4fmu-v3_default') {
          steps {
            sh '''make distclean;
make px4fmu-v3_default'''
          }
        }
        stage('nuttx_px4fmu-v4_default') {
          steps {
            sh '''make distclean;
make px4fmu-v4_default'''
          }
        }
      }
    }
    stage('Test') {
      parallel {
        stage('tests') {
          steps {
            sh '''make distclean;
make tests'''
          }
        }
        stage('check_format') {
          steps {
            sh 'make check_format'
          }
        }
        stage('clang-tidy') {
          steps {
            sh '''make distclean;
make clang-tidy-quiet'''
          }
        }
        stage('tests_coverage') {
          steps {
            sh '''make distclean;
make tests_coverage'''
          }
        }
      }
    }
  }
  environment {
    CI = '1'
    CCACHE_DISABLE = '1'
  }
}