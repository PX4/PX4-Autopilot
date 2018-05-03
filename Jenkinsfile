pipeline {
  agent none
  stages {
    stage('build') {
      parallel {

        stage('build') {
          environment {
            CCACHE_BASEDIR = "${env.WORKSPACE}"
          }
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-12-30'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            } 
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            sh 'make'
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('build clang') {
          environment {
            CCACHE_BASEDIR = "${env.WORKSPACE}"
            CC = 'clang'
            CXX = 'clang++'
          }
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-12-30'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            sh 'make'
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('pytest') {
          agent {
            docker {
              image 'px4io/px4-dev-ecl:2018-04-22'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            sh 'make'
            sh 'ccache -s'
            //sh 'RUN_PYTEST=1 ./build.sh'
            sh 'make distclean'
          }
        }
        
        stage('OSX') {
          agent {
            node {
              label 'mac'
            }
          }
          environment {
            CCACHE_BASEDIR = "${env.WORKSPACE}"
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            sh 'make'
            sh 'ccache -s'
            sh 'make distclean'
          }
        }    

      }
    }
  }

  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '30'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
