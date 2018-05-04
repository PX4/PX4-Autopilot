pipeline {
  agent none
  stages {
    stage('Build') {
      parallel {

        stage('Linux GCC') {
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

        stage('Linux Clang') {
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

      } // parallel
    } // stage Build

    stage('Test') {
      parallel {

        stage('coverage') {
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
            sh 'make coverage'
            //sh 'bash <(curl -s https://codecov.io/bash) -t ${CODECOV_TOKEN}'
            sh 'make coverage_html'
            // publish html
            publishHTML target: [
              reportTitles: 'code coverage',
              allowMissing: false,
              alwaysLinkToLastBuild: true,
              keepAll: true,
              reportDir: 'build/coverage_build/out',
              reportFiles: '*',
              reportName: 'Code Coverage'
            ]
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('EKF pytest') {
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
            sh 'make test_EKF'
            sh 'ccache -s'
            archiveArtifacts(artifacts: 'build/test_build/*.pdf')
            sh 'make distclean'
          }
        }

        stage('test') {
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
            sh 'make test'
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('doxygen') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2018-03-30'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            //sh 'make doxygen'
            sh 'ccache -s'
            // publish html
            publishHTML target: [
              reportTitles: 'Doxygen',
              allowMissing: true,
              alwaysLinkToLastBuild: true,
              keepAll: true,
              reportDir: 'build/doxygen/Documentation',
              reportFiles: '*',
              reportName: 'doxygen'
            ]
            sh 'make distclean'
          }
        }

      } // parallel
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
