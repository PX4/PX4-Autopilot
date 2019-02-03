pipeline {
  agent none
  stages {
    stage('Build') {
      parallel {

        stage('Linux GCC') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2019-01-31'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
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

        stage('Linux Clang') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2019-01-31'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          environment {
            CCACHE_BASEDIR = "${env.WORKSPACE}"
            CC = 'clang'
            CXX = 'clang++'
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
              image 'px4io/px4-dev-ecl:2019-01-31'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            sh 'make coverage'
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
            withCredentials([string(credentialsId: 'ECL_CODECOV_TOKEN', variable: 'CODECOV_TOKEN')]) {
              sh 'curl -s https://codecov.io/bash | bash -s - -F ecl_tests'
            }
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('EKF pytest') {
          agent {
            docker {
              image 'px4io/px4-dev-ecl:2019-01-31'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            sh 'make test_EKF'
            sh 'ccache -s'
            archiveArtifacts 'build/test_build/*.pdf'
            sh 'make distclean'
          }
        }

        stage('test') {
          agent {
            docker {
              image 'px4io/px4-dev-ecl:2019-01-31'
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
              image 'px4io/px4-dev-base:2019-01-31'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'ccache -z'
            sh 'make distclean'
            //sh 'make doxygen'
            sh 'ccache -s'
            publishHTML([
                          reportTitles: 'Doxygen',
                          allowMissing: true,
                          alwaysLinkToLastBuild: true,
                          keepAll: true,
                          reportDir: 'build/doxygen/Documentation',
                          reportFiles: '*',
                          reportName: 'doxygen'
                          ])
            sh 'make distclean'
          }
        }

        stage('PX4/Firmware build') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2019-01-31'
              args '-v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          environment {
            GIT_COMMITTER_EMAIL = "bot@px4.io"
            GIT_COMMITTER_NAME = "PX4BuildBot"
            PX4_FIRMWARE_TEST_BRANCH = "ecl_${env.JOB_BASE_NAME}"
          }
          steps {
            sh 'export'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone --branch master --origin px4 https://${GIT_USER}:${GIT_PASS}@github.com/PX4/Firmware.git')
              sh('cd Firmware; git checkout -B ${PX4_FIRMWARE_TEST_BRANCH}')
              sh('if [ -n "${CHANGE_FORK+set}" ]; then cd Firmware; git config --file=.gitmodules submodule."src/lib/ecl".url https://github.com/${CHANGE_FORK}/ecl.git; fi')
              sh('cd Firmware; git submodule sync')
              sh('cd Firmware; git submodule update --init --recursive --force src/lib/ecl')
              sh 'cd Firmware/src/lib/ecl; git fetch --all --tags; git checkout -f ${GIT_COMMIT}'
              sh('cd Firmware; git commit -a -m "PX4/ecl test `date` ${GIT_BRANCH} ${GIT_COMMIT}"')
              sh('cd Firmware; git push --force px4 ${PX4_FIRMWARE_TEST_BRANCH}')
            }
          }
        }

      } // parallel
    } // stage Test
  } // stages

  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '30'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
