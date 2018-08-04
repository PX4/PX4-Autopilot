pipeline {
  agent none
  stages {

 stage('Analysis') {

      parallel {

        stage('Style Check') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-07-19' }
          }

          steps {
            sh 'make check_format'
          }
        }

        stage('bloaty px4fmu-v2') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'make nuttx_px4fmu-v2_default'
            sh 'make nuttx_px4fmu-v2_default bloaty_symbols'
            sh 'make nuttx_px4fmu-v2_default bloaty_compileunits'
            sh 'make nuttx_px4fmu-v2_default bloaty_inlines'
            sh 'make nuttx_px4fmu-v2_default bloaty_templates'
            sh 'make nuttx_px4fmu-v2_default bloaty_compare_master'
            sh 'make sizes'
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('bloaty px4fmu-v5') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'make nuttx_px4fmu-v5_default'
            sh 'make nuttx_px4fmu-v5_default bloaty_symbols'
            sh 'make nuttx_px4fmu-v5_default bloaty_compileunits'
            sh 'make nuttx_px4fmu-v5_default bloaty_inlines'
            sh 'make nuttx_px4fmu-v5_default bloaty_templates'
            sh 'make nuttx_px4fmu-v5_default bloaty_compare_master'
            sh 'make sizes'
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

        stage('clang analyzer') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make scan-build'
            // publish html
            publishHTML target: [
              reportTitles: 'clang static analyzer',
              allowMissing: false,
              alwaysLinkToLastBuild: true,
              keepAll: true,
              reportDir: 'build/scan-build/report_latest',
              reportFiles: '*',
              reportName: 'Clang Static Analyzer'
            ]
            sh 'make distclean'
          }
          when {
            anyOf {
              branch 'master'
              branch 'beta'
              branch 'stable'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('clang tidy') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make clang-tidy-quiet'
            sh 'make distclean'
          }
        }

        stage('cppcheck') {
          agent {
            docker {
              image 'px4io/px4-dev-base:ubuntu17.10'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make cppcheck'
            // publish html
            publishHTML target: [
              reportTitles: 'Cppcheck',
              allowMissing: false,
              alwaysLinkToLastBuild: true,
              keepAll: true,
              reportDir: 'build/cppcheck/',
              reportFiles: '*',
              reportName: 'Cppcheck'
            ]
            sh 'make distclean'
          }
          when {
            anyOf {
              branch 'master'
              branch 'beta'
              branch 'stable'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('check stack') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make px4fmu-v2_default stack_check'
            sh 'make distclean'
          }
        }

        stage('code coverage (mission test)') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'ulimit -c unlimited; make tests_mission_coverage'
            withCredentials([string(credentialsId: 'FIRMWARE_CODECOV_TOKEN', variable: 'CODECOV_TOKEN')]) {
              sh 'curl -s https://codecov.io/bash | bash -s - -F mission'
            }
            sh 'make distclean'
          }
        }

        stage('code coverage (unit tests)') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ulimit -c unlimited; make tests_coverage'
            withCredentials([string(credentialsId: 'FIRMWARE_CODECOV_TOKEN', variable: 'CODECOV_TOKEN')]) {
              sh 'curl -s https://codecov.io/bash | bash -s - -F unittests'
            }
            sh 'make distclean'
          }
          post {
            failure {
              sh('ls -a')
              sh('find . -name core')
              sh('gdb --batch --quiet -ex "thread apply all bt full" -ex "quit" build/posix_sitl_default/px4 core || true') // always pass for now
            }
          }
        }

      } // parallel
    } // stage Analysis

    stage('Test') {
      parallel {

        stage('unit tests') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'make posix_sitl_default test_results_junit'
            sh 'ccache -s'
            junit 'build/posix_sitl_default/JUnitTestResults.xml'
            sh 'make distclean'
          }
        }

        stage('unit tests (address sanitizer)') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          environment {
              PX4_ASAN = 1
              ASAN_OPTIONS = "color=always:check_initialization_order=1:detect_stack_use_after_return=1"
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'make tests || true' // always pass for now, TODO: PX4 sitl clean shutdown
            sh 'ccache -s'
            sh 'make distclean'
          }
        }

      } // parallel
    } // stage Test

    stage('Generate Metadata') {

      parallel {

        stage('airframe') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-07-19' }
          }
          steps {
            sh 'make distclean'
            sh 'make airframe_metadata'
            archiveArtifacts(artifacts: 'airframes.md, airframes.xml', fingerprint: true)
            sh 'make distclean'
          }
        }

        stage('parameter') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-07-19' }
          }
          steps {
            sh 'make distclean'
            sh 'make parameters_metadata'
            archiveArtifacts(artifacts: 'parameters.md, parameters.xml', fingerprint: true)
            sh 'make distclean'
          }
        }

        stage('module') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-07-19' }
          }
          steps {
            sh 'make distclean'
            sh 'make module_documentation'
            archiveArtifacts(artifacts: 'modules/*.md', fingerprint: true)
            sh 'make distclean'
          }
        }

        stage('uorb graphs') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-07-19'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make uorb_graphs'
            archiveArtifacts(artifacts: 'Tools/uorb_graph/graph_sitl.json')
            sh 'make distclean'
          }
        }

      } // parallel
    } // stage: Generate Metadata

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
