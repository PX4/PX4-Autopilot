pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {
          def builds = [:]


          // nuttx default targets that are archived and uploaded to s3
          for (def option in ["px4fmu-v4", "px4fmu-v4pro", "px4fmu-v5", "aerofc-v1"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-nuttx:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "git fetch --tags"
                      sh "make nuttx_${node_name}_default"
                      sh "make nuttx_${node_name}_rtps"
                      sh "make sizes"
                      sh "ccache -s"
                      archiveArtifacts(artifacts: 'build/*/*.px4', fingerprint: true)
                    }
                  }
                }
              }
            }
          }


          // special case for fmu-v2/fmu-v3
          builds["px4fmu-v2"] = {
            node {
              stage("Build Test ${node_name}") {
                docker.image('px4io/px4-dev-nuttx:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                  stage("${node_name}") {
                    checkout scm
                    sh "make clean"
                    sh "ccache -z"
                    sh "git fetch --tags"
                    sh "make px4io-v2_default"
                    sh "make nuttx_px4fmu-v2_default"
                    sh "make nuttx_px4fmu-v2_lpe"
                    sh "make nuttx_px4fmu-v3_default"
                    sh "make nuttx_px4fmu-v3_rtps"
                    sh "make sizes"
                    sh "ccache -s"
                    archiveArtifacts(artifacts: 'build/*/*.px4', fingerprint: true)
                  }
                }
              }
            }
          }


          // nuttx default targets that are archived and uploaded to s3
          for (def option in ["aerocore2", "auav-x21", "crazyflie", "mindpx-v2", "nxphlite-v3", "tap-v1"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-nuttx:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "git fetch --tags"
                      sh "make nuttx_${node_name}_default"
                      sh "make sizes"
                      sh "ccache -s"
                      archiveArtifacts(artifacts: 'build/*/*.px4', fingerprint: true)
                    }
                  }
                }
              }
            }
          }


          // other nuttx default targets
          for (def option in ["px4-same70xplained-v1", "px4-stm32f4discovery", "px4cannode-v1", "px4esc-v1", "px4nucleoF767ZI-v1", "s2740vc-v1"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-nuttx:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "make nuttx_${node_name}_default"
                      sh "make sizes"
                      sh "ccache -s"
                    }
                  }
                }
              }
            }
          }


          // posix_sitl
          for (def option in ["sitl_default", "sitl_rtps"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-base:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "make posix_${node_name}"
                      sh "ccache -s"
                    }
                  }
                }
              }
            }
          }


          // raspberry pi and bebop (armhf)
          for (def option in ["rpi_cross", "bebop_default"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-raspi:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "make posix_${node_name}"
                      sh "ccache -s"
                    }
                  }
                }
              }
            }
          }


          // other armhf (to be merged with raspi and bebop)
          for (def option in ["ocpoc_ubuntu"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-armhf:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "make posix_${node_name}"
                      sh "ccache -s"
                    }
                  }
                }
              }
            }
          }


          // snapdragon eagle (posix + qurt)
          for (def option in ["eagle_default"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.withRegistry('https://registry.hub.docker.com', 'docker_hub_dagar') {
                    docker.image("lorenzmeier/px4-dev-snapdragon:2017-12-29").inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                      stage("${node_name}") {
                        checkout scm
                        sh "make clean"
                        sh "ccache -z"
                        sh "make ${node_name}"
                        sh "ccache -s"
                      }
                    }
                  }
                }
              }
            }
          }


          // GCC7 posix
          for (def option in ["sitl_default"]) {
            def node_name = "${option}"

            builds["${node_name} (GCC7)"] = {
              node {
                stage("Build Test ${node_name} (GCC7)") {
                  docker.image('px4io/px4-dev-base-archlinux:2017-12-29').inside('-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "ccache -z"
                      sh "make posix_${node_name}"
                      sh "ccache -s"
                    }
                  }
                }
              }
            }
          }

          parallel builds
        }
      }
    }

    stage('Test') {
      parallel {

        stage('check style') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-12-29'
              args '-e CI=true'
            }
          }
          steps {
            sh 'make check_format'
          }
        }

        stage('clang analyzer') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-10-23'
              args '-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
            }
          }
          steps {
            sh 'make clean'
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
          }
        }

        stage('clang tidy') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-10-23'
              args '-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
            }
          }
          steps {
            sh 'make clean'
            sh 'make clang-tidy-quiet'
          }
        }

        stage('cppcheck') {
          agent {
            docker {
              image 'px4io/px4-dev-base:ubuntu17.10'
              args '-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
            }
          }
          steps {
            sh 'make clean'
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
          }
        }

        stage('tests') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-12-29'
              args '-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
            }
          }
          steps {
            sh 'make clean'
            sh 'make posix_sitl_default test_results_junit'
            junit 'build/posix_sitl_default/JUnitTestResults.xml'
          }
        }

        // temporarily disabled until stable
        //stage('tests coverage') {
        //  agent {
        //    docker {
        //      image 'px4io/px4-dev-base:2017-12-29'
        //      args '-e CI=true -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
        //    }
        //  }
        //  steps {
        //    sh 'make clean'
        //    sh 'make tests_coverage'
        //    // publish html
        //    publishHTML target: [
        //      allowMissing: false,
        //      alwaysLinkToLastBuild: false,
        //      keepAll: true,
        //      reportDir: 'build/posix_sitl_default/coverage-html',
        //      reportFiles: '*',
        //      reportName: 'Coverage Report'
        //    ]
        //  }
        //}

      }
    }

    stage('Generate Metadata') {

      parallel {

        stage('airframe') {
          agent {
            docker { image 'px4io/px4-dev-base:2017-12-29' }
          }
          steps {
            sh 'make airframe_metadata'
            archiveArtifacts(artifacts: 'airframes.md, airframes.xml', fingerprint: true)
          }
        }

        stage('parameter') {
          agent {
            docker { image 'px4io/px4-dev-base:2017-12-29' }
          }
          steps {
            sh 'make parameters_metadata'
            archiveArtifacts(artifacts: 'parameters.md, parameters.xml', fingerprint: true)
          }
        }

        stage('module') {
          agent {
            docker { image 'px4io/px4-dev-base:2017-12-29' }
          }
          steps {
            sh 'make module_documentation'
            archiveArtifacts(artifacts: 'modules/*.md', fingerprint: true)
          }
        }
      }
    }

    stage('S3 Upload') {
      agent {
        docker { image 'px4io/px4-dev-base:2017-12-29' }
      }

      when {
        anyOf {
          branch 'master'
          branch 'beta'
          branch 'stable'
        }
      }

      steps {
        sh 'echo "uploading to S3"'
      }
    }
  }

  options {
    buildDiscarder(logRotator(numToKeepStr: '5'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
