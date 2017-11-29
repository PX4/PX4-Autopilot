pipeline {
  agent none
  options {
    buildDiscarder(logRotator(numToKeepStr: '10'))
    timeout(time: 60, unit: 'MINUTES')
    timestamps()
  }
  stages {
    stage('Build') {
      steps {
        script {
          def builds = [:]

          // nuttx default targets that are archived and uploaded to s3
          for (def option in ["px4fmu-v2", "px4fmu-v3", "px4fmu-v4", "px4fmu-v4pro", "px4fmu-v5", "aerocore2", "aerofc-v1", "auav-x21", "crazyflie", "mindpx-v2", "tap-v1", "nxphlite-v3"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-nuttx:2017-10-23').inside("--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true") {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "make nuttx_${node_name}_default"
                      sh "ccache -s"
                      archive 'build/*/*.px4'
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
                  docker.image('px4io/px4-dev-nuttx:2017-10-23').inside("--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true") {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "make nuttx_${node_name}_default"
                      sh "ccache -s"
                    }
                  }
                }
              }
            }
          }

          // nuttx non default targets
          for (def option in ["px4fmu-v2_lpe", "px4fmu-v3_rtps", "px4fmu-v4_rtps", "px4fmu-v4pro_rtps", "px4fmu-v5_rtps"]) {
            def node_name = "${option}"

            builds["${node_name}"] = {
              node {
                stage("Build Test ${node_name}") {
                  docker.image('px4io/px4-dev-nuttx:2017-10-23').inside("--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true") {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
                      sh "make nuttx_${node_name}"
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
                  docker.image('px4io/px4-dev-raspi:2017-10-23').inside("--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true") {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
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
                  docker.image('px4io/px4-dev-armhf:2017-10-23').inside("--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true") {
                    stage("${node_name}") {
                      checkout scm
                      sh "make clean"
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
        stage('check_format') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-10-23'
              args '--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true'
            }
          }
          steps {
            sh 'make check_format'
          }
        }
        stage('clang-tidy') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-10-23'
              args '--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true'
            }
          }
          steps {
            sh 'make clean'
            sh 'make clang-tidy-quiet'
          }
        }
        stage('tests') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-10-23'
              args '--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true'
            }
          }
          steps {
            sh 'make clean'
            sh 'make posix_sitl_default test_results_junit'
            junit 'build/posix_sitl_default/JUnitTestResults.xml'
          }
        }
        stage('tests coverage') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-10-23'
              args '--env CCACHE_DIR=/tmp/ccache --volume=/tmp/ccache:/tmp/ccache:rw --env CI=true'
            }
          }
          steps {
            sh 'make clean'
            sh 'make tests_coverage'
            // publish html
            publishHTML target: [
              allowMissing: false,
              alwaysLinkToLastBuild: false,
              keepAll: true,
              reportDir: 'build/posix_sitl_default/coverage-html',
              reportFiles: '*',
              reportName: 'Coverage Report'
            ]
          }
        }
      }
    }
    stage('Generate Metadata') {
      agent {
        docker {
          image 'px4io/px4-dev-base:2017-10-23'
        }
      }
      steps {
        sh 'make airframe_metadata'
        archive 'airframes.md, airframes.xml'
        sh 'make parameters_metadata'
        archive 'parameters.md, parameters.xml'
        sh 'make module_documentation'
        archive 'modules/*.md'
      }
    }
    stage('S3 Upload') {
      agent {
        docker {
          image 'px4io/px4-dev-base:2017-10-23'
        }
      }
      when {
        branch '*/master|*/beta|*/stable'
      }
      steps {
        sh 'echo "uploading to S3"'
      }
    }
  }
}
