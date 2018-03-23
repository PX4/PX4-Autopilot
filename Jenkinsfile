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
                  docker.image('px4io/px4-dev-nuttx:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
                      sh "ccache -z"
                      sh "git fetch --tags"
                      sh "make nuttx_${node_name}_default"
                      // bloaty output and compare with last successful master
                      sh "bloaty -n 100 -d symbols -s file build/nuttx_${node_name}_default/nuttx_${node_name}_default.elf"
                      sh "bloaty -n 100 -d compileunits -s file build/nuttx_${node_name}_default/nuttx_${node_name}_default.elf"
                      sh "wget --no-verbose -N https://s3.amazonaws.com/px4-travis/Firmware/master/nuttx_${node_name}_default.elf"
                      sh "bloaty -d symbols -C full -s file build/nuttx_${node_name}_default/nuttx_${node_name}_default.elf -- nuttx_${node_name}_default.elf"
                      sh "make nuttx_${node_name}_rtps"
                      sh "make sizes"
                      sh "ccache -s"
                      archiveArtifacts(artifacts: 'build/*/*.px4', fingerprint: true)
                      archiveArtifacts(artifacts: 'build/*/*.elf', fingerprint: true)
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
                docker.image('px4io/px4-dev-nuttx:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                  stage("${node_name}") {
                    checkout scm
                    sh "export"
                    sh "make distclean"
                    sh "ccache -z"
                    sh "git fetch --tags"
                    sh "make px4io-v2_default"
                    sh "make nuttx_px4fmu-v2_default"
                    // bloaty output and compare with last successful master
                    sh "bloaty -n 100 -d symbols -s file build/nuttx_px4fmu-v2_default/nuttx_px4fmu-v2_default.elf"
                    sh "bloaty -n 100 -d compileunits -s file build/nuttx_px4fmu-v2_default/nuttx_px4fmu-v2_default.elf"
                    sh "wget --no-verbose -N https://s3.amazonaws.com/px4-travis/Firmware/master/nuttx_px4fmu-v2_default.elf"
                    sh "bloaty -d symbols -C full -s file build/nuttx_px4fmu-v2_default/nuttx_px4fmu-v2_default.elf -- nuttx_px4fmu-v2_default.elf"
                    sh "make nuttx_px4fmu-v2_lpe"
                    sh "make nuttx_px4fmu-v3_default"
                    sh "make nuttx_px4fmu-v3_rtps"
                    sh "make sizes"
                    sh "ccache -s"
                    archiveArtifacts(artifacts: 'build/*/*.px4', fingerprint: true)
                    archiveArtifacts(artifacts: 'build/*/*.elf', fingerprint: true)
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
                  docker.image('px4io/px4-dev-nuttx:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
                      sh "ccache -z"
                      sh "git fetch --tags"
                      sh "make nuttx_${node_name}_default"
                      sh "make sizes"
                      sh "ccache -s"
                      archiveArtifacts(artifacts: 'build/*/*.px4', fingerprint: true)
                      archiveArtifacts(artifacts: 'build/*/*.elf', fingerprint: true)
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
                  docker.image('px4io/px4-dev-nuttx:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
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
                  docker.image('px4io/px4-dev-base:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
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
                  docker.image('px4io/px4-dev-raspi:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
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
                  docker.image('px4io/px4-dev-armhf:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
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
                    docker.image("lorenzmeier/px4-dev-snapdragon:2017-12-29").inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                      stage("${node_name}") {
                        checkout scm
                        sh "export"
                        sh "make distclean"
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
                  docker.image('px4io/px4-dev-base-archlinux:2017-12-30').inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                    stage("${node_name}") {
                      checkout scm
                      sh "export"
                      sh "make distclean"
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
              image 'px4io/px4-dev-base:2017-12-30'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make check_format'
          }
        }

        stage('clang analyzer') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-12-30'
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
          }
          when {
            anyOf {
              branch 'master'
              branch 'beta'
              branch 'stable'
            }
          }
        }

        stage('clang tidy') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-12-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make clang-tidy-quiet'
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
          }
          when {
            anyOf {
              branch 'master'
              branch 'beta'
              branch 'stable'
            }
          }
        }

        stage('tests') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-12-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make posix_sitl_default test_results_junit'
            junit 'build/posix_sitl_default/JUnitTestResults.xml'
          }
        }

        stage('ROS vtol mission new 1') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_new_1 vehicle:=standard_vtol'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_new_1: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_new_1: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission new 2') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_new_2 vehicle:=standard_vtol'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_new_2: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_new_2: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission old 1') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_old_1 vehicle:=standard_vtol'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_old_1: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_old_1: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission old 2') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_old_2 vehicle:=standard_vtol'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_old_2: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS mission test vtol_old_2: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission old 3') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            //sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            //sh 'git fetch --tags'
            //sh 'make posix_sitl_default'
            //sh 'make posix_sitl_default sitl_gazebo'
            //sh './test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_old_3 vehicle:=standard_vtol'
          }
          post {
            //success {
            //  sh './Tools/upload_log.py -q --description "ROS mission test vtol_old_3: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            //}
            failure {
              //sh './Tools/upload_log.py -q --description "ROS mission test vtol_old_3: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS MC mission box') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=multirotor_box vehicle:=iris'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS mission test multirotor_box: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS mission test multirotor_box: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS offboard att') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_tests_offboard_attctl.test'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS offboard attitude test: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS offboard attitude test: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS offboard pos') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2017-12-31'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'git fetch --tags'
            sh 'make posix_sitl_default'
            sh 'make posix_sitl_default sitl_gazebo'
            sh './test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test'
          }
          post {
            success {
              sh './Tools/upload_log.py -q --description "ROS offboard position test: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
            }
            failure {
              sh './Tools/upload_log.py -q --description "ROS offboard position test: ${CHANGE_ID}" --feedback "${CHANGE_TITLE} - ${CHANGE_URL}" --source CI --email "${CHANGE_AUTHOR_EMAIL}" .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

      }
    }

    stage('Generate Metadata') {

      parallel {

        stage('airframe') {
          agent {
            docker { image 'px4io/px4-dev-base:2017-12-30' }
          }
          steps {
            sh 'make distclean'
            sh 'make airframe_metadata'
            archiveArtifacts(artifacts: 'airframes.md, airframes.xml', fingerprint: true)
          }
        }

        stage('parameter') {
          agent {
            docker { image 'px4io/px4-dev-base:2017-12-30' }
          }
          steps {
            sh 'make distclean'
            sh 'make parameters_metadata'
            archiveArtifacts(artifacts: 'parameters.md, parameters.xml', fingerprint: true)
          }
        }

        stage('module') {
          agent {
            docker { image 'px4io/px4-dev-base:2017-12-30' }
          }
          steps {
            sh 'make distclean'
            sh 'make module_documentation'
            archiveArtifacts(artifacts: 'modules/*.md', fingerprint: true)
          }
        }

        stage('uorb graphs') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2017-12-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make uorb_graphs'
            archiveArtifacts(artifacts: 'Tools/uorb_graph/graph_sitl.json')
          }
        }
      }
    }

    stage('S3 Upload') {
      agent {
        docker { image 'px4io/px4-dev-base:2017-12-30' }
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
  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '5'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
