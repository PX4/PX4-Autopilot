pipeline {
  agent none
  stages {

    stage('Build') {
      steps {
        script {
          def builds = [:]

          def docker_base = "px4io/px4-dev-base:2018-03-30"
          def docker_nuttx = "px4io/px4-dev-nuttx:2018-03-30"
          def docker_ros = "px4io/px4-dev-ros:2018-03-30"
          def docker_rpi = "px4io/px4-dev-raspi:2018-03-30"
          def docker_armhf = "px4io/px4-dev-armhf:2017-12-30"
          def docker_arch = "px4io/px4-dev-base-archlinux:2018-03-30"
          def docker_snapdragon = "lorenzmeier/px4-dev-snapdragon:2017-12-29"
          def docker_clang = "px4io/px4-dev-clang:2018-03-30"

          // fmu-v2_{default, lpe} and fmu-v3_{default, rtps}
          // bloaty compare to last successful master build
          builds["px4fmu-v2"] = {
            node {
              stage("Build Test px4fmu-v2") {
                docker.image(docker_nuttx).inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
                  stage("px4fmu-v2") {
                    checkout scm
                    sh "export"
                    sh "make distclean"
                    sh "ccache -z"
                    sh "git fetch --tags"
                    sh "make nuttx_px4io-v2_default"
                    sh "make nuttx_px4io-v2_default bloaty_symbols"
                    sh "make nuttx_px4io-v2_default bloaty_compileunits"
                    sh "make nuttx_px4io-v2_default bloaty_compare_master"
                    sh "make nuttx_px4fmu-v2_default"
                    sh "make nuttx_px4fmu-v2_default bloaty_symbols"
                    sh "make nuttx_px4fmu-v2_default bloaty_compileunits"
                    sh "make nuttx_px4fmu-v2_default bloaty_inlines"
                    sh "make nuttx_px4fmu-v2_default bloaty_templates"
                    sh "make nuttx_px4fmu-v2_default bloaty_compare_master"
                    sh "make nuttx_px4fmu-v2_lpe"
                    sh "make nuttx_px4fmu-v2_test"
                    sh "make nuttx_px4fmu-v3_default"
                    sh "make nuttx_px4fmu-v3_rtps"
                    sh "make sizes"
                    sh "ccache -s"
                    archiveArtifacts(allowEmptyArchive: false, artifacts: 'build/**/*.px4, build/**/*.elf', fingerprint: true, onlyIfSuccessful: true)
                    sh "make distclean"
                  }
                }
              }
            }
          }

          // nuttx default targets that are archived and uploaded to s3
          for (def option in ["px4fmu-v4", "px4fmu-v4pro", "px4fmu-v5", "aerofc-v1", "aerocore2", "auav-x21", "crazyflie", "mindpx-v2", "nxphlite-v3", "tap-v1", "omnibus-f4sd"]) {
            def node_name = "${option}"
            builds[node_name] = createBuildNodeArchive(docker_nuttx, "${node_name}_default")
          }

          // other nuttx default targets
          for (def option in ["px4-same70xplained-v1", "px4-stm32f4discovery", "px4cannode-v1", "px4esc-v1", "px4nucleoF767ZI-v1", "s2740vc-v1"]) {
            def node_name = "${option}"
            builds[node_name] = createBuildNode(docker_nuttx, "${node_name}_default")
          }

          builds["sitl_rtps"] = createBuildNode(docker_base, 'posix_sitl_rtps')
          builds["sitl (GCC 7)"] = createBuildNode(docker_arch, 'posix_sitl_default')

          builds["rpi"] = createBuildNode(docker_rpi, 'posix_rpi_cross')
          builds["bebop"] = createBuildNode(docker_rpi, 'posix_bebop_default')

          builds["ocpoc"] = createBuildNode(docker_armhf, 'posix_ocpoc_ubuntu')

          // snapdragon (eagle_default)
          builds["eagle (linux)"] = createBuildNodeDockerLogin(docker_snapdragon, 'docker_hub_dagar', 'posix_eagle_default')
          builds["eagle (qurt)"] = createBuildNodeDockerLogin(docker_snapdragon, 'docker_hub_dagar', 'qurt_eagle_default')

          // posix_sitl_default with package
          builds["sitl"] = {
            node {
              stage("Build Test sitl") {
                docker.image(docker_ros).inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE') {
                  stage("sitl") {
                    checkout scm
                    sh "export"
                    sh "make distclean"
                    sh "ccache -z"
                    sh "make posix_sitl_default"
                    sh "make posix_sitl_default sitl_gazebo"
                    sh "make posix_sitl_default package"
                    sh "ccache -s"
                    stash name: "px4_sitl_package", includes: "build/posix_sitl_default/*.bz2"
                    sh "make distclean"
                  }
                }
              }
            }
          }

          // MAC OS posix_sitl_default
          builds["sitl (OSX)"] = {
            node("mac") {
              withEnv(["CCACHE_BASEDIR=${pwd()}"]) {
                stage("sitl (OSX)") {
                  checkout scm
                  sh "export"
                  sh "make distclean"
                  sh "ccache -z"
                  sh "make posix_sitl_default"
                  sh "ccache -s"
                  sh "make distclean"
                }
              }
            }
          }

          // MAC OS nuttx_px4fmu-v4pro_default
          builds["px4fmu-v4pro (OSX)"] = {
            node("mac") {
              withEnv(["CCACHE_BASEDIR=${pwd()}"]) {
                stage("px4fmu-v4pro (OSX)") {
                  checkout scm
                  sh "export"
                  sh "make distclean"
                  sh "ccache -z"
                  sh "make nuttx_px4fmu-v4pro_default"
                  sh "ccache -s"
                  sh "make distclean"
                }
              }
            }
          }

          parallel builds
        } // script
      } // steps
    } // stage Builds

    stage('Test') {
      parallel {

        stage('Style Check') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-03-30' }
          }

          steps {
            sh 'make check_format'
          }
        }

        stage('clang analyzer') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2018-03-30'
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
            }
          }
        }

        stage('tests') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make posix_sitl_default test_results_junit'
            junit 'build/posix_sitl_default/JUnitTestResults.xml'
            sh 'make distclean'
          }
        }

        stage('test mission (code coverage)') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean; rm -rf .ros; rm -rf .gazebo'
            sh 'make tests_mission_coverage'
            withCredentials([string(credentialsId: 'FIRMWARE_CODECOV_TOKEN', variable: 'CODECOV_TOKEN')]) {
              sh 'curl -s https://codecov.io/bash | bash -s'
            }
            sh 'make distclean'
          }
        }

        // TODO: PX4 requires clean shutdown first
        // stage('tests (address sanitizer)') {
        //   agent {
        //     docker {
        //       image 'px4io/px4-dev-base:2018-03-30'
        //       args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
        //     }
        //   }
        //   environment {
        //       PX4_ASAN = 1
        //       ASAN_OPTIONS = "color=always:check_initialization_order=1:detect_stack_use_after_return=1"
        //   }
        //   steps {
        //     sh 'export'
        //     sh 'make distclean'
        //     sh 'make tests'
        //     sh 'make distclean'
        //   }
        // }

        // TODO: test and re-enable once GDB is available in px4-dev-ros
        // stage('tests (code coverage)') {
        //   agent {
        //     docker {
        //       image 'px4io/px4-dev-ros:2018-03-30'
        //       args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
        //     }
        //   }
        //   steps {
        //     sh 'export'
        //     sh 'make distclean'
        //     sh 'ulimit -c unlimited; make tests_coverage'
        //     sh 'ls'
        //     withCredentials([string(credentialsId: 'FIRMWARE_CODECOV_TOKEN', variable: 'CODECOV_TOKEN')]) {
        //       sh 'curl -s https://codecov.io/bash | bash -s'
        //     }
        //     sh 'make distclean'
        //   }
        //   post {
        //     failure {
        //       sh('find . -name core')
        //       sh('gdb --batch --quiet -ex "thread apply all bt full" -ex "quit" build/posix_sitl_default/px4 core')
        //     }
        //   }
        // }

        stage('check stack') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-03-30'
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

        stage('ROS vtol standard mission') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_new_1 vehicle:=standard_vtol'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol tailsitter mission') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_new_1 vehicle:=tailsitter'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol tiltrotor mission') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_new_1 vehicle:=tiltrotor'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission new 2') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_new_2 vehicle:=standard_vtol'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission old 1') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_old_1 vehicle:=standard_vtol'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS vtol mission old 2') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=vtol_old_2 vehicle:=standard_vtol'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS MC mission box') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=multirotor_box vehicle:=iris'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS offboard att') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          options {
            skipDefaultCheckout()
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_tests_offboard_attctl.test'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
              archiveArtifacts '.ros/**/*.ulg'
              archiveArtifacts '.ros/**/rosunit-*.xml'
              archiveArtifacts '.ros/**/rostest-*.log'
            }
          }
        }

        stage('ROS offboard pos') {
          agent {
            docker {
              image 'px4io/px4-dev-ros:2018-03-30'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'export'
            sh 'rm -rf build; rm -rf px4-posix_sitl_default*; rm -rf .ros; rm -rf .gazebo'
            unstash 'px4_sitl_package'
            sh 'tar -xjpvf build/posix_sitl_default/px4-posix_sitl_default*.bz2'
            sh 'px4-posix_sitl_default*/px4/test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test'
            sh 'px4-posix_sitl_default*/px4/Tools/ecl_ekf/process_logdata_ekf.py `find . -name *.ulg -print -quit`'
          }
          post {
            always {
              sh 'px4-posix_sitl_default*/px4/Tools/upload_log.py -q --description "${JOB_NAME}: ${STAGE_NAME}" --feedback "${JOB_NAME} ${CHANGE_TITLE} ${CHANGE_URL}" --source CI .ros/rootfs/fs/microsd/log/*/*.ulg'
              archiveArtifacts '.ros/**/*.pdf'
              archiveArtifacts '.ros/**/*.csv'
              deleteDir()
            }
            failure {
              sh 'ls -a'
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
            docker { image 'px4io/px4-dev-base:2018-03-30' }
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
            docker { image 'px4io/px4-dev-base:2018-03-30' }
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
            docker { image 'px4io/px4-dev-base:2018-03-30' }
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
              image 'px4io/px4-dev-nuttx:2018-03-30'
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
      }
    }

    // TODO: actually upload artifacts to S3
    stage('S3 Upload') {
      agent {
        docker { image 'px4io/px4-dev-base:2018-03-30' }
      }
      options {
            skipDefaultCheckout()
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

def createBuildNode(String docker_repo, String target) {
  return {
    node {
      docker.image(docker_repo).inside('-e CCACHE_BASEDIR=${WORKSPACE} -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
        stage(target) {
          sh('export')
          checkout scm
          sh('make distclean')
          sh('git fetch --tags')
          sh('ccache -z')
          sh('make ' + target)
          sh('ccache -s')
          sh('make sizes')
          sh('make distclean')
        }
      }
    }
  }
}

def createBuildNodeArchive(String docker_repo, String target) {
  return {
    node {
      docker.image(docker_repo).inside('-e CCACHE_BASEDIR=${WORKSPACE} -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
        stage(target) {
          sh('export')
          checkout scm
          sh('make distclean')
          sh('git fetch --tags')
          sh('ccache -z')
          sh('make ' + target)
          sh('ccache -s')
          sh('make sizes')
          archiveArtifacts(allowEmptyArchive: false, artifacts: 'build/**/*.px4, build/**/*.elf, build/**/*.bin', fingerprint: true, onlyIfSuccessful: true)
          sh('make distclean')
        }
      }
    }
  }
}

def createBuildNodeDockerLogin(String docker_repo, String docker_credentials, String target) {
  return {
    node {
      docker.withRegistry('https://registry.hub.docker.com', docker_credentials) {
        docker.image(docker_repo).inside('-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw') {
          stage(target) {
            sh('export')
            checkout scm
            sh('make distclean')
            sh('git fetch --tags')
            sh('ccache -z')
            sh('make ' + target)
            sh('ccache -s')
            sh('make sizes')
            sh('make distclean')
          }
        }
      }
    }
  }
}
