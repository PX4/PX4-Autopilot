#!/usr/bin/env groovy

pipeline {
  agent none
  stages {

    stage('Analysis') {

      parallel {

        stage('Catkin build on ROS workspace') {
          agent {
            docker {
              image 'px4io/px4-dev-ros-melodic:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'ls -l'
            sh '''#!/bin/bash -l
              echo $0;
              mkdir -p catkin_ws/src;
              cd catkin_ws;
              git -C ${WORKSPACE}/catkin_ws/src/Firmware submodule update --init --recursive --force Tools/sitl_gazebo
              git clone --recursive ${WORKSPACE}/catkin_ws/src/Firmware/Tools/sitl_gazebo src/mavlink_sitl_gazebo;
              source /opt/ros/melodic/setup.bash;
              catkin init;
              catkin build -j$(nproc) -l$(nproc);
            '''
            // test if the binary was correctly installed and runs using 'mavros_posix_silt.launch'
            sh '''#!/bin/bash -l
              echo $0;
              source catkin_ws/devel/setup.bash;
              rostest px4 pub_test.launch;
            '''
          }
          post {
            always {
              sh 'rm -rf catkin_ws'
            }
          }
          options {
            checkoutToSubdirectory('catkin_ws/src/Firmware')
          }
        }

        stage('Colcon build on ROS2 workspace') {
          agent {
            docker {
              image 'px4io/px4-dev-ros2-bouncy:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw -e HOME=$WORKSPACE'
            }
          }
          steps {
            sh 'ls -l'
            sh '''#!/bin/bash -l
              echo $0;
              unset ROS_DISTRO;
              mkdir -p colcon_ws/src;
              cd colcon_ws;
              git -C ${WORKSPACE}/colcon_ws/src/Firmware submodule update --init --recursive --force Tools/sitl_gazebo
              git clone --recursive ${WORKSPACE}/colcon_ws/src/Firmware/Tools/sitl_gazebo src/mavlink_sitl_gazebo;
              source /opt/ros/bouncy/setup.sh;
              source /opt/ros/melodic/setup.sh;
              colcon build --event-handlers console_direct+ --symlink-install;
            '''
          }
          post {
            always {
              sh 'rm -rf colcon_ws'
            }
          }
          options {
            checkoutToSubdirectory('colcon_ws/src/Firmware')
          }
        }

        stage('Style check') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh 'make check_format'
          }
          post {
            always {
              sh 'rm -rf catkin_ws'
            }
          }
        }

        stage('Bloaty px4_fmu-v2') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'make px4_fmu-v2_default'
            sh 'make px4_fmu-v2_default bloaty_symbols'
            sh 'make px4_fmu-v2_default bloaty_compileunits'
            sh 'make px4_fmu-v2_default bloaty_inlines'
            sh 'make px4_fmu-v2_default bloaty_templates'
            sh 'make px4_fmu-v2_default bloaty_compare_master'
            sh 'make sizes'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Bloaty px4_fmu-v5') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'make px4_fmu-v5_default'
            sh 'make px4_fmu-v5_default bloaty_symbols'
            sh 'make px4_fmu-v5_default bloaty_compileunits'
            sh 'make px4_fmu-v5_default bloaty_inlines'
            sh 'make px4_fmu-v5_default bloaty_templates'
            sh 'make px4_fmu-v5_default bloaty_compare_master'
            sh 'make sizes'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('No-ninja px4_fmu-v2') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'NO_NINJA_BUILD=1 make px4_fmu-v2_default'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('No-ninja px4_fmu-v5') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'NO_NINJA_BUILD=1 make px4_fmu-v5_default'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('No-ninja SITL build') {
          agent {
            docker {
              image 'px4io/px4-dev-base-bionic:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'NO_NINJA_BUILD=1 make px4_sitl_default'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('SITL unit tests') {
          agent {
            docker {
              image 'px4io/px4-dev-base-bionic:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'ccache -z'
            sh 'git fetch --tags'
            sh 'make tests'
            sh 'ccache -s'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Clang analyzer') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2019-03-08'
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
          post {
            always {
              sh 'make distclean'
            }
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

        // stage('Clang tidy') {
        //   agent {
        //     docker {
        //       image 'px4io/px4-dev-clang:2019-03-08'
        //       args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
        //     }
        //   }
        //   steps {
        //     sh 'export'
        //     retry (3) {
        //       sh 'make distclean'
        //       sh 'make clang-tidy-quiet'
        //     }
        //   }
        //   post {
        //     always {
        //       sh 'make distclean'
        //     }
        //   }
        // }

        stage('Cppcheck') {
          agent {
            docker {
              image 'px4io/px4-dev-base-bionic:2019-03-08'
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
          post {
            always {
              sh 'make distclean'
            }
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

        stage('Check stack') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make px4_fmu-v2_default stack_check'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('ShellCheck') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make shellcheck_all'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Module config validation') {
          agent {
            docker {
              image 'px4io/px4-dev-base-bionic:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make validate_module_configs'
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

      } // parallel
    } // stage Analysis

    stage('Generate Metadata') {

      parallel {

        stage('Airframe') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh 'make distclean'
            sh 'make airframe_metadata'
            dir('build/px4_sitl_default/docs') {
              archiveArtifacts(artifacts: 'airframes.md, airframes.xml')
              stash includes: 'airframes.md, airframes.xml', name: 'metadata_airframes'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Parameter') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh 'make distclean'
            sh 'make parameters_metadata'
            dir('build/px4_sitl_default/docs') {
              archiveArtifacts(artifacts: 'parameters.md, parameters.xml')
              stash includes: 'parameters.md, parameters.xml', name: 'metadata_parameters'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('Module') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh 'make distclean'
            sh 'make module_documentation'
            dir('build/px4_sitl_default/docs') {
              archiveArtifacts(artifacts: 'modules/*.md')
              stash includes: 'modules/*.md', name: 'metadata_module_documentation'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

        stage('uORB graphs') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2019-03-08'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make uorb_graphs'
            dir('Tools/uorb_graph') {
              archiveArtifacts(artifacts: 'graph_px4_sitl.json')
              stash includes: 'graph_px4_sitl.json', name: 'uorb_graph'
            }
          }
          post {
            always {
              sh 'make distclean'
            }
          }
        }

      } // parallel
    } // stage: Generate Metadata

    stage('Deploy') {

      parallel {

        stage('Devguide') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            unstash 'metadata_module_documentation'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/Devguide.git')
              sh('cp airframes.md Devguide/en/airframes/airframe_reference.md')
              sh('cp parameters.md Devguide/en/advanced/parameter_reference.md')
              sh('cp -R modules/*.md Devguide/en/middleware/')
              sh('cd Devguide; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd Devguide; git push origin master || true')
              sh('rm -rf Devguide')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('Userguide') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_user_guide.git')
              sh('cp airframes.md px4_user_guide/en/airframes/airframe_reference.md')
              sh('cp parameters.md px4_user_guide/en/advanced_config/parameter_reference.md')
              sh('cd px4_user_guide; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd px4_user_guide; git push origin master || true')
              sh('rm -rf px4_user_guide')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('QGroundControl') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/mavlink/qgroundcontrol.git')
              sh('cp airframes.xml qgroundcontrol/src/AutoPilotPlugins/PX4/AirframeFactMetaData.xml')
              sh('cp parameters.xml qgroundcontrol/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml')
              sh('cd qgroundcontrol; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd qgroundcontrol; git push origin master || true')
              sh('rm -rf qgroundcontrol')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

        stage('PX4 ROS msgs') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh('export')
            sh('make distclean')
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh("git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_msgs.git")
              // 'master' branch
              sh('python msg/tools/uorb_to_ros_msgs.py msg/ px4_msgs/msg/')
              sh('cd px4_msgs; git status; git add .; git commit -a -m "Update message definitions `date`" || true')
              sh('cd px4_msgs; git push origin master || true')
              // 'ros1' branch
              sh('cd px4_msgs; git checkout ros1')
              sh('python msg/tools/uorb_to_ros_msgs.py msg/ px4_msgs/msg/')
              sh('cd px4_msgs; git status; git add .; git commit -a -m "Update message definitions `date`" || true')
              sh('cd px4_msgs; git push origin ros1 || true')
              sh('rm -rf px4_msgs')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('PX4 ROS2 bridge') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh('export')
            sh('make distclean')
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh("git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_ros_com.git -b ${BRANCH_NAME}")
              // deploy uORB RTPS ID map
              sh('python msg/tools/uorb_to_ros_rtps_ids.py -i msg/tools/uorb_rtps_message_ids.yaml -o px4_ros_com/templates/uorb_rtps_message_ids.yaml')
              sh('cd px4_ros_com; git status; git add .; git commit -a -m "Update uORB RTPS ID map `date`" || true')
              sh('cd px4_ros_com; git push origin ${BRANCH_NAME} || true')
              // deploy uORB RTPS required tools
              sh('cp msg/tools/uorb_rtps_classifier.py px4_ros_com/scripts/uorb_rtps_classifier.py')
              sh('cp msg/tools/generate_microRTPS_bridge.py px4_ros_com/scripts/generate_microRTPS_bridge.py')
              sh('cp msg/tools/px_generate_uorb_topic_files.py px4_ros_com/scripts/px_generate_uorb_topic_files.py')
              sh('cp msg/tools/px_generate_uorb_topic_helper.py px4_ros_com/scripts/px_generate_uorb_topic_helper.py')
              sh('cd px4_ros_com; git status; git add .; git commit -a -m "Update uORB RTPS script tools `date`" || true')
              sh('cd px4_ros_com; git push origin ${BRANCH_NAME} || true')
              sh('rm -rf px4_msgs')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
        }

        stage('S3') {
          agent {
            docker { image 'px4io/px4-dev-base-bionic:2019-03-08' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            sh('ls')
            withAWS(credentials: 'px4_aws_s3_key', region: 'us-east-1') {
              s3Upload(acl: 'PublicRead', bucket: 'px4-travis', file: 'airframes.xml', path: 'Firmware/master/')
              s3Upload(acl: 'PublicRead', bucket: 'px4-travis', file: 'parameters.xml', path: 'Firmware/master/')
            }
          }
          when {
            anyOf {
              branch 'master'
              branch 'pr-jenkins' // for testing
            }
          }
          options {
            skipDefaultCheckout()
          }
        }

      } // parallel
    } // stage: Generate Metadata

  } // stages

  environment {
    CCACHE_DIR = '/tmp/ccache'
    CI = true
    GIT_AUTHOR_EMAIL = "bot@px4.io"
    GIT_AUTHOR_NAME = "PX4BuildBot"
    GIT_COMMITTER_EMAIL = "bot@px4.io"
    GIT_COMMITTER_NAME = "PX4BuildBot"
  }
  options {
    buildDiscarder(logRotator(numToKeepStr: '20', artifactDaysToKeepStr: '30'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
