pipeline {
  agent none
  stages {

    stage('Analysis') {

      parallel {

        stage('Style Check') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-09-11' }
          }

          steps {
            sh 'make check_format'
          }
        }

        stage('bloaty px4fmu-v2') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-09-11'
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
              image 'px4io/px4-dev-nuttx:2018-09-11'
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
              image 'px4io/px4-dev-clang:2018-09-11'
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
              image 'px4io/px4-dev-base:2018-09-11'
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
              image 'px4io/px4-dev-nuttx:2018-09-11'
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

        stage('ShellCheck') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-09-11'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make shellcheck_all'
            sh 'make distclean'
          }
        }

        stage('Module Config Validation') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2018-09-11'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make validate_module_configs'
          }
        }

      } // parallel
    } // stage Analysis

    stage('Generate Metadata') {

      parallel {

        stage('airframe') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-09-11' }
          }
          steps {
            sh 'make distclean'
            sh 'make airframe_metadata'
            dir('build/posix_sitl_default/docs') {
              archiveArtifacts(artifacts: 'airframes.md, airframes.xml')
              stash includes: 'airframes.md, airframes.xml', name: 'metadata_airframes'
            }
            sh 'make distclean'
          }
        }

        stage('parameter') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-09-11' }
          }
          steps {
            sh 'make distclean'
            sh 'make parameters_metadata'
            dir('build/posix_sitl_default/docs') {
              archiveArtifacts(artifacts: 'parameters.md, parameters.xml')
              stash includes: 'parameters.md, parameters.xml', name: 'metadata_parameters'
            }
            sh 'make distclean'
          }
        }

        stage('module') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-09-11' }
          }
          steps {
            sh 'make distclean'
            sh 'make module_documentation'
            dir('build/posix_sitl_default/docs') {
              archiveArtifacts(artifacts: 'modules/*.md')
              stash includes: 'modules/*.md', name: 'metadata_module_documentation'
            }
            sh 'make distclean'
          }
        }

        stage('uorb graphs') {
          agent {
            docker {
              image 'px4io/px4-dev-nuttx:2018-09-11'
              args '-e CCACHE_BASEDIR=$WORKSPACE -v ${CCACHE_DIR}:${CCACHE_DIR}:rw'
            }
          }
          steps {
            sh 'export'
            sh 'make distclean'
            sh 'make uorb_graphs'
            dir('Tools/uorb_graph') {
              archiveArtifacts(artifacts: 'graph_sitl.json')
              stash includes: 'graph_sitl.json', name: 'uorb_graph'
            }
            sh 'make distclean'
          }
        }

      } // parallel
    } // stage: Generate Metadata

    stage('Deploy') {

      parallel {

        stage('Devguide') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-09-11' }
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
              sh('cd Devguide; git checkout -B pr-firmware_metadata_update; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd Devguide; git push --force origin pr-firmware_metadata_update || true')
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
            docker { image 'px4io/px4-dev-base:2018-09-11' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/px4_user_guide.git')
              sh('cp airframes.md px4_user_guide/en/airframes/airframe_reference.md')
              sh('cp parameters.md px4_user_guide/en/advanced_config/parameter_reference.md')
              sh('cd px4_user_guide; git checkout -B pr-firmware_metadata_update; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd px4_user_guide; git push --force origin pr-firmware_metadata_update || true')
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
            docker { image 'px4io/px4-dev-base:2018-09-11' }
          }
          steps {
            sh('export')
            unstash 'metadata_airframes'
            unstash 'metadata_parameters'
            withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
              sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/mavlink/qgroundcontrol.git')
              sh('cp airframes.xml qgroundcontrol/src/AutoPilotPlugins/PX4/AirframeFactMetaData.xml')
              sh('cp parameters.xml qgroundcontrol/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.xml')
              sh('cd qgroundcontrol; git checkout -B pr-firmware_metadata_update; git status; git add .; git commit -a -m "Update PX4 Firmware metadata `date`" || true')
              sh('cd qgroundcontrol; git push --force origin pr-firmware_metadata_update || true')
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

        stage('S3') {
          agent {
            docker { image 'px4io/px4-dev-base:2018-09-11' }
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
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '30'))
    timeout(time: 60, unit: 'MINUTES')
  }
}
