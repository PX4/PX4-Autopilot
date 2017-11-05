pipeline {
  agent none
  stages {
    stage('Quality Checks') {
      agent { docker 'px4io/px4-dev-base:2017-08-29' }
      steps {
        sh 'make check_format'
      }
    }
    stage('Build') {
        stage('nuttx_px4fmu-v2_default') {
          agent { docker 'px4io/px4-dev-nuttx:2017-08-29' }
          steps {
            sh 'ccache -s; make nuttx_px4fmu-v2_default; ccache -s'
            archiveArtifacts 'build/*/*.px4'
          }
        }
        stage('nuttx_px4fmu-v5_default') {
          agent { docker 'px4io/px4-dev-nuttx:2017-08-29' }
          steps {
            sh 'ccache -s; make nuttx_px4fmu-v5_default; ccache -s'
            archiveArtifacts 'build/*/*.px4'
          }
        }
        stage('posix_rpi_cross') {
          agent { docker 'px4io/px4-dev-raspi:2017-08-29' }
          steps {
            sh 'ccache -s; make posix_rpi_cross; ccache -s'
            archiveArtifacts 'build/*/*.px4'
          }
        }
    }
    stage('Test') {
      agent { docker 'px4io/px4-dev-base:2017-08-29' }
      steps {
        sh 'make posix_sitl_default test_results_junit; ccache -s'
        junit 'build/posix_sitl_default/JUnitTestResults.xml'
      }
    }
    stage('Generate Metadata') {
      agent { docker 'px4io/px4-dev-base:2017-08-29' }
      parallel {
        stage('airframe') {
          steps {
            sh 'make airframe_metadata'
            archiveArtifacts 'airframes.md, airframes.xml'
          }
        }
        stage('parameters') {
          steps {
            sh 'make parameters_metadata'
            archiveArtifacts 'parameters.md, parameters.xml'
          }
        }
        stage('modules') {
          steps {
            sh 'make module_documentation'
            archiveArtifacts 'modules/*.md'
          }
        }
      }
    }
    stage('Deploy') {
      agent { docker 'px4io/px4-dev-base:2017-08-29' }
      when {
        branch 'master|beta|stable'
      }
      steps {
        sh 'echo "uploading to S3"'
      }
    }
  }
}
