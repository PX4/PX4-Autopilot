pipeline {
  agent none
  stages {
    stage('Quality Checks') {
      agent {
        docker {
          image 'px4io/px4-dev-simulation:2017-10-23'
          args '--env CCACHE_DISABLE=1 --env CI=true'
          label 'docker'
        }
      }
      steps {
        sh 'make check_format'
      }
    }
    stage('Build') {
      agent {
        docker {
          image 'px4io/px4-dev-simulation:2017-10-23'
          args '--env CCACHE_DISABLE=1 --env CI=true'
          label 'docker'
        }
      }
      steps {
        sh 'make nuttx_px4fmu-v2_default'
        archiveArtifacts 'build/*/*.px4'
      }
    }
    stage('Test') {
      agent {
        docker {
          image 'px4io/px4-dev-simulation:2017-10-23'
          args '--env CCACHE_DISABLE=1 --env CI=true'
          label 'docker'
        }
      }
      steps {
        sh 'make posix_sitl_default test_results_junit'
        junit 'build/posix_sitl_default/JUnitTestResults.xml'
      }
    }
    stage('Generate Metadata') {
      parallel {
        stage('airframe') {
          agent {
            docker {
              image 'px4io/px4-dev-simulation:2017-10-23'
              args '--env CCACHE_DISABLE=1 --env CI=true'
              label 'docker'
            }
          }
          steps {
            sh 'make airframe_metadata'
            archiveArtifacts 'airframes.md, airframes.xml'
          }
        }
        stage('parameters') {
          agent {
            docker {
              image 'px4io/px4-dev-simulation:2017-10-23'
              args '--env CCACHE_DISABLE=1 --env CI=true'
              label 'docker'
            }
          }
          steps {
            sh 'make parameters_metadata'
            archiveArtifacts 'parameters.md, parameters.xml'
          }
        }
        stage('modules') {
          agent {
            docker {
              image 'px4io/px4-dev-simulation:2017-10-23'
              args '--env CCACHE_DISABLE=1 --env CI=true'
              label 'docker'
            }
          }
          steps {
            sh 'make module_documentation'
            archiveArtifacts 'modules/*.md'
          }
        }
      }
    }
    stage('S3 Upload') {
      agent {
        docker {
          image 'px4io/px4-dev-simulation:2017-10-23'
          args '--env CCACHE_DISABLE=1 --env CI=true'
          label 'docker'
        }
      }
      when {
        branch 'master|beta|stable'
      }
      steps {
        sh 'echo "uploading to S3"'
      }
    }

    stage('HIL: Build') {
      agent { 
        label 'PX4FMU_V4'
      }
      steps {
          // sh 'make posix'
          // sh 'ant -buildfile ./Tools/jMAVSim/build.xml'
          sh 'make px4fmu-v4_default'
      }
    }
    stage('HIL: Flash') {
      agent { 
        label 'PX4FMU_V4'
      }
      steps {
          parallel(
              'Flashing (USB)': {
                  timeout(time: 120, unit: 'SECONDS') {
                      sh './Tools/HIL/reboot_device.sh USB'
                      sh './Tools/px_uploader.py --port $USB_DEVICE --baud-flightstack 115200 --baud-bootloader 115200 ./build/px4fmu-v4_default/px4fmu-v4_default.px4'
                      sh './Tools/HIL/reboot_device.sh USB'
                  }
              },
              'Monitoring (FTDI)': {
                  timeout(time: 120, unit: 'SECONDS') {
                      sh './Tools/HIL/reboot_device.sh FTDI'
                      sh './Tools/HIL/monitor_firmware_upload.py --device $FTDI_DEVICE --baudrate 57600'
                  }
              }
          )   
      }
    }
    stage('HIL: Test') {
      agent { 
        label 'PX4FMU_V4'
      }
      steps {
          timeout(time: 60, unit: 'SECONDS') {
              sh './Tools/HIL/reboot_device.sh FTDI'
              sh './Tools/HIL/reboot_device.sh USB'
              sh './Tools/HIL/run_tests.py --device $FTDI_DEVICE'
          }
      }
    }
  }
}
