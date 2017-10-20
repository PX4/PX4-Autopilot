pipeline {
  agent any
  stages {
    stage('Build') {
      parallel {
        stage('px4fmu_firmware') {
          steps {
            sh './Tools/docker_run.sh \'make px4fmu_firmware\''
          }
        }
        stage('posix_sitl_default') {
          steps {
            sh './Tools/docker_run.sh \'make posix_sitl_default\''
          }
        }
        stage('alt_firmware') {
          steps {
            sh './Tools/docker_run.sh \'make alt_firmware\''
          }
        }
        stage('misc_qgc_extra_firmware') {
          steps {
            sh './Tools/docker_run.sh \'make misc_qgc_extra_firmware\''
          }
        }
      }
    }
    stage('Test') {
      parallel {
        stage('tests') {
          steps {
            sh './Tools/docker_run.sh \'make tests\''
          }
        }
        stage('check_format') {
          steps {
            sh './Tools/docker_run.sh \'make check_format\''
          }
        }
        stage('clang-tidy') {
          steps {
            sh './Tools/docker_run.sh \'make clang-tidy-quiet\''
          }
        }
        stage('tests_coverage') {
          steps {
            sh './Tools/docker_run.sh \'make tests_coverage\''
          }
        }
      }
    }
  }
  environment {
    CI = '1'
  }
}