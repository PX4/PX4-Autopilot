pipeline {
  agent {
    docker {
      image 'px4io/px4-dev-simulation:2017-09-26'
      args '--env CCACHE_DISABLE=1 --env CI=true'
    }
    
  }
  stages {
    stage('Quality Checks') {
      steps {
        sh 'make check_format'
      }
    }
    stage('Build') {
      parallel {
        stage('posix_sitl_default') {
          steps {
            sh 'make posix_sitl_default'
          }
        }
        stage('nuttx_px4fmu-v2_default') {
          steps {
            sh 'make nuttx_px4fmu-v2_default'
            archiveArtifacts 'build/*/*.px4'
          }
        }
      }
    }
    stage('Test') {
      steps {
        sh 'make tests'
      }
    }
    stage('Generate Metadata') {
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
      parallel {
        stage('Dev Guide Update') {
          when {
            branch 'master'
          }
          steps {
            sh 'git clone https://github.com/PX4/Devguide.git'
          }
        }
        stage('User Guide Update') {
          when {
            branch 'master'
          }
          steps {
            sh 'git clone https://github.com/PX4/px4_user_guide.git'
          }
        }
        stage('QGC Metadata Update') {
          when {
            branch 'master'
          }
          steps {
            sh 'git clone https://github.com/mavlink/qgroundcontrol.git'
          }
        }
        stage('S3 Upload') {
          when {
            branch 'master|beta|stable'
          }
          steps {
            sh 'echo "uploading to S3"'
          }
        }
      }
    }
  }
}
