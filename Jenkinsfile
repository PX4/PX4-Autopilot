pipeline {
  agent {
    docker {
      image 'px4io/px4-dev-simulation:2017-09-26'
      args '--env=CCACHE_DISABLE --env=CI'
    }
    
  }
  stages {
    stage('Quick Check') {
      steps {
        sh '''make distclean;
make posix_sitl_default;'''
      }
    }
    stage('Deploy') {
      parallel {
        stage('airframe') {
          steps {
            sh 'make airframe_metadata'
            archiveArtifacts 'airframes.md, airframes.xml'
            sh '''git clone --branch master https://github.com/PX4/Devguide.git
ls Devguide'''
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
  }
  environment {
    CI = '1'
    CCACHE_DISABLE = '1'
  }
}