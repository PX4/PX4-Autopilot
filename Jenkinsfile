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
        stage('Build') {
          steps {
            sh 'make posix_sitl_default'
          }
        }
        stage('nuttx_px4fmu-v2_default') {
          steps {
            sh 'make nuttx_px4fmu-v2_default'
            memoryMap {
              wordSize 32
              showBytesOnGraphs true
              scale "KILO"
              parser("GCC", "gcc-5391", "cmd.ld", "mem.map") {
                parserTitle "gcc graphs"
                graph{
                  graphCaption "RAM"
                  graphData    "ram08+ram09"
                }
                graph {
                  graphCaption "ETC"
                  graphData    "etc."
                }
              }
            }
          }
        }
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
    stage('Test') {
      steps {
        sh 'make tests'
      }
    }
    stage('Deploy') {
      parallel {
        stage('User Guide Update') {
          steps {
            sh 'git clone https://github.com/PX4/px4_user_guide.git'
          }
        }
        stage('Dev Guide Update') {
          steps {
            sh 'git clone https://github.com/PX4/Devguide.git'
          }
        }
        stage('S3 Upload') {
          steps {
            sh 'echo "uploading to S3"'
          }
        }
      }
    }
  }
}
