pipeline {
  agent none
  stages {
    stage('build') {
      parallel {

        stage('build') {
          agent {
            docker {
              image 'px4io/px4-dev-base:2017-12-30'
              args '-e CI=true -e CCACHE_BASEDIR=$WORKSPACE -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
            } 
          }
          steps {
            sh 'git clean -ff -x -d .'
            sh 'git submodule deinit -f .'
            sh 'git submodule update --init --recursive'
            sh './build.sh'
          }
        }

        stage('build clang') {
          agent {
            docker {
              image 'px4io/px4-dev-clang:2017-12-30'
              args '-e CI=true -e CCACHE_BASEDIR=$WORKSPACE -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
            }
            
          }
          steps {
            sh 'git clean -ff -x -d .'
            sh 'git submodule deinit -f .'
            sh 'git submodule update --init --recursive'
            sh 'CC=clang CXX=clang++ ./build.sh'
          }
        }

        //stage('pytest') {
        //  agent {
        //    docker {
        //      image 'px4io/px4-dev-base:2017-12-30'
        //      args '-e CI=true -e CCACHE_BASEDIR=$WORKSPACE -e CCACHE_DIR=/tmp/ccache -v /tmp/ccache:/tmp/ccache:rw'
        //    }
        //  }
        //  steps {
        //    sh 'git clean -ff -x -d .'
        //    sh 'git submodule deinit -f .'
        //    sh 'git submodule update --init --recursive'
        //    sh 'pip install --user -r ./EKF/tests/pytest/requirements.txt'
        //    sh './build.sh'
        //  }
        //}

      }
    }
  }
}
