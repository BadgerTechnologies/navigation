pipeline {
  agent {
    docker {
      label "ros && docker && small"
      image "242567060652.dkr.ecr.us-west-2.amazonaws.com/ros-ci/cpplint:v1.1"
      registryUrl "https://242567060652.dkr.ecr.us-west-2.amazonaws.com"
      registryCredentialsId "ecr:us-west-2:CIUser"
    }
  }
  options {
    buildDiscarder(logRotator(daysToKeepStr: '14', numToKeepStr: '10'))
  }
  stages {
    stage('test') {
      steps {
        // Currently only the costmap_3d package passes cpplint.
        // The rest of the packages are upstream and have many lint errors.
        dir('costmap_3d') {
          sh label: 'Verify costmap_3d c++ code with cpplint', script: "badger-cpplint"
        }
      }
      post {
        always {
          junit testResults: 'costmap_3d/results/cpplint_junit.xml'
        }
      }
    }
  }
  post {
    always {
      archiveArtifacts artifacts: 'costmap_3d/results/*'
    }
  }
}
