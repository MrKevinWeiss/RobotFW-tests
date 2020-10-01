nodes = nodesByLabel('HIL')
node_boards = []
node_tests = []
robot_fw_commit_id = ""
riot_commit_id = ""

pipeline {
    agent { label 'master' }
    options {
        // If the whole process takes more than x hours then exit
        // This must be longer since multiple jobs can be started but waiting on nodes to complete
        timeout(time: 3, unit: 'HOURS')
        // Failing fast allows the nodes to be interrupted as some steps can take a while
        parallelsAlwaysFailFast()
    }
    parameters {
        choice(name: 'HIL_RIOT_VERSION', choices: ['submodule', 'master', 'pull'], description: 'The RIOT branch or PR to test.')
        string(name: 'HIL_RIOT_PULL', defaultValue: '0', description: 'RIOT pull request number')
    }
    stages {
        stage('setup') {
            steps {
                createPipelineTriggers()
                stepClone()
                stash name: 'sources', useDefaultExcludes: false
                collect_tests()
                collect_boards()
            }
        }
        stage('build tests') {
            steps {
                build_jobs()
            }
        }

        stage('node test') {
            steps {
                runParallel items: node_boards.collect { "${it}" }
            }
        }

        stage('compile results') {
            steps {
                step_compile_results()
            }
        }
        stage('notify') {
            steps {
                emailext (
                    body: '''${SCRIPT, template="groovy-html.template"}''',
                    mimeType: 'text/html',
                    subject: "${currentBuild.fullDisplayName}",
                    from: 'jenkins@riot-ci.inet.haw-hamburg.de',
                    to: '${DEFAULT_RECIPIENTS}',
                    replyTo: '${DEFAULT_RECIPIENTS}'
                )
            }
        }
    }
}

def runParallel(args) {
    parallel args.items.collectEntries { name -> [ "${name}": {

        node (name) {
            stage("${name}") {
                // We want to timeout a node if it doesn't respond
                // The timeout should only start once it is acquired
                timeout(time: 60, unit: 'MINUTES') {
                    script {
                        stepRunNodeTests()
                    }
                }
            }
        }
    }]}
}

void createPipelineTriggers() {
    script {
        def triggers = []
        if (env.BRANCH_NAME == 'nightly') {
            triggers = [parameterizedCron('0 1 * * * % HIL_RIOT_VERSION=master')]
        }
        properties([
            pipelineTriggers(triggers)
        ])
    }
}

def collect_tests() {
    script {
        node_tests = sh(returnStdout: true,
                script:  """
                            for dir in \$(find tests -maxdepth 1 -mindepth 1 -type d); do
                                [ -d \$dir/tests ] && { echo \$dir ; } || true
                            done
                        """).tokenize()
        sh script: "echo collected tests: ${node_tests.join(",")}", label: "print tests"
    }
}

def collect_boards() {
    script {
        for (int i=0; i < nodes.size(); ++i) {
            node (nodes[i]) {
                node_boards.push(env.BOARD)
            }
        }
        node_boards.unique()
        sh script: "echo collected boards: ${node_boards.join(",")}", label: "print boards"
    }
}

def build_job(board, test) {
    exit_code = sh(
        script: "BUILD_IN_DOCKER=1 BOARD=${board} make -C ${test} clean all -j",
        returnStatus: true,
        label: "Build BOARD=${board} TEST=${test}"
    )

    if (exit_code == 0) {
        /* Must remove all / to get stash to work */
        s_name = (board + "_" + test).replace("/", "_")
        stash name: s_name, includes: "${test}/bin/${board}/*.elf,${test}/bin/${board}/*.hex,${test}/bin/${board}/*.bin"
    }
}

def build_jobs() {
    script {
        node ("riot_build") {
            deleteDir()
            sh script: """
                sh ${GIT_CACHE_PATH}git-cache clone https://github.com/RIOT-OS/RobotFW-tests ${robot_fw_commit_id} .
                sh ${GIT_CACHE_PATH}git-cache clone https://github.com/RIOT-OS/RIOT ${riot_commit_id} RIOT
            """, label: "checkout from git cache"
            for (int t_idx=0; t_idx < node_tests.size(); t_idx++) {
                for (int b_idx=0; b_idx < node_boards.size(); b_idx++) {
                    build_job(node_boards[b_idx], node_tests[t_idx])
                }
            }
        }
    }
}

def step_compile_results()
{
    /* Some hacks are needed since the master must run the script on the
     * archive but there is not simple way of finding the location of the
     * archive. The best way is to take the env vars and parse them to
     * fit the path. The branch name has some kind of hash at the end so an ls
     * and grep should return whatever the directory name is.
     * There is an assumption that the grep will only find one result
     */
    sh '''
        HIL_JOB_NAME=$(echo ${JOB_NAME}| cut -d'/' -f 1)
        HIL_BRANCH_NAME=$(echo $JOB_NAME| cut -d'/' -f 2)
        HIL_BRANCH_NAME=$(echo $HIL_BRANCH_NAME | sed 's/%2F/-/g')
        HIL_BRANCH_NAME=$(echo $HIL_BRANCH_NAME | sed 's/_/-/g')
        HIL_BRANCH_NAME=$(ls ${JENKINS_HOME}/jobs/${HIL_JOB_NAME}/branches/ | grep "^$HIL_BRANCH_NAME")
        ./dist/tools/ci/results_to_xml.sh ${JENKINS_HOME}/jobs/${HIL_JOB_NAME}/branches/${HIL_BRANCH_NAME}/builds/${BUILD_NUMBER}/archive/build/robot/
    '''
}

def stepClone()
{
    deleteDir()
    checkout scm
    if ("${env.BRANCH_NAME}" == 'nightly') {
        // update nightly branch to latest master and push
        withCredentials([usernamePassword(credentialsId: 'da54a500-472f-4005-9399-a0ab5ce4da7e', passwordVariable: 'GIT_PASSWORD', usernameVariable: 'GIT_USERNAME')]) {
            sh(script: """
                git config --global credential.username ${GIT_USERNAME}
                git config --global credential.helper "!echo password=${GIT_PASSWORD}; echo"
                git pull --rebase origin master
                git push origin HEAD:nightly
            """, label: "Update robot nightly branch")
        }
    }
    if ("${params.HIL_RIOT_VERSION}" == 'master') {
        sh script: 'git submodule update --init --remote --rebase --depth 1', label: "checkout latest RIOT master"
    }
    else {
        sh script: 'git submodule update --init --depth 1', label: "update RIOT submodule"
        if ("${params.HIL_RIOT_VERSION}" == 'pull' && "${params.HIL_RIOT_PULL}" != '0') {
            // checkout specified PR number
            def prnum = params.HIL_RIOT_PULL.toInteger()
            sh script: """
                cd RIOT
                git fetch origin +refs/pull/${prnum}/merge
                git checkout FETCH_HEAD
            """, label: "checkout RIOT PR ${prnum}"
        }
    }
    robot_fw_commit_id = sh(script: "git rev-parse HEAD", returnStdout: true).trim()
    riot_commit_id = sh(script: """
            cd RIOT
            git rev-parse HEAD
        """, returnStdout: true).trim()
    sh script: "echo robot_fw_commit_id " + robot_fw_commit_id, label: "robot_fw_commit_id=${robot_fw_commit_id}"
    sh script: "echo riot_commit_id " + riot_commit_id, label: "riot_commit_id=${riot_commit_id}"
}

def stepRunNodeTests()
{
    catchError(buildResult: 'UNSTABLE', stageResult: 'FAILURE') {
        stage( "${env.BOARD} setup"){
            stepPrepareNodeWorkingDir()
            //tests = stepDiscoverTests()
        }
        for (int i=0; i < node_tests.size(); i++) {
            stage("${node_tests[i]}") {
                def timeout_stop_exc = null
                catchError(buildResult: 'UNSTABLE', stageResult: 'FAILURE', catchInterruptions: false) {
                    stepPrintEnv()
                    stepReset(node_tests[i])
                    stepUnstashBinaries(node_tests[i])
                    //s_name = (${env.BOARD} + "_" + node_tests[i]).replace("/", "_")
                    //echo s_name
                    //unstash name:
                    //stepMake(tests[i])
                    stepFlash(node_tests[i])
                    stepTest(node_tests[i])
                    stepArchiveTestResults(node_tests[i])
                }
            }
        }
    }
}

def stepUnstashBinaries(test) {
    unstash name: "${env.BOARD}_${test.replace("/", "_")}"
}

def stepPrepareNodeWorkingDir()
{
    deleteDir()
    //unstash name: 'sources'
    sh script: """
                sh ${GIT_CACHE_PATH}git-cache clone https://github.com/RIOT-OS/RobotFW-tests ${robot_fw_commit_id} .
                sh ${GIT_CACHE_PATH}git-cache clone https://github.com/RIOT-OS/RIOT ${riot_commit_id} RIOT
            """, label: "checkout from git cache"

}

def stepDiscoverTests() {
    return sh(returnStdout: true,
    script:  """
                for dir in \$(find tests -maxdepth 1 -mindepth 1 -type d); do
                    [ -d \$dir/tests ] && { echo \$dir ; } || true
                done
            """).tokenize()
}

def stepPrintEnv()
{
    sh 'dist/tools/ci/print_environment.sh'
}

def stepReset(test)
{
    sh "python3 -m bph_pal --philip_reset"
    sh "make -C ${test} reset"
}

def stepMake(test)
{
    sh "make -C ${test}"
}

def stepFlash(test)
{
    sh "make -C ${test} flash-only"
}

def stepTest(test)
{
    def test_name = test.replaceAll('/', '_')
    sh "make -C ${test} robot-clean || true"
    // We don't want to stop running other tests since the robot-test is allowed to fail
    catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE', catchInterruptions: false) {
        sh "make -C ${test} robot-test"
    }
}

def stepArchiveTestResults(test)
{
    def test_name = test.replaceAll('/', '_')
    sh "make -C ${test} robot-html || true"
    archiveArtifacts artifacts: "build/robot/${env.BOARD}/${test_name}/*.xml"
    archiveArtifacts artifacts: "build/robot/${env.BOARD}/${test_name}/*.html"
    junit "build/robot/${env.BOARD}/${test_name}/xunit.xml"
}
