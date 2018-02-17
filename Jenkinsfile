pipeline {
    agent {
        docker { image 'lochnair/mtk-buildenv:latest' }
    }

    stages {
        stage('Clean') {
            steps {
               sh 'git reset --hard'
               sh 'git clean -fX'
            }
        }

        stage('Prepare for out-of-tree builds') {
            steps {
                sh 'make -j5 ARCH=mips CROSS_COMPILE=mipsel-mtk-linux- prepare modules_prepare'
                sh 'rm -rf tmp && mkdir tmp'
                sh 'tar --exclude-vcs --exclude=tmp -cf tmp/e50-ksrc.tar .'
                sh 'mv tmp/e50-ksrc.tar e50-ksrc.tar'
            }
        }

        stage('Build') {
            steps {
                sh 'make -j5 ARCH=mips CROSS_COMPILE=mipsel-mtk-linux- vmlinux modules'
                sh './build_image.sh'
            }
        }
        
        stage('Archive kernel image') {
            steps {
                archiveArtifacts artifacts: 'uImage', fingerprint: true, onlyIfSuccessful: true
            }
        }
        
        stage('Archive kernel modules') {
            steps {
                sh 'make ARCH=mips CROSS_COMPILE=mipsel-mtk-linux- INSTALL_MOD_PATH=destdir modules_install'
                sh 'tar cvjf e50-modules.tar.bz2 -C destdir .'
                archiveArtifacts artifacts: 'e50-modules.tar.bz2', fingerprint: true, onlyIfSuccessful: true
            }
        }

        stage('Archive build tree') {
            steps {
                sh 'tar -uvf e50-ksrc.tar Module.symvers'
                sh 'bzip2 e50-ksrc.tar'
                archiveArtifacts artifacts: 'e50-ksrc.tar.bz2', fingerprint: true, onlyIfSuccessful: true
            }
        }
    }
}
