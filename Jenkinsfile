pipeline {
    agent {
        docker { image 'lochnair/debian-crossenv:mipsel' }
    }

    stages {
        stage('Clean') {
            steps {
               sh 'git reset --hard'
               sh 'git clean -fX'
            }
        }

        stage('Install dependencies') {
            steps {
                sh 'su-exec root apt-get update'
                sh 'su-exec root apt-get -y install bc bison flex'
            }
        }
        
        stage('Prepare for out-of-tree builds') {
            steps {
                sh 'make ARCH=mips ubnt_er_e50_defconfig'
                sh 'make -j5 ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- prepare modules_prepare'
                sh 'rm -rf tmp && mkdir tmp'
                sh 'tar --exclude-vcs --exclude=tmp -cf tmp/e50-ksrc.tar .'
                sh 'mv tmp/e50-ksrc.tar e50-ksrc.tar'
            }
        }

        stage('Build') {
            steps {
                sh 'make -j5 ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- vmlinux modules'
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
                sh 'make ARCH=mips CROSS_COMPILE=mipsel-linux-gnu- INSTALL_MOD_PATH=destdir modules_install'
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
