#include <QtGui/QApplication>
#include "mainwindow.h"
#include <QTextCodec>

#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <signal.h>
#include <linux/vt.h>
#include <linux/fb.h>
#include <linux/kd.h>
#include <semaphore.h>
#include <linux/fb.h>

#include "ctrlapplication.h"

MainWindow *gpMainWnd;

void bb_signals(int sigs, void (*f)(int))
{
    int sig_no = 0;
    int bit = 1;

    while (sigs) {
        if (sigs & bit) {
            sigs &= ~bit;
            signal(sig_no, f);
        }
        sig_no++;
        bit <<= 1;
    }
}

void preMainCalbrate(void)
{
    int pid;

    int status;

    if (access("/etc/pointercal", F_OK) == 0)
    {
        return ;
    }

    if (access("/usr/local/tslib/bin/ts_calibrate", F_OK) != 0)
    {
        _exit(EXIT_FAILURE);
    }

    pid = vfork();
    if (pid < 0) {
        /* TODO: log perror? */
        _exit(EXIT_FAILURE);
    }

    if (!pid) {

        char *argv[3];

        bb_signals(0
            | (1 << SIGCHLD)
            | (1 << SIGPIPE)
            | (1 << SIGHUP)
            , SIG_DFL);

        argv[0] = "/usr/local/tslib/bin/ts_calibrate";
        argv[1] = NULL;

        execv(argv[0], argv);

    }

    wait(&status);

}


int main(int argc, char *argv[])
{
#ifndef QT_WEBKIT
    CtrlApplication a(argc, argv);
#else
    QApplication a(argc, argv);
#endif

    MainWindow w;

#ifndef QT_WEBKIT
    preMainCalbrate();
#endif
    chdir(dirname(argv[0]));

    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
    QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));
    gpMainWnd = &w;

    w.show();
    
    return a.exec();
}
