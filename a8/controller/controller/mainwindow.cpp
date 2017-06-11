#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);

    setWindowFlags(Qt::FramelessWindowHint);


     m_timeTimer = new QTimer(this);
     connect(m_timeTimer, SIGNAL(timeout()), this, SLOT(on_timerEvent()));
     m_timeTimer->start(1000*60); // peroid of one minute

     m_timerSecond = new QTimer(this);
     connect(m_timerSecond, SIGNAL(timeout()), this, SLOT(on_timerSecondEvent()));
     m_timerSecond->start(1000); // peroid of one second

	 connect(this, SIGNAL(dispIndication(unsigned char *,int)),
		 this, SLOT(on_dispIndication(unsigned char *,int)));

	 CcbInit();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::clearIdleSecond()
{
}


void MainWindow::on_timerEvent()
{

}

void MainWindow::on_timerSecondEvent()
{

}

void MainWindow::on_dispIndication(unsigned char *pucData,int iLength)
{
    qDebug("on_dispIndication \r\n");
}

void MainWindow::emitDispIndication(unsigned char *pucData,int iLength)
{
	emit dispIndication(pucData,iLength);
}

void DispIndicationEntry(unsigned char *pucData,int iLength)
{
   // do something here
   gpMainWnd->emitDispIndication(pucData,iLength);
}


void MainWindow::on_pbRun_clicked()
{

}

void MainWindow::on_pbQryFm_clicked()
{
	DispCmdEntry(DISP_CMD_QRY_FM,NULL,0);

}

void MainWindow::on_pbQryQm_clicked()
{
	DispCmdEntry(DISP_CMD_QRY_QM,NULL,0);

}
