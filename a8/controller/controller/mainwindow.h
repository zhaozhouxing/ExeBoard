#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QSettings>
#include "Display.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void clearIdleSecond();

    friend void DispIndicationEntry(unsigned char *pucData,int iLength);

    void emitDispIndication(unsigned char *pucData,int iLength);

private slots:
    void on_timerEvent();
    void on_timerSecondEvent();
    void on_dispIndication(unsigned char *pucData,int iLength);

    void on_pbRun_clicked();

    void on_pbQryFm_clicked();

    void on_pbQryQm_clicked();

signals:
    void dispIndication(unsigned char *pucData,int iLength);
    
private:
    Ui::MainWindow *ui;


    QTimer* m_timeTimer;
    QTimer* m_timerSecond;

};

extern MainWindow *gpMainWnd;

#endif // MAINWINDOW_H
