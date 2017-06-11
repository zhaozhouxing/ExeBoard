#ifndef CTRLAPPLICATION_H
#define CTRLAPPLICATION_H

#include <QApplication>
#include <QWSEvent>

class CtrlApplication : public QApplication
{
    Q_OBJECT

public:
    explicit CtrlApplication( int argc, char **argv );
    virtual bool qwsEventFilter(QWSEvent *event);
signals:

public slots:

};

#endif // CTRLAPPLICATION_H
