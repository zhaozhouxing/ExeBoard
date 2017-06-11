#include "ctrlapplication.h"
#include "mainwindow.h"

CtrlApplication::CtrlApplication( int argc, char **argv ):
    QApplication( argc, argv )
{
}


bool CtrlApplication::qwsEventFilter(QWSEvent *event)
{


    if (QWSEvent::Mouse  == event->type
        || QWSEvent::Key == event->type)
    {
        if (gpMainWnd != NULL)
        {
            gpMainWnd->clearIdleSecond();
        }
    }

    if(QWSEvent::Key == event->type)
    {
        QWSKeyEvent *ke = static_cast<QWSKeyEvent*>(event);
        QEvent::Type type = ke->simpleData.is_press ?
                            QEvent::KeyPress : QEvent::KeyRelease;


        // qDebug("Key: %d,%d\n",ke->simpleData.keycode,type);
        switch(ke->simpleData.keycode)
        {
        case Qt::Key_Home: // for lcd screen on off, code: 0x01000010
            if (type == QEvent::KeyRelease)
            {
            }
            return TRUE;
        case Qt::Key_Down: // code: 0x01000015
            // do someting, later implement
            return TRUE;
        case Qt::Key_Up: // code : 0x01000013
            // do someting, later implement
            return TRUE;
        case Qt::Key_End: // code : 0x01000011
            // enter into sleep mode
            return TRUE;
        }
    }
    return false;
}
