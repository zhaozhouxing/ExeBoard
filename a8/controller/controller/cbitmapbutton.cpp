#include "cbitmapbutton.h"
#include <QtGui/qpainter.h>

CBitmapButton::CBitmapButton(QWidget *parent) :
    QPushButton(parent)
{
pressPicture = NULL;
buttonPicture = NULL;
pmIcon = NULL;
}

void CBitmapButton::mousePressEvent(QMouseEvent *e)
{
  this->setIcon (QIcon(*pressPicture));
}

/*!
    \reimp
*/
void CBitmapButton::mouseReleaseEvent(QMouseEvent *e)
{
  this->setIcon(QIcon(*buttonPicture));
  //this->setIconSize(QSize(buttonPicture->width(), buttonPicture->height()));
  emit clicked();
}

void CBitmapButton::setButtonPicture(QPixmap *pic)  
{  
    buttonPicture = pic;  
      
    this->setIcon(QIcon(*buttonPicture));
    this->setIconSize(QSize(buttonPicture->width(), buttonPicture->height()));
    //this->setFlat(true);
}  
  
void CBitmapButton::setPressPicture(QPixmap *pic)  
{  
    pressPicture = pic;  
}  

void CBitmapButton::paintEvent(QPaintEvent *e)
{
    QPushButton::paintEvent(e);

    QPainter painter(this);

    if (!strTip.isEmpty())
    {
        painter.setPen(colorText);
        painter.drawText(rect(),Qt::AlignCenter,strTip);
    }
    if (pmIcon != NULL)
    {
        if (BITMAPBUTTON_ICON_TOPLEFT == iconposType)
        {
            painter.drawPixmap(rect().x(),rect().y(),pmIcon->width(),pmIcon->height(),*pmIcon);
        }
        else if (BITMAPBUTTON_ICON_CENTER == iconposType)
        {
            painter.drawPixmap(rect().x()+rect().width()/2-pmIcon->width()/2,rect().y()+rect().height()/2-pmIcon->height()/2,pmIcon->width(),pmIcon->height(),*pmIcon);
        }
        else
        {
            painter.drawPixmap(rect().x(),rect().y()+rect().height()/2-pmIcon->height()/2,pmIcon->width(),pmIcon->height(),*pmIcon);
        }
    }
}

void CBitmapButton::setTip(QString &str,QColor color,QPixmap *pIcon,int iconpos)
{
    strTip = str;
    colorText = color;
    pmIcon = pIcon;
    iconposType = iconpos;
}

void CBitmapButton::setTip(QPixmap *pIcon,int iconpos)
{
    pmIcon = pIcon;
    iconposType = iconpos;
}
