#ifndef CBITMAPBUTTON_H
#define CBITMAPBUTTON_H

#include <QWidget>
#include <QtGui/QPushButton>

typedef enum
{
    BITMAPBUTTON_ICON_TOPLEFT,
    BITMAPBUTTON_ICON_CENTER,
    BITMAPBUTTON_ICON_VCENTER,
    BITMAPBUTTON_ICON_NUM
}BITMAPBUTTON_ICON_ENUM;

class CBitmapButton : public QPushButton
{
    Q_OBJECT
public:
    explicit CBitmapButton(QWidget *parent = 0);
    void setPressPicture(QPixmap *pic)  ;
    void setButtonPicture(QPixmap *pic) ;
    void setTip(QString &str,QColor color = 0xffffff,QPixmap *pIcon=NULL,int iconpos = BITMAPBUTTON_ICON_VCENTER);
    void setTip(QPixmap *pIcon=NULL,int iconpos = BITMAPBUTTON_ICON_VCENTER);
signals:
    
public slots:

protected:
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
protected:
    virtual void paintEvent(QPaintEvent *pe);

private:
	QPixmap *pressPicture;
	QPixmap *buttonPicture;

    QString strTip;
    QColor colorText;
    QPixmap *pmIcon;
    int     iconposType;

};

#endif // CBITMAPBUTTON_H
