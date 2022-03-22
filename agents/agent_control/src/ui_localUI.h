/********************************************************************************
** Form generated from reading UI file 'localUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOCALUI_H
#define UI_LOCALUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_local_guiDlg
{
public:
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *params;
    QWidget *charge;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLCDNumber *lcdNumber;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLCDNumber *speed;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QLCDNumber *wheel_angle;
    QWidget *gps;
    QHBoxLayout *horizontalLayout_6;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_4;
    QLCDNumber *longitud_n;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QLCDNumber *latitud_n;
    QFrame *laser;
    QWidget *webengine;

    void setupUi(QWidget *local_guiDlg)
    {
        if (local_guiDlg->objectName().isEmpty())
            local_guiDlg->setObjectName(QString::fromUtf8("local_guiDlg"));
        local_guiDlg->resize(991, 539);
        verticalLayout_3 = new QVBoxLayout(local_guiDlg);
        verticalLayout_3->setSpacing(9);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout_3->setContentsMargins(9, -1, -1, -1);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setSizeConstraint(QLayout::SetDefaultConstraint);
        params = new QHBoxLayout();
        params->setSpacing(1);
        params->setObjectName(QString::fromUtf8("params"));
        charge = new QWidget(local_guiDlg);
        charge->setObjectName(QString::fromUtf8("charge"));
        verticalLayout = new QVBoxLayout(charge);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(charge);
        label->setObjectName(QString::fromUtf8("label"));
        label->setStyleSheet(QString::fromUtf8("font: 75 13pt \"Current batery\";"));

        horizontalLayout->addWidget(label);

        lcdNumber = new QLCDNumber(charge);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setEnabled(true);
        lcdNumber->setMinimumSize(QSize(150, 10));
        lcdNumber->setMaximumSize(QSize(300, 300));
        lcdNumber->setStyleSheet(QString::fromUtf8("font: 75 13pt \"Ubuntu Condensed\";\n"
"color: rgb(170, 0, 0)"));
        lcdNumber->setSmallDecimalPoint(false);
        lcdNumber->setProperty("intValue", QVariant(100));

        horizontalLayout->addWidget(lcdNumber);


        verticalLayout->addLayout(horizontalLayout);


        params->addWidget(charge);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(local_guiDlg);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setStyleSheet(QString::fromUtf8("font: 75 13pt"));

        horizontalLayout_5->addWidget(label_5);

        speed = new QLCDNumber(local_guiDlg);
        speed->setObjectName(QString::fromUtf8("speed"));
        speed->setStyleSheet(QString::fromUtf8("font: 75 13pt \"Ubuntu Condensed\";\n"
""));

        horizontalLayout_5->addWidget(speed);


        params->addLayout(horizontalLayout_5);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_6 = new QLabel(local_guiDlg);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setStyleSheet(QString::fromUtf8("font: 75 13pt"));

        horizontalLayout_4->addWidget(label_6);

        wheel_angle = new QLCDNumber(local_guiDlg);
        wheel_angle->setObjectName(QString::fromUtf8("wheel_angle"));
        wheel_angle->setStyleSheet(QString::fromUtf8("font: 75 13pt \"Ubuntu Condensed\";\n"
""));

        horizontalLayout_4->addWidget(wheel_angle);


        params->addLayout(horizontalLayout_4);

        gps = new QWidget(local_guiDlg);
        gps->setObjectName(QString::fromUtf8("gps"));
        horizontalLayout_6 = new QHBoxLayout(gps);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_4 = new QLabel(gps);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setStyleSheet(QString::fromUtf8("font: 75 11pt"));

        horizontalLayout_2->addWidget(label_4);

        longitud_n = new QLCDNumber(gps);
        longitud_n->setObjectName(QString::fromUtf8("longitud_n"));
        longitud_n->setDigitCount(8);

        horizontalLayout_2->addWidget(longitud_n);


        horizontalLayout_6->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_3 = new QLabel(gps);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setStyleSheet(QString::fromUtf8("font: 75 11pt"));

        horizontalLayout_3->addWidget(label_3);

        latitud_n = new QLCDNumber(gps);
        latitud_n->setObjectName(QString::fromUtf8("latitud_n"));
        latitud_n->setDigitCount(8);

        horizontalLayout_3->addWidget(latitud_n);


        horizontalLayout_6->addLayout(horizontalLayout_3);


        params->addWidget(gps);

        params->setStretch(1, 2);
        params->setStretch(2, 2);
        params->setStretch(3, 6);

        verticalLayout_2->addLayout(params);

        laser = new QFrame(local_guiDlg);
        laser->setObjectName(QString::fromUtf8("laser"));
        laser->setFrameShape(QFrame::StyledPanel);
        laser->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(laser);

        webengine = new QWidget(local_guiDlg);
        webengine->setObjectName(QString::fromUtf8("webengine"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(webengine->sizePolicy().hasHeightForWidth());
        webengine->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(webengine);

        verticalLayout_2->setStretch(1, 2);
        verticalLayout_2->setStretch(2, 2);

        verticalLayout_3->addLayout(verticalLayout_2);


        retranslateUi(local_guiDlg);

        QMetaObject::connectSlotsByName(local_guiDlg);
    } // setupUi

    void retranslateUi(QWidget *local_guiDlg)
    {
        local_guiDlg->setWindowTitle(QApplication::translate("local_guiDlg", "Melex controller", nullptr));
        label->setText(QApplication::translate("local_guiDlg", "<html><head/><body><p align=\"center\">Battery</p></body></html>", nullptr));
#ifndef QT_NO_TOOLTIP
        lcdNumber->setToolTip(QApplication::translate("local_guiDlg", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        lcdNumber->setWhatsThis(QApplication::translate("local_guiDlg", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
#endif // QT_NO_WHATSTHIS
#ifndef QT_NO_ACCESSIBILITY
        lcdNumber->setAccessibleDescription(QString());
#endif // QT_NO_ACCESSIBILITY
        label_5->setText(QApplication::translate("local_guiDlg", "Speed", nullptr));
        label_6->setText(QApplication::translate("local_guiDlg", "Wheel angle", nullptr));
        label_4->setText(QApplication::translate("local_guiDlg", "Longitud: ", nullptr));
        label_3->setText(QApplication::translate("local_guiDlg", "Latitud:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class local_guiDlg: public Ui_local_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOCALUI_H
