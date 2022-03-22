/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QAction *actionStart_Stop;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QSplitter *splitter_1;
    QSplitter *splitter_2;
    QMenuBar *menubar;
    QMenu *menuSimulation;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(865, 602);
        actionStart_Stop = new QAction(guiDlg);
        actionStart_Stop->setObjectName(QString::fromUtf8("actionStart_Stop"));
        centralwidget = new QWidget(guiDlg);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        splitter_1 = new QSplitter(centralwidget);
        splitter_1->setObjectName(QString::fromUtf8("splitter_1"));
        splitter_1->setOrientation(Qt::Horizontal);
        splitter_2 = new QSplitter(splitter_1);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(splitter_2->sizePolicy().hasHeightForWidth());
        splitter_2->setSizePolicy(sizePolicy);
        splitter_2->setOrientation(Qt::Vertical);
        splitter_1->addWidget(splitter_2);

        gridLayout->addWidget(splitter_1, 0, 0, 1, 1);

        guiDlg->setCentralWidget(centralwidget);
        menubar = new QMenuBar(guiDlg);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 865, 22));
        menuSimulation = new QMenu(menubar);
        menuSimulation->setObjectName(QString::fromUtf8("menuSimulation"));
        guiDlg->setMenuBar(menubar);
        statusbar = new QStatusBar(guiDlg);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        guiDlg->setStatusBar(statusbar);

        menubar->addAction(menuSimulation->menuAction());
        menuSimulation->addAction(actionStart_Stop);

        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QMainWindow *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "agent_control", nullptr));
        actionStart_Stop->setText(QApplication::translate("guiDlg", "Start/Stop", nullptr));
        menuSimulation->setTitle(QApplication::translate("guiDlg", "Simulation", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
