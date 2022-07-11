/********************************************************************************
** Form generated from reading UI file 'mission_pathfollowUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MISSION_PATHFOLLOWUI_H
#define UI_MISSION_PATHFOLLOWUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PathFollow_UI
{
public:
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QComboBox *list_pickuppoints;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_2;
    QComboBox *list_destinationpoints;

    void setupUi(QWidget *PathFollow_UI)
    {
        if (PathFollow_UI->objectName().isEmpty())
            PathFollow_UI->setObjectName(QString::fromUtf8("PathFollow_UI"));
        PathFollow_UI->resize(526, 578);
        PathFollow_UI->setTabletTracking(false);
        verticalLayout_2 = new QVBoxLayout(PathFollow_UI);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label = new QLabel(PathFollow_UI);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_3->addWidget(label);

        list_pickuppoints = new QComboBox(PathFollow_UI);
        list_pickuppoints->setObjectName(QString::fromUtf8("list_pickuppoints"));

        horizontalLayout_3->addWidget(list_pickuppoints);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_2 = new QLabel(PathFollow_UI);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_4->addWidget(label_2);

        list_destinationpoints = new QComboBox(PathFollow_UI);
        list_destinationpoints->setObjectName(QString::fromUtf8("list_destinationpoints"));

        horizontalLayout_4->addWidget(list_destinationpoints);


        verticalLayout->addLayout(horizontalLayout_4);


        horizontalLayout_5->addLayout(verticalLayout);


        verticalLayout_2->addLayout(horizontalLayout_5);


        retranslateUi(PathFollow_UI);

        QMetaObject::connectSlotsByName(PathFollow_UI);
    } // setupUi

    void retranslateUi(QWidget *PathFollow_UI)
    {
        PathFollow_UI->setWindowTitle(QApplication::translate("PathFollow_UI", "Follow Path Action", nullptr));
        label->setText(QApplication::translate("PathFollow_UI", "Pick up point", nullptr));
        label_2->setText(QApplication::translate("PathFollow_UI", "Destination point", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PathFollow_UI: public Ui_PathFollow_UI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MISSION_PATHFOLLOWUI_H
