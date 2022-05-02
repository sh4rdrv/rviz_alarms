/********************************************************************************
** Form generated from reading UI file 'hello_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HELLO_GUI_H
#define UI_HELLO_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HelloGui
{
public:
    QWidget *widget;
    QFormLayout *formLayout;
    QLabel *chatter_lbl;
    QLabel *chatter;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout;
    QPushButton *hi_button;
    QSpinBox *spinBox;
    QSpinBox *hi_num;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_3;
    QSlider *horizontalSlider;

    void setupUi(QWidget *HelloGui)
    {
        if (HelloGui->objectName().isEmpty())
            HelloGui->setObjectName(QStringLiteral("HelloGui"));
        HelloGui->setWindowModality(Qt::NonModal);
        HelloGui->resize(682, 526);
        widget = new QWidget(HelloGui);
        widget->setObjectName(QStringLiteral("widget"));
        formLayout = new QFormLayout(widget);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        chatter_lbl = new QLabel(widget);
        chatter_lbl->setObjectName(QStringLiteral("chatter_lbl"));

        formLayout->setWidget(0, QFormLayout::LabelRole, chatter_lbl);

        chatter = new QLabel(widget);
        chatter->setObjectName(QStringLiteral("chatter"));

        formLayout->setWidget(0, QFormLayout::FieldRole, chatter);

        widget1 = new QWidget(HelloGui);
        widget1->setObjectName(QStringLiteral("widget1"));
        widget1->setGeometry(QRect(200, 60, 135, 26));
        horizontalLayout = new QHBoxLayout(widget1);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        hi_button = new QPushButton(widget1);
        hi_button->setObjectName(QStringLiteral("hi_button"));

        horizontalLayout->addWidget(hi_button);

        spinBox = new QSpinBox(HelloGui);
        spinBox->setObjectName(QStringLiteral("spinBox"));
        spinBox->setGeometry(QRect(300, 410, 47, 24));
        hi_num = new QSpinBox(HelloGui);
        hi_num->setObjectName(QStringLiteral("hi_num"));
        hi_num->setGeometry(QRect(430, 60, 47, 24));
        horizontalLayoutWidget = new QWidget(HelloGui);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(100, 310, 160, 80));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalSlider = new QSlider(horizontalLayoutWidget);
        horizontalSlider->setObjectName(QStringLiteral("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(horizontalSlider);


        retranslateUi(HelloGui);
        QObject::connect(horizontalSlider, SIGNAL(valueChanged(int)), spinBox, SLOT(setValue(int)));
        QObject::connect(spinBox, SIGNAL(valueChanged(int)), horizontalSlider, SLOT(setValue(int)));

        QMetaObject::connectSlotsByName(HelloGui);
    } // setupUi

    void retranslateUi(QWidget *HelloGui)
    {
        HelloGui->setWindowTitle(QApplication::translate("HelloGui", "Form", Q_NULLPTR));
        chatter_lbl->setText(QApplication::translate("HelloGui", "Chatter:", Q_NULLPTR));
        chatter->setText(QApplication::translate("HelloGui", "---", Q_NULLPTR));
        hi_button->setText(QApplication::translate("HelloGui", "Say Hi", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class HelloGui: public Ui_HelloGui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HELLO_GUI_H
