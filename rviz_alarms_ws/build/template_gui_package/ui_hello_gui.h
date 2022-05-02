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
    QWidget *layoutWidget;
    QFormLayout *formLayout;
    QLabel *chatter_lbl;
    QLabel *chatter;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout;
    QPushButton *hi_button;
    QSpinBox *spinBox;
    QSpinBox *hi_num;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_3;
    QSlider *horizontalSlider;
    QWidget *widget;

    void setupUi(QWidget *HelloGui)
    {
        if (HelloGui->objectName().isEmpty())
            HelloGui->setObjectName(QStringLiteral("HelloGui"));
        HelloGui->setWindowModality(Qt::NonModal);
        HelloGui->resize(682, 526);
        layoutWidget = new QWidget(HelloGui);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(0, 0, 71, 17));
        formLayout = new QFormLayout(layoutWidget);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        chatter_lbl = new QLabel(layoutWidget);
        chatter_lbl->setObjectName(QStringLiteral("chatter_lbl"));

        formLayout->setWidget(0, QFormLayout::LabelRole, chatter_lbl);

        chatter = new QLabel(layoutWidget);
        chatter->setObjectName(QStringLiteral("chatter"));

        formLayout->setWidget(0, QFormLayout::FieldRole, chatter);

        layoutWidget1 = new QWidget(HelloGui);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(200, 60, 135, 26));
        horizontalLayout = new QHBoxLayout(layoutWidget1);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        hi_button = new QPushButton(layoutWidget1);
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
        horizontalLayoutWidget->setGeometry(QRect(100, 380, 160, 80));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        horizontalSlider = new QSlider(horizontalLayoutWidget);
        horizontalSlider->setObjectName(QStringLiteral("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(horizontalSlider);

        widget = new QWidget(HelloGui);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(400, 230, 120, 80));
        widget->setAutoFillBackground(true);

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
