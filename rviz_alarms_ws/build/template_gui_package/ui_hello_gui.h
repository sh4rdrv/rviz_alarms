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
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HelloGui
{
public:
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout;
    QLabel *chatter_lbl;
    QLabel *chatter;
    QHBoxLayout *horizontalLayout;
    QPushButton *hi_button;
    QSpinBox *hi_num;

    void setupUi(QWidget *HelloGui)
    {
        if (HelloGui->objectName().isEmpty())
            HelloGui->setObjectName(QStringLiteral("HelloGui"));
        HelloGui->resize(375, 72);
        verticalLayout = new QVBoxLayout(HelloGui);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QStringLiteral("formLayout"));
        chatter_lbl = new QLabel(HelloGui);
        chatter_lbl->setObjectName(QStringLiteral("chatter_lbl"));

        formLayout->setWidget(0, QFormLayout::LabelRole, chatter_lbl);

        chatter = new QLabel(HelloGui);
        chatter->setObjectName(QStringLiteral("chatter"));

        formLayout->setWidget(0, QFormLayout::FieldRole, chatter);


        verticalLayout->addLayout(formLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        hi_button = new QPushButton(HelloGui);
        hi_button->setObjectName(QStringLiteral("hi_button"));

        horizontalLayout->addWidget(hi_button);

        hi_num = new QSpinBox(HelloGui);
        hi_num->setObjectName(QStringLiteral("hi_num"));

        horizontalLayout->addWidget(hi_num);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(HelloGui);

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
