#include "pagewelcome.h"
#include "ui_pagewelcome.h"

PageWelcome::PageWelcome(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageWelcome)
{
    ui->setupUi(this);


    connect(ui->pPBStart, &QPushButton::clicked, this, &PageWelcome::startClicked);
    ui->label_3->setStyleSheet(
        "QLabel {"
        "border-image: url(:/new/prefix1/Gemini_Generated_Image_2fvasb2fvasb2fva.png);"
        "border-radius:10px;"
        " padiddng::6px 12px;"
        "}"
        );
}

PageWelcome::~PageWelcome(){
    delete ui;
}
