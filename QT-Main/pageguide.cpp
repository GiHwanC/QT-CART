#include "pageguide.h"
#include "ui_pageguide.h"
#include <QPixmap>
#include <QDebug>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);

    // 1. 노드 생성
    node_ = rclcpp::Node::make_shared("page_guide_node");

    // 2. 자율 주행 목표 퍼블리셔 (/goal_pose)
    goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // 3. 수동 조작 퍼블리셔 (/cmd_vel)
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    QPixmap map(":/house.pgm");
    if (map.isNull()) {
        qDebug() << "map load failed";
    } else {
        ui->labelMap->setPixmap(map);
        ui->labelMap->setScaledContents(true);
    }

    // 1. 버튼 3개 생성 (힙 메모리에 할당)
    QPushButton *codeBtnFwd = new QPushButton("▲ Forward", this);
    QPushButton *codeBtnStop = new QPushButton("■ Stop", this);
    QPushButton *codeBtnBack = new QPushButton("▼ Back", this);

    // 2. 버튼 스타일 꾸미기 (색상 및 크기)
    codeBtnFwd->setStyleSheet("background-color: #90EE90; height: 50px; font-weight: bold; font-size: 14px;"); // 연두색
    codeBtnStop->setStyleSheet("background-color: #FF7F7F; height: 50px; font-weight: bold; font-size: 14px;"); // 빨간색
    codeBtnBack->setStyleSheet("background-color: #ADD8E6; height: 50px; font-weight: bold; font-size: 14px;"); // 하늘색

    // 3. 가로 레이아웃을 만들어서 버튼 3개를 나란히 넣음
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(codeBtnFwd);
    buttonLayout->addWidget(codeBtnStop);
    buttonLayout->addWidget(codeBtnBack);

    // 4. 기존 화면 레이아웃에 버튼 레이아웃 추가
    // (기존 레이아웃이 있으면 거기에 추가하고, 없으면 전체 레이아웃을 새로 만듭니다)
    if (this->layout()) {
        // 기존 레이아웃이 있다면 (예: 수직 레이아웃) 맨 아래에 추가
        // 주의: 기존 레이아웃 타입에 따라 addLayout이 안 될 수도 있으니 cast 필요할 수 있음.
        // 여기서는 안전하게 dynamic_cast 시도 혹은 addItem 사용
        QVBoxLayout* vLayout = dynamic_cast<QVBoxLayout*>(this->layout());
        if (vLayout) {
            vLayout->addLayout(buttonLayout);
        } else {
            // 수직 레이아웃이 아니면 그냥 아이템으로 추가 시도
            this->layout()->addItem(buttonLayout);
        }
    } else {
        // 레이아웃이 아예 없는 위젯이라면, 새로 세로 레이아웃을 만들어서 적용
        QVBoxLayout *mainLayout = new QVBoxLayout(this);
        // 기존 UI(ui->setupUi로 만들어진 것들)가 겹치지 않게 주의해야 함.
        // 보통은 setupUi 안에 레이아웃이 있으므로 이 블록으로 잘 안 들어옴.
        mainLayout->addLayout(buttonLayout);
    }

    // 생성한 버튼을 클릭했을 때 실행할 함수 연결 (Signal & Slot)
    connect(codeBtnFwd, &QPushButton::clicked, this, &PageGuide::on_btnForward_clicked);
    connect(codeBtnStop, &QPushButton::clicked, this, &PageGuide::on_btnStop_clicked);
    connect(codeBtnBack, &QPushButton::clicked, this, &PageGuide::on_btnBack_clicked);
    connect(ui->btnBackToCart, SIGNAL(clicked()), this, SLOT(on_btnBackToCart_clicked()));
}

PageGuide::~PageGuide()
{
    delete ui;
}

// 자율 주행 목표 전송
void PageGuide::sendGoal(double x, double y, double w)
{
    if (!goal_publisher_) {
        qDebug() << "Error: Goal Publisher not initialized!";
        return;
    }

    auto msg = geometry_msgs::msg::PoseStamped();

    msg.header.frame_id = "map";
    msg.header.stamp = node_->now();

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = w;

    goal_publisher_->publish(msg);
    qDebug() << "Sending Goal -> X:" << x << " Y:" << y;
}

// 수동 조작 속도 전송
void PageGuide::moveTurtle(double linear, double angular)
{
    if (!cmd_vel_publisher_) {
        qDebug() << "Error: Cmd_vel Publisher not initialized!";
        return;
    }

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;

    cmd_vel_publisher_->publish(msg);
    qDebug() << "Manual Move -> Linear:" << linear << " Angular:" << angular;
}

// 페이지를 벗어나면 자율주행 종료
void PageGuide::hideEvent(QHideEvent *event)
{
    qDebug() << "[PageGuide] Leaving Guide Mode -> Robot Stopped.";
    on_btnStop_clicked();

    QWidget::hideEvent(event);
}

void PageGuide::on_btnBackToCart_clicked()
{
    emit backToCartClicked();
}

void PageGuide::on_foodIcon_clicked()
{
    sendGoal(-0.8, 0.0);
}

void PageGuide::on_groceryIcon_clicked()
{
    sendGoal(2.0, 0.0);
}

void PageGuide::on_btnForward_clicked()
{
    moveTurtle(0.2, 0.0);
}

void PageGuide::on_btnStop_clicked()
{
    moveTurtle(0.0, 0.0);
}

void PageGuide::on_btnBack_clicked()
{
    moveTurtle(-0.2, 0.0);
}
