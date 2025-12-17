#ifndef PAGEGUIDE_H
#define PAGEGUIDE_H

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// ▼ [추가] 속도 명령 메시지 헤더
#include <geometry_msgs/msg/twist.hpp>

namespace Ui {
class PageGuide;
}

class PageGuide : public QWidget
{
    Q_OBJECT

public:
    explicit PageGuide(QWidget *parent = nullptr);
    ~PageGuide();

signals:
    void backToCartClicked();

private slots:
    void on_btnBackToCart_clicked();
    void on_foodIcon_clicked();
    void on_groceryIcon_clicked();

    // ▼ [추가] 수동 조작 버튼 슬롯 (UI에서 우클릭 -> Go to slot으로도 생성 가능)
    void on_btnForward_clicked();
    void on_btnStop_clicked();
    void on_btnBack_clicked();

private:
    Ui::PageGuide *ui;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

    // ▼ [추가] 속도 명령 퍼블리셔
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    void sendGoal(double x, double y, double w = 1.0);

    // ▼ [추가] 속도 전송 도우미 함수
    void moveTurtle(double linear, double angular);
};

#endif // PAGEGUIDE_H
