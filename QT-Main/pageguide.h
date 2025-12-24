#ifndef PAGEGUIDE_H
#define PAGEGUIDE_H

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QHideEvent>

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

    // 수동 조작 버튼 슬롯
    void on_btnForward_clicked();
    void on_btnStop_clicked();
    void on_btnBack_clicked();

    // 화면이 숨겨질 때(다른 화면으로 갈 때) 자동 실행되는 이벤트 함수
protected:
    void hideEvent(QHideEvent *event) override;

private:
    Ui::PageGuide *ui;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    void sendGoal(double x, double y, double w = 1.0);
    void moveTurtle(double linear, double angular);
};

#endif // PAGEGUIDE_H
