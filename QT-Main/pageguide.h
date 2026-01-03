#ifndef PAGEGUIDE_H
#define PAGEGUIDE_H

#include <QWidget>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/twist.hpp>
#include <QPixmap>
#include <QPointer>

class QDialog;
namespace Ui {
class PageGuide;
}

class PageGuide : public QWidget
{
    Q_OBJECT
protected:
    void resizeEvent(QResizeEvent *e) override;
public:
    explicit PageGuide(QWidget *parent = nullptr);
    ~PageGuide();

signals:
    void backToCartClicked();
    void requestGoal(double x, double y);
    void requestCancelGoal();

private slots:

    void onBackToCartClicked();

    void onPuzzleClicked();
    void onCreamClicked();
    void onSnackClicked();
    void onPayClicked();
    void onbtnphoneClicked();


private:
    Ui::PageGuide *ui;
    void applyTreePixmap();
    QPixmap m_treePixmap;
    // rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

    // ▼ [추가] 속도 명령 퍼블리셔
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    void sendGoal(double x, double y, double w = 1.0);

    // ▼ [추가] 속도 전송 도우미 함수
    void moveTurtle(double linear, double angular);
    void showMovePopup(const QString &zoneText);
     void showMovePopupAsync(const QString &zoneText);
    QPointer<QDialog> m_moveDlg;                      // ✅ 현재 떠있는 팝업 추적
};

#endif // PAGEGUIDE_H
