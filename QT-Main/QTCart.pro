QT       += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# ==========================================
# [ROS2 설정 - 수동 연결 (Humble)]
# ==========================================

ROS_DIR = /opt/ros/humble

# 1. 헤더 파일 경로 (INCLUDEPATH)
INCLUDEPATH += $$ROS_DIR/include \
               $$ROS_DIR/include/rclcpp \
               $$ROS_DIR/include/rcl \
               $$ROS_DIR/include/rcutils \
               $$ROS_DIR/include/rmw \
               $$ROS_DIR/include/rcpputils \
               $$ROS_DIR/include/builtin_interfaces \
               $$ROS_DIR/include/geometry_msgs \
               $$ROS_DIR/include/std_msgs \
               $$ROS_DIR/include/nav_msgs \
               $$ROS_DIR/include/rosidl_runtime_c \
               $$ROS_DIR/include/rosidl_runtime_cpp \
               $$ROS_DIR/include/rosidl_typesupport_interface \
               $$ROS_DIR/include/rosidl_typesupport_cpp \
               $$ROS_DIR/include/rosidl_typesupport_introspection_cpp \
               $$ROS_DIR/include/tracetools \
               $$ROS_DIR/include/libstatistics_collector \
               $$ROS_DIR/include/statistics_msgs \
               $$ROS_DIR/include/rcl_yaml_param_parser \
               $$ROS_DIR/include/libyaml_vendor \
               $$ROS_DIR/include/rcl_interfaces \
               $$ROS_DIR/include/rcl_logging_interface \
               $$ROS_DIR/include/rosgraph_msgs \
               $$ROS_DIR/include/unique_identifier_msgs \
               $$ROS_DIR/include/action_msgs \
               $$ROS_DIR/include/rosidl_adapter

# 2. 라이브러리 경로 및 링크 (LIBS)
# 중요: geometry_msgs 관련 라이브러리 대거 추가됨
LIBS += -L$$ROS_DIR/lib \
        -lrclcpp \
        -lrcl \
        -lrcutils \
        -lrcpputils \
        -lrosidl_runtime_c \
        -lrcl_yaml_param_parser \
        -lyaml \
        -ltracetools \
        -lrmw \
        -lgeometry_msgs__rosidl_typesupport_cpp \
        -lgeometry_msgs__rosidl_typesupport_c \
        -lstd_msgs__rosidl_typesupport_cpp \
        -lstd_msgs__rosidl_typesupport_c

# ==========================================

SOURCES += \
    barcodescanner.cpp \
    main.cpp \
    mainwidget.cpp \
    pagecart.cpp \
    pageguide.cpp \
    pagepay.cpp \
    pagepay_card.cpp \
    pagewelcome.cpp \
    uwbdriver.cpp

HEADERS += \
    barcodescanner.h \
    item.h \
    mainwidget.h \
    pagecart.h \
    pageguide.h \
    pagepay.h \
    pagepay_card.h \
    pagewelcome.h \
    uwbdriver.h

FORMS += \
    mainwidget.ui \
    pagecart.ui \
    pageguide.ui \
    pagepay.ui \
    pagepay_card.ui \
    pagewelcome.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES +=

RESOURCES += \
    cart_img.qrc \
    cart_obj.qrc \
    image.qrc \
    maps.qrc
