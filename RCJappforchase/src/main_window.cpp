#include "main_window.h"

#include "path_planner.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtWidgets/QApplication>
#include <QtWidgets/QBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>

#include <cmath>

namespace rcj {
namespace {

constexpr double kPi = 3.14159265358979323846;

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("RCJ 小车上位机"));
    resize(1280, 860);

    map_ = new MapWidget(this);
    map_->loadMap(QStringLiteral(RCJ_APP_SOURCE_DIR) + QStringLiteral("/Pic/map.png"));

    auto *splitter = new QSplitter(this);
    splitter->addWidget(map_);
    splitter->addWidget(createSidePanel());
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);
    setCentralWidget(splitter);

    connect(&serial_, &SerialClient::logLine, this, &MainWindow::log);
    connect(&serial_, &SerialClient::payloadReceived, this, &MainWindow::handlePayload);
    connect(&serial_, &SerialClient::connectionChanged, this, [this](bool connected) {
        connectButton_->setText(connected && !serial_.isSimulated() ? QStringLiteral("断开") : QStringLiteral("连接"));
    });
    connect(map_, &MapWidget::statusText, this, [this](const QString &text) {
        statusLabel_->setText(text);
        log(text);
    });
    connect(map_, &MapWidget::selectionChanged, this, &MainWindow::updateSelectionControls);

    requestTimer_.setInterval(250);
    connect(&requestTimer_, &QTimer::timeout, this, &MainWindow::sendRequest);
    requestTimer_.start();

    refreshPorts();
}

QWidget *MainWindow::createSidePanel()
{
    auto *tabs = new QTabWidget(this);
    tabs->setMinimumWidth(390);
    tabs->addTab(createConnectionTab(), QStringLiteral("串口"));
    tabs->addTab(createMapTab(), QStringLiteral("地图"));
    tabs->addTab(createObstacleTab(), QStringLiteral("障碍物"));
    tabs->addTab(createMotionTab(), QStringLiteral("运动"));
    tabs->addTab(createPeripheralTab(), QStringLiteral("外设"));
    return tabs;
}

QWidget *MainWindow::createConnectionTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *row = new QHBoxLayout;
    portCombo_ = new QComboBox(page);
    auto *refreshButton = new QPushButton(QStringLiteral("刷新"), page);
    connectButton_ = new QPushButton(QStringLiteral("连接"), page);
    row->addWidget(portCombo_, 1);
    row->addWidget(refreshButton);
    row->addWidget(connectButton_);
    layout->addLayout(row);

    simulateCheck_ = new QCheckBox(QStringLiteral("模拟 STM32 回复"), page);
    layout->addWidget(simulateCheck_);

    statusLabel_ = new QLabel(QStringLiteral("就绪"), page);
    statusLabel_->setWordWrap(true);
    layout->addWidget(statusLabel_);

    logView_ = new QPlainTextEdit(page);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(1200);
    layout->addWidget(logView_, 1);

    connect(refreshButton, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(connectButton_, &QPushButton::clicked, this, &MainWindow::connectOrDisconnect);
    connect(simulateCheck_, &QCheckBox::toggled, &serial_, &SerialClient::setSimulated);
    return page;
}

QWidget *MainWindow::createMapTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *calibBox = new QGroupBox(QStringLiteral("地图标定"), page);
    auto *calibLayout = new QFormLayout(calibBox);
    calibDistanceSpin_ = new QDoubleSpinBox(calibBox);
    calibDistanceSpin_->setRange(1.0, 1000.0);
    calibDistanceSpin_->setValue(182.0);
    calibDistanceSpin_->setSuffix(QStringLiteral(" cm"));
    auto *calibButtons = new QHBoxLayout;
    auto *pointA = new QPushButton(QStringLiteral("选择 A 点"), calibBox);
    auto *pointB = new QPushButton(QStringLiteral("选择 B 点"), calibBox);
    auto *apply = new QPushButton(QStringLiteral("应用标定"), calibBox);
    calibButtons->addWidget(pointA);
    calibButtons->addWidget(pointB);
    calibButtons->addWidget(apply);
    calibLayout->addRow(QStringLiteral("实际距离"), calibDistanceSpin_);
    calibLayout->addRow(calibButtons);
    layout->addWidget(calibBox);

    auto *boundaryBox = new QGroupBox(QStringLiteral("禁行边界"), page);
    auto *boundaryLayout = new QVBoxLayout(boundaryBox);
    auto *boundaryButton = new QPushButton(QStringLiteral("编辑边界多边形"), boundaryBox);
    boundaryLayout->addWidget(boundaryButton);
    layout->addWidget(boundaryBox);

    auto *poseBox = new QGroupBox(QStringLiteral("小车位姿"), page);
    auto *poseLayout = new QFormLayout(poseBox);
    auto *pickRobot = new QPushButton(QStringLiteral("选择小车中心"), poseBox);
    auto *pickHeading = new QPushButton(QStringLiteral("选择车头方向"), poseBox);
    robotYawSpin_ = new QDoubleSpinBox(poseBox);
    robotYawSpin_->setRange(0.0, 359.99);
    robotYawSpin_->setDecimals(2);
    robotYawSpin_->setSuffix(QStringLiteral(" deg"));
    poseLayout->addRow(pickRobot);
    poseLayout->addRow(pickHeading);
    poseLayout->addRow(QStringLiteral("车头朝向"), robotYawSpin_);
    layout->addWidget(poseBox);

    auto *saveButton = new QPushButton(QStringLiteral("保存配置"), page);
    auto *loadButton = new QPushButton(QStringLiteral("加载配置"), page);
    layout->addWidget(saveButton);
    layout->addWidget(loadButton);
    layout->addStretch(1);

    connect(pointA, &QPushButton::clicked, this, [this]() { map_->setMode(MapWidget::Mode::CalibA); });
    connect(pointB, &QPushButton::clicked, this, [this]() { map_->setMode(MapWidget::Mode::CalibB); });
    connect(apply, &QPushButton::clicked, this, [this]() { map_->applyCalibration(calibDistanceSpin_->value()); });
    connect(boundaryButton, &QPushButton::clicked, this, [this]() {
        map_->setMode(MapWidget::Mode::Boundary);
        log(QStringLiteral("左键依次点击边界点，右键结束边界编辑。"));
    });
    connect(pickRobot, &QPushButton::clicked, this, [this]() { map_->setMode(MapWidget::Mode::RobotPose); });
    connect(pickHeading, &QPushButton::clicked, this, [this]() {
        map_->setMode(MapWidget::Mode::RobotHeading);
        log(QStringLiteral("请在地图上点击小车车头指向的位置。"));
    });
    connect(robotYawSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::setInitialRobotYaw);
    connect(map_, &MapWidget::robotYawEdited, this, [this](double yawDeg) {
        odomToMapYawDeg_ = yawDeg;
        robotYawSpin_->blockSignals(true);
        robotYawSpin_->setValue(yawDeg);
        robotYawSpin_->blockSignals(false);
    });
    connect(saveButton, &QPushButton::clicked, this, &MainWindow::saveConfig);
    connect(loadButton, &QPushButton::clicked, this, &MainWindow::loadConfig);
    return page;
}

QWidget *MainWindow::createObstacleTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *addRect = new QPushButton(QStringLiteral("添加矩形障碍"), page);
    auto *addCircle = new QPushButton(QStringLiteral("添加圆形障碍"), page);
    auto *addPolygon = new QPushButton(QStringLiteral("绘制多边形障碍"), page);
    auto *selectMode = new QPushButton(QStringLiteral("选择 / 移动"), page);
    auto *deleteButton = new QPushButton(QStringLiteral("删除选中项"), page);
    layout->addWidget(addRect);
    layout->addWidget(addCircle);
    layout->addWidget(addPolygon);
    layout->addWidget(selectMode);
    layout->addWidget(deleteButton);

    auto *sizeBox = new QGroupBox(QStringLiteral("选中障碍尺寸"), page);
    auto *form = new QFormLayout(sizeBox);
    obstacleWidthSpin_ = new QDoubleSpinBox(sizeBox);
    obstacleHeightSpin_ = new QDoubleSpinBox(sizeBox);
    obstacleRadiusSpin_ = new QDoubleSpinBox(sizeBox);
    for (QDoubleSpinBox *spin : {obstacleWidthSpin_, obstacleHeightSpin_, obstacleRadiusSpin_}) {
        spin->setRange(1.0, 500.0);
        spin->setSuffix(QStringLiteral(" cm"));
    }
    form->addRow(QStringLiteral("宽度"), obstacleWidthSpin_);
    form->addRow(QStringLiteral("高度"), obstacleHeightSpin_);
    form->addRow(QStringLiteral("半径"), obstacleRadiusSpin_);
    layout->addWidget(sizeBox);
    layout->addStretch(1);

    connect(addRect, &QPushButton::clicked, map_, &MapWidget::addRectangleObstacle);
    connect(addCircle, &QPushButton::clicked, map_, &MapWidget::addCircleObstacle);
    connect(addPolygon, &QPushButton::clicked, this, [this]() {
        map_->setMode(MapWidget::Mode::AddPolygon);
        log(QStringLiteral("左键依次点击多边形顶点，右键结束并生成障碍物。"));
    });
    connect(selectMode, &QPushButton::clicked, this, [this]() { map_->setMode(MapWidget::Mode::Select); });
    connect(deleteButton, &QPushButton::clicked, map_, &MapWidget::deleteSelected);
    connect(obstacleWidthSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            map_, &MapWidget::setSelectedObstacleWidthCm);
    connect(obstacleHeightSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            map_, &MapWidget::setSelectedObstacleHeightCm);
    connect(obstacleRadiusSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            map_, &MapWidget::setSelectedObstacleRadiusCm);
    return page;
}

QWidget *MainWindow::createMotionTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *addWaypoint = new QPushButton(QStringLiteral("添加路径点"), page);
    auto *selectMode = new QPushButton(QStringLiteral("选择 / 移动"), page);
    auto *deleteWaypoint = new QPushButton(QStringLiteral("删除选中目标点"), page);
    auto *planButton = new QPushButton(QStringLiteral("规划路径"), page);
    auto *executeButton = new QPushButton(QStringLiteral("执行路径"), page);
    auto *stopButton = new QPushButton(QStringLiteral("急停"), page);
    stopButton->setStyleSheet(QStringLiteral("font-weight: 600; color: #b00020;"));

    waypointHeadingCheck_ = new QCheckBox(QStringLiteral("选中路径点设置朝向"), page);
    waypointHeadingSpin_ = new QDoubleSpinBox(page);
    waypointHeadingSpin_->setRange(0.0, 359.99);
    waypointHeadingSpin_->setDecimals(2);
    waypointHeadingSpin_->setSuffix(QStringLiteral(" deg"));
    waypointXSpin_ = new QDoubleSpinBox(page);
    waypointYSpin_ = new QDoubleSpinBox(page);
    for (QDoubleSpinBox *spin : {waypointXSpin_, waypointYSpin_}) {
        spin->setRange(-1000.0, 1000.0);
        spin->setDecimals(2);
        spin->setSuffix(QStringLiteral(" cm"));
    }
    waypointStepSpin_ = new QDoubleSpinBox(page);
    waypointStepSpin_->setRange(0.1, 100.0);
    waypointStepSpin_->setDecimals(1);
    waypointStepSpin_->setValue(1.0);
    waypointStepSpin_->setSuffix(QStringLiteral(" cm"));

    auto *coordBox = new QGroupBox(QStringLiteral("选中目标点坐标"), page);
    auto *coordLayout = new QFormLayout(coordBox);
    auto *applyCoord = new QPushButton(QStringLiteral("应用坐标"), coordBox);
    coordLayout->addRow(QStringLiteral("X"), waypointXSpin_);
    coordLayout->addRow(QStringLiteral("Y"), waypointYSpin_);
    coordLayout->addRow(applyCoord);

    auto *nudgeBox = new QGroupBox(QStringLiteral("方向键微调"), page);
    auto *nudgeLayout = new QGridLayout(nudgeBox);
    auto *upButton = new QPushButton(QStringLiteral("↑"), nudgeBox);
    auto *downButton = new QPushButton(QStringLiteral("↓"), nudgeBox);
    auto *leftButton = new QPushButton(QStringLiteral("←"), nudgeBox);
    auto *rightButton = new QPushButton(QStringLiteral("→"), nudgeBox);
    nudgeLayout->addWidget(new QLabel(QStringLiteral("步长"), nudgeBox), 0, 0);
    nudgeLayout->addWidget(waypointStepSpin_, 0, 1, 1, 2);
    nudgeLayout->addWidget(upButton, 1, 1);
    nudgeLayout->addWidget(leftButton, 2, 0);
    nudgeLayout->addWidget(rightButton, 2, 2);
    nudgeLayout->addWidget(downButton, 3, 1);

    speedProfileSpin_ = new QSpinBox(page);
    speedProfileSpin_->setRange(0, 2);
    speedProfileSpin_->setValue(1);

    layout->addWidget(addWaypoint);
    layout->addWidget(selectMode);
    layout->addWidget(deleteWaypoint);
    layout->addWidget(waypointHeadingCheck_);
    layout->addWidget(waypointHeadingSpin_);
    layout->addWidget(coordBox);
    layout->addWidget(nudgeBox);
    layout->addWidget(new QLabel(QStringLiteral("速度曲线：0 急 / 1 正常 / 2 柔和"), page));
    layout->addWidget(speedProfileSpin_);
    layout->addWidget(planButton);
    layout->addWidget(executeButton);
    layout->addWidget(stopButton);
    layout->addStretch(1);

    connect(addWaypoint, &QPushButton::clicked, this, [this]() { map_->setMode(MapWidget::Mode::AddWaypoint); });
    connect(selectMode, &QPushButton::clicked, this, [this]() { map_->setMode(MapWidget::Mode::Select); });
    connect(deleteWaypoint, &QPushButton::clicked, map_, &MapWidget::deleteSelectedWaypoint);
    connect(planButton, &QPushButton::clicked, this, &MainWindow::planPath);
    connect(executeButton, &QPushButton::clicked, this, &MainWindow::executePath);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopMotion);
    connect(waypointHeadingCheck_, &QCheckBox::toggled, this, [this](bool checked) {
        if (updatingWaypointControls_) {
            return;
        }
        map_->setSelectedWaypointHeading(checked, waypointHeadingSpin_->value());
    });
    connect(waypointHeadingSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (updatingWaypointControls_) {
            return;
        }
        map_->setSelectedWaypointHeading(waypointHeadingCheck_->isChecked(), value);
    });
    connect(applyCoord, &QPushButton::clicked, this, &MainWindow::applyWaypointCoordinates);
    connect(waypointXSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        applyWaypointCoordinates();
    });
    connect(waypointYSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        applyWaypointCoordinates();
    });
    connect(upButton, &QPushButton::clicked, this, [this]() {
        map_->nudgeSelectedWaypointCm(0.0, waypointStepSpin_->value());
    });
    connect(downButton, &QPushButton::clicked, this, [this]() {
        map_->nudgeSelectedWaypointCm(0.0, -waypointStepSpin_->value());
    });
    connect(leftButton, &QPushButton::clicked, this, [this]() {
        map_->nudgeSelectedWaypointCm(-waypointStepSpin_->value(), 0.0);
    });
    connect(rightButton, &QPushButton::clicked, this, [this]() {
        map_->nudgeSelectedWaypointCm(waypointStepSpin_->value(), 0.0);
    });
    return page;
}

QWidget *MainWindow::createPeripheralTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    suckSpeedSpin_ = new QSpinBox(page);
    suckSpeedSpin_->setRange(0, 100);
    suckSpeedSpin_->setSuffix(QStringLiteral(" %"));
    auto *suckButton = new QPushButton(QStringLiteral("设置吸力电机"), page);

    kickSpeedSpin_ = new QSpinBox(page);
    kickSpeedSpin_->setRange(0, 100);
    kickSpeedSpin_->setSuffix(QStringLiteral(" %"));
    kickReverseCheck_ = new QCheckBox(QStringLiteral("踢球电机反向"), page);
    auto *kickButton = new QPushButton(QStringLiteral("设置踢球电机"), page);

    dctCheck_ = new QCheckBox(QStringLiteral("继电器开启"), page);
    motionEnableCheck_ = new QCheckBox(QStringLiteral("底盘运动使能"), page);
    motionEnableCheck_->setChecked(true);

    auto *ballButton = new QPushButton(QStringLiteral("查询吸球检测"), page);
    auto *irButton = new QPushButton(QStringLiteral("查询红外通道"), page);
    auto *irNormal = new QPushButton(QStringLiteral("红外普通模式 pt"), page);
    auto *irMod = new QPushButton(QStringLiteral("红外调制模式 tz"), page);
    auto *yawZero = new QPushButton(QStringLiteral("Yaw 清零"), page);
    auto *resetMcu = new QPushButton(QStringLiteral("复位 MCU"), page);

    layout->addWidget(suckSpeedSpin_);
    layout->addWidget(suckButton);
    layout->addWidget(kickSpeedSpin_);
    layout->addWidget(kickReverseCheck_);
    layout->addWidget(kickButton);
    layout->addWidget(dctCheck_);
    layout->addWidget(motionEnableCheck_);
    layout->addWidget(ballButton);
    layout->addWidget(irButton);
    layout->addWidget(irNormal);
    layout->addWidget(irMod);
    layout->addWidget(yawZero);
    layout->addWidget(resetMcu);
    layout->addStretch(1);

    connect(suckButton, &QPushButton::clicked, this, [this]() {
        sendCommand(QStringLiteral("cmd_suck %1").arg(suckSpeedSpin_->value()));
    });
    connect(kickButton, &QPushButton::clicked, this, [this]() {
        sendCommand(QStringLiteral("cmd_tqdj %1 %2")
                        .arg(kickSpeedSpin_->value())
                        .arg(kickReverseCheck_->isChecked() ? 1 : 0));
    });
    connect(dctCheck_, &QCheckBox::toggled, this, [this](bool checked) {
        sendCommand(QStringLiteral("cmd_dct %1").arg(checked ? 1 : 0));
    });
    connect(motionEnableCheck_, &QCheckBox::toggled, this, [this](bool checked) {
        sendCommand(QStringLiteral("cmd_conmotion %1").arg(checked ? 1 : 0));
    });
    connect(ballButton, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("cmd_xqcx")); });
    connect(irButton, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("cmd_infred")); });
    connect(irNormal, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("cmd_infred_mode pt")); });
    connect(irMod, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("cmd_infred_mode tz")); });
    connect(yawZero, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("cmd_anglecal")); });
    connect(resetMcu, &QPushButton::clicked, this, [this]() { sendCommand(QStringLiteral("cmd_mcureset")); });
    return page;
}

void MainWindow::refreshPorts()
{
    portCombo_->clear();
    portCombo_->addItems(SerialClient::availablePorts());
}

void MainWindow::connectOrDisconnect()
{
    if (serial_.isSimulated()) {
        log(QStringLiteral("模拟模式不需要连接串口。"));
        return;
    }
    if (serial_.isConnected()) {
        serial_.closePort();
        return;
    }
    if (portCombo_->currentText().isEmpty()) {
        log(QStringLiteral("未选择串口。"));
        return;
    }
    serial_.openPort(portCombo_->currentText());
}

void MainWindow::handlePayload(const QString &payload)
{
    const QStringList parts = payload.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (parts.isEmpty()) {
        return;
    }

    if (parts.at(0) == QStringLiteral("cmd_request") && parts.size() >= 5) {
        bool okDx = false;
        bool okDy = false;
        const double dx = parts.at(1).toDouble(&okDx);
        const double dy = parts.at(2).toDouble(&okDy);
        bool okDyaw = false;
        const double dyaw = parts.at(3).toDouble(&okDyaw);
        if (okDx && okDy && okDyaw) {
            const QPointF mapDelta = odomDeltaToMapCm(QPointF(dx, dy));
            const double mapYaw = std::fmod(map_->robotYawDeg() + dyaw + 360.0, 360.0);
            map_->moveRobotByCm(mapDelta.x(), mapDelta.y(), mapYaw);
            robotYawSpin_->blockSignals(true);
            robotYawSpin_->setValue(mapYaw);
            robotYawSpin_->blockSignals(false);
        }
        return;
    }

    if (executionState_ == ExecutionState::WaitingDistance &&
        parts.at(0) == QStringLiteral("cmd_dis") &&
        parts.size() >= 2 &&
        parts.at(1) == QStringLiteral("done")) {
        double headingDeg = 0.0;
        if (headingForWaypointNear(currentTarget_, &headingDeg)) {
            executionState_ = ExecutionState::WaitingTurn;
            sendCommand(QStringLiteral("cmd_turn %1").arg(headingDeg, 0, 'f', 2));
        } else {
            sendNextPathSegment();
        }
        return;
    }

    if (executionState_ == ExecutionState::WaitingTurn &&
        parts.at(0) == QStringLiteral("cmd_turn") &&
        parts.size() >= 2 &&
        parts.at(1) == QStringLiteral("done")) {
        sendNextPathSegment();
    }
}

void MainWindow::planPath()
{
    if (!ensureReadyForPlanning()) {
        return;
    }
    const QVector<Waypoint> waypoints = map_->waypoints();
    if (waypoints.isEmpty()) {
        log(QStringLiteral("请至少添加一个路径点。"));
        return;
    }

    QVector<QPointF> fullPath;
    QPointF start = map_->robotPixel();
    const double inflatePx = 12.5 / map_->cmPerPixel();
    if (!PathPlanner::pointIsFree(map_->boundary(), map_->obstacles(), start, inflatePx)) {
        log(QStringLiteral("路径规划失败：小车当前位置不在可通行区域内，或距离边界/障碍物太近。"));
        return;
    }
    for (int i = 0; i < waypoints.size(); ++i) {
        if (!PathPlanner::pointIsFree(map_->boundary(), map_->obstacles(), waypoints.at(i).position, inflatePx)) {
            log(QStringLiteral("路径规划失败：第 %1 个目标点不在可通行区域内，或距离边界/障碍物太近。").arg(i + 1));
            return;
        }
        const QVector<QPointF> segment = PathPlanner::plan(map_->boundary(),
                                                           map_->obstacles(),
                                                           start,
                                                           waypoints.at(i).position,
                                                           map_->cmPerPixel());
        if (segment.isEmpty()) {
            log(QStringLiteral("路径规划失败：无法到达第 %1 个目标点。").arg(i + 1));
            return;
        }
        for (int j = 0; j < segment.size(); ++j) {
            if (!fullPath.isEmpty() && j == 0) {
                continue;
            }
            fullPath << segment.at(j);
        }
        start = waypoints.at(i).position;
    }
    map_->setPlannedPath(fullPath);
    log(QStringLiteral("路径规划完成：串联 %1 个目标点，共 %2 个路径节点。")
            .arg(waypoints.size())
            .arg(fullPath.size()));
}

void MainWindow::executePath()
{
    if (!ensureReadyForExecution()) {
        return;
    }
    if (map_->plannedPath().size() < 2) {
        planPath();
    }
    executionPath_ = map_->plannedPath();
    if (executionPath_.size() < 2) {
        return;
    }
    executionIndex_ = 1;
    sendNextPathSegment();
}

void MainWindow::stopMotion()
{
    executionState_ = ExecutionState::Idle;
    executionPath_.clear();
    executionIndex_ = 0;
    sendCommand(QStringLiteral("cmd_juststop"));
}

void MainWindow::sendNextPathSegment()
{
    if (executionIndex_ >= executionPath_.size()) {
        executionState_ = ExecutionState::Idle;
        log(QStringLiteral("路径执行完成。"));
        return;
    }

    const QPointF current = map_->robotPixel();
    currentTarget_ = executionPath_.at(executionIndex_);
    const QPointF deltaPx = currentTarget_ - current;
    const QPointF mapDeltaCm(deltaPx.x() * map_->cmPerPixel(),
                             -deltaPx.y() * map_->cmPerPixel());
    const QPointF odomDeltaCm = mapDeltaToOdomCm(mapDeltaCm);
    executionIndex_++;
    executionState_ = ExecutionState::WaitingDistance;
    sendCommand(QStringLiteral("cmd_dis %1 %2 %3")
                    .arg(odomDeltaCm.x(), 0, 'f', 2)
                    .arg(odomDeltaCm.y(), 0, 'f', 2)
                    .arg(speedProfileSpin_->value()));
}

void MainWindow::updateSelectionControls()
{
    const bool hasWaypoint = map_->selectedWaypointIndex() >= 0;
    waypointHeadingCheck_->setEnabled(hasWaypoint);
    waypointHeadingSpin_->setEnabled(hasWaypoint);
    waypointXSpin_->setEnabled(hasWaypoint);
    waypointYSpin_->setEnabled(hasWaypoint);
    waypointStepSpin_->setEnabled(hasWaypoint);
    const QVector<Waypoint> waypoints = map_->waypoints();
    updatingWaypointControls_ = true;
    if (hasWaypoint && map_->selectedWaypointIndex() < waypoints.size()) {
        const Waypoint &waypoint = waypoints.at(map_->selectedWaypointIndex());
        waypointHeadingCheck_->setChecked(waypoint.hasHeading);
        waypointHeadingSpin_->setValue(waypoint.headingDeg);
        QPointF world;
        if (map_->selectedWaypointWorldCm(&world)) {
            waypointXSpin_->setValue(world.x());
            waypointYSpin_->setValue(world.y());
        }
    }
    updatingWaypointControls_ = false;
}

void MainWindow::applyWaypointCoordinates()
{
    if (updatingWaypointControls_) {
        return;
    }
    if (map_->selectedWaypointIndex() < 0) {
        return;
    }
    map_->setSelectedWaypointWorldCm(waypointXSpin_->value(), waypointYSpin_->value());
}

void MainWindow::setInitialRobotYaw(double yawDeg)
{
    odomToMapYawDeg_ = std::fmod(yawDeg + 360.0, 360.0);
    map_->setRobotYaw(odomToMapYawDeg_);
}

void MainWindow::saveConfig()
{
    const QString path = QFileDialog::getSaveFileName(this,
                                                      QStringLiteral("保存配置"),
                                                      defaultConfigPath(),
                                                      QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::warning(this, QStringLiteral("保存失败"), file.errorString());
        return;
    }
    file.write(QJsonDocument(map_->toJson()).toJson(QJsonDocument::Indented));
    log(QStringLiteral("配置已保存：%1").arg(path));
}

void MainWindow::loadConfig()
{
    const QString path = QFileDialog::getOpenFileName(this,
                                                      QStringLiteral("加载配置"),
                                                      defaultConfigPath(),
                                                      QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) {
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, QStringLiteral("加载失败"), file.errorString());
        return;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        QMessageBox::warning(this, QStringLiteral("加载失败"), QStringLiteral("JSON 文件格式无效。"));
        return;
    }
    map_->fromJson(doc.object());
    odomToMapYawDeg_ = map_->robotYawDeg();
    robotYawSpin_->blockSignals(true);
    robotYawSpin_->setValue(odomToMapYawDeg_);
    robotYawSpin_->blockSignals(false);
    log(QStringLiteral("配置已加载：%1").arg(path));
}

void MainWindow::sendRequest()
{
    if (serial_.isConnected()) {
        sendCommand(QStringLiteral("cmd_request"));
    }
}

void MainWindow::sendCommand(const QString &payload)
{
    serial_.sendPayload(payload);
}

void MainWindow::log(const QString &line)
{
    if (logView_ != nullptr) {
        logView_->appendPlainText(line);
    }
    statusBar()->showMessage(line, 6000);
}

bool MainWindow::ensureReadyForPlanning()
{
    if (!map_->isCalibrated()) {
        log(QStringLiteral("请先完成地图标定。"));
        return false;
    }
    if (!map_->hasRobotPose()) {
        log(QStringLiteral("请先设置小车位姿。"));
        return false;
    }
    return true;
}

bool MainWindow::ensureReadyForExecution()
{
    if (!ensureReadyForPlanning()) {
        return false;
    }
    if (!serial_.isConnected()) {
        log(QStringLiteral("请先连接串口，或开启模拟回复。"));
        return false;
    }
    return true;
}

bool MainWindow::headingForWaypointNear(const QPointF &point, double *headingDeg) const
{
    const QVector<Waypoint> waypoints = map_->waypoints();
    for (const Waypoint &waypoint : waypoints) {
        const double dx = waypoint.position.x() - point.x();
        const double dy = waypoint.position.y() - point.y();
        if (std::sqrt((dx * dx) + (dy * dy)) <= 8.0 && waypoint.hasHeading) {
            if (headingDeg != nullptr) {
                *headingDeg = waypoint.headingDeg;
            }
            return true;
        }
    }
    return false;
}

QPointF MainWindow::mapDeltaToOdomCm(const QPointF &mapDeltaCm) const
{
    const double rad = -odomToMapYawDeg_ * kPi / 180.0;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return QPointF((mapDeltaCm.x() * c) - (mapDeltaCm.y() * s),
                   (mapDeltaCm.x() * s) + (mapDeltaCm.y() * c));
}

QPointF MainWindow::odomDeltaToMapCm(const QPointF &odomDeltaCm) const
{
    const double rad = odomToMapYawDeg_ * kPi / 180.0;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return QPointF((odomDeltaCm.x() * c) - (odomDeltaCm.y() * s),
                   (odomDeltaCm.x() * s) + (odomDeltaCm.y() * c));
}

QString MainWindow::defaultConfigPath() const
{
    return QDir::cleanPath(QCoreApplication::applicationDirPath() +
                           QStringLiteral("/map_config.json"));
}

} // namespace rcj
