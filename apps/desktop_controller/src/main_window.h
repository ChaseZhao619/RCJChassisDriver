#pragma once

#include "map_widget.h"
#include "serial_client.h"

#include <QtCore/QTimer>
#include <QtWidgets/QMainWindow>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QPlainTextEdit;
class QSpinBox;

namespace rcj {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void refreshPorts();
    void connectOrDisconnect();
    void handlePayload(const QString &payload);
    void planPath();
    void executePath();
    void stopMotion();
    void sendNextPathSegment();
    void updateSelectionControls();
    void applyWaypointCoordinates();
    void setInitialRobotYaw(double yawDeg);
    void saveConfig();
    void loadConfig();
    void sendRequest();

private:
    enum class ExecutionState {
        Idle,
        WaitingDistance,
        WaitingTurn,
    };

    QWidget *createSidePanel();
    QWidget *createConnectionTab();
    QWidget *createMapTab();
    QWidget *createObstacleTab();
    QWidget *createMotionTab();
    QWidget *createPeripheralTab();
    void sendCommand(const QString &payload);
    void log(const QString &line);
    bool ensureReadyForPlanning();
    bool ensureReadyForExecution();
    bool headingForWaypointNear(const QPointF &point, double *headingDeg) const;
    QPointF mapDeltaToOdomCm(const QPointF &mapDeltaCm) const;
    QPointF odomDeltaToMapCm(const QPointF &odomDeltaCm) const;
    QString defaultConfigPath() const;

    MapWidget *map_ = nullptr;
    SerialClient serial_;
    QTimer requestTimer_;

    QComboBox *portCombo_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QCheckBox *simulateCheck_ = nullptr;
    QPlainTextEdit *logView_ = nullptr;
    QLabel *statusLabel_ = nullptr;

    QDoubleSpinBox *calibDistanceSpin_ = nullptr;
    QDoubleSpinBox *robotYawSpin_ = nullptr;
    QDoubleSpinBox *obstacleWidthSpin_ = nullptr;
    QDoubleSpinBox *obstacleHeightSpin_ = nullptr;
    QDoubleSpinBox *obstacleRadiusSpin_ = nullptr;
    QCheckBox *waypointHeadingCheck_ = nullptr;
    QDoubleSpinBox *waypointHeadingSpin_ = nullptr;
    QDoubleSpinBox *waypointXSpin_ = nullptr;
    QDoubleSpinBox *waypointYSpin_ = nullptr;
    QDoubleSpinBox *waypointStepSpin_ = nullptr;
    QSpinBox *speedProfileSpin_ = nullptr;
    QSpinBox *suckSpeedSpin_ = nullptr;
    QSpinBox *kickSpeedSpin_ = nullptr;
    QCheckBox *kickReverseCheck_ = nullptr;
    QCheckBox *dctCheck_ = nullptr;
    QCheckBox *motionEnableCheck_ = nullptr;

    QVector<QPointF> executionPath_;
    int executionIndex_ = 0;
    ExecutionState executionState_ = ExecutionState::Idle;
    QPointF currentTarget_;
    bool updatingWaypointControls_ = false;
    double odomToMapYawDeg_ = 0.0;
};

} // namespace rcj
