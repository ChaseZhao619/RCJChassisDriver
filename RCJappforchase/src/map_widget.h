#pragma once

#include "path_planner.h"

#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtGui/QImage>
#include <QtWidgets/QWidget>

namespace rcj {

class MapWidget : public QWidget {
    Q_OBJECT

public:
    enum class Mode {
        Select,
        CalibA,
        CalibB,
        RobotPose,
        RobotHeading,
        AddWaypoint,
        AddPolygon,
        Boundary,
    };

    explicit MapWidget(QWidget *parent = nullptr);

    bool loadMap(const QString &path);
    void setMode(Mode mode);
    Mode mode() const;

    bool isCalibrated() const;
    double cmPerPixel() const;
    QPointF pixelToWorldCm(const QPointF &pixel) const;
    QPointF worldCmToPixel(const QPointF &world) const;

    QPolygonF boundary() const;
    QVector<Obstacle> obstacles() const;
    QVector<Waypoint> waypoints() const;
    QVector<QPointF> plannedPath() const;
    QPointF robotPixel() const;
    bool hasRobotPose() const;
    double robotYawDeg() const;

    void applyCalibration(double distanceCm);
    void setRobotYaw(double yawDeg);
    void moveRobotByCm(double dxCm, double dyCm, double yawDeg);
    void setPlannedPath(const QVector<QPointF> &path);
    void clearPlannedPath();

    void addRectangleObstacle();
    void addCircleObstacle();
    void deleteSelected();
    void setSelectedObstacleWidthCm(double widthCm);
    void setSelectedObstacleHeightCm(double heightCm);
    void setSelectedObstacleRadiusCm(double radiusCm);
    int selectedObstacleIndex() const;
    int selectedWaypointIndex() const;
    bool selectedWaypointWorldCm(QPointF *world) const;
    void setSelectedWaypointHeading(bool enabled, double headingDeg);
    void setSelectedWaypointWorldCm(double xCm, double yCm);
    void nudgeSelectedWaypointCm(double dxCm, double dyCm);
    void deleteSelectedWaypoint();

    QJsonObject toJson() const;
    void fromJson(const QJsonObject &json);

signals:
    void statusText(const QString &text);
    void dataChanged();
    void selectionChanged();
    void robotYawEdited(double yawDeg);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    QSize sizeHint() const override;

private:
    QPointF widgetToImage(const QPointF &point) const;
    QPointF imageToWidget(const QPointF &point) const;
    QRectF imageTargetRect() const;
    void resetDefaultBoundary();
    QRect findDarkPixelBounds() const;
    void drawObstacle(QPainter &painter, const Obstacle &obstacle, bool selected);
    int hitTestObstacle(const QPointF &imagePoint) const;
    int hitTestWaypoint(const QPointF &imagePoint) const;
    void moveObstacle(Obstacle &obstacle, const QPointF &delta);
    double pxFromCm(double cm) const;
    double cmFromPx(double px) const;
    QJsonArray pointsToJson(const QPolygonF &points) const;
    QPolygonF pointsFromJson(const QJsonArray &array) const;

    QImage mapImage_;
    QString mapPath_;
    Mode mode_ = Mode::Select;

    bool calibASet_ = false;
    bool calibBSet_ = false;
    QPointF calibA_;
    QPointF calibB_;
    double cmPerPixel_ = 0.0;
    QPointF originPixel_;

    QPolygonF boundary_;
    QPolygonF boundaryDraft_;
    QVector<Obstacle> obstacles_;
    QVector<Waypoint> waypoints_;
    QVector<QPointF> plannedPath_;

    bool robotSet_ = false;
    QPointF robotPixel_;
    double robotYawDeg_ = 0.0;

    int selectedObstacle_ = -1;
    int selectedWaypoint_ = -1;
    bool dragging_ = false;
    QPointF lastMouseImage_;
    QPolygonF polygonDraft_;
};

} // namespace rcj
