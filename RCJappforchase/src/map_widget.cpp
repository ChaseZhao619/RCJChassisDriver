#include "map_widget.h"

#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QToolTip>

#include <cmath>

namespace rcj {
namespace {

constexpr double kRobotDiameterCm = 21.0;
constexpr double kDefaultObstacleCm = 30.0;
constexpr double kPi = 3.14159265358979323846;

QJsonObject pointToJson(const QPointF &point)
{
    QJsonObject object;
    object[QStringLiteral("x")] = point.x();
    object[QStringLiteral("y")] = point.y();
    return object;
}

QPointF pointFromJson(const QJsonObject &object)
{
    return QPointF(object.value(QStringLiteral("x")).toDouble(),
                   object.value(QStringLiteral("y")).toDouble());
}

double distance(const QPointF &a, const QPointF &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return std::sqrt((dx * dx) + (dy * dy));
}

} // namespace

MapWidget::MapWidget(QWidget *parent)
    : QWidget(parent)
{
    setMouseTracking(true);
    setMinimumSize(420, 520);
}

bool MapWidget::loadMap(const QString &path)
{
    QImage image(path);
    if (image.isNull()) {
        emit statusText(QStringLiteral("地图加载失败：%1").arg(path));
        return false;
    }

    mapImage_ = image.convertToFormat(QImage::Format_ARGB32);
    mapPath_ = path;
    resetDefaultBoundary();
    update();
    emit statusText(QStringLiteral("地图已加载：%1").arg(path));
    emit dataChanged();
    return true;
}

void MapWidget::setMode(Mode mode)
{
    mode_ = mode;
    if (mode_ != Mode::AddPolygon) {
        polygonDraft_.clear();
    }
    if (mode_ != Mode::Boundary) {
        boundaryDraft_.clear();
    }
    update();
}

MapWidget::Mode MapWidget::mode() const
{
    return mode_;
}

bool MapWidget::isCalibrated() const
{
    return cmPerPixel_ > 0.0;
}

double MapWidget::cmPerPixel() const
{
    return cmPerPixel_;
}

QPointF MapWidget::pixelToWorldCm(const QPointF &pixel) const
{
    if (!isCalibrated()) {
        return {};
    }
    return QPointF((pixel.x() - originPixel_.x()) * cmPerPixel_,
                   (originPixel_.y() - pixel.y()) * cmPerPixel_);
}

QPointF MapWidget::worldCmToPixel(const QPointF &world) const
{
    if (!isCalibrated()) {
        return {};
    }
    return QPointF(originPixel_.x() + (world.x() / cmPerPixel_),
                   originPixel_.y() - (world.y() / cmPerPixel_));
}

QPolygonF MapWidget::boundary() const
{
    return boundary_;
}

QVector<Obstacle> MapWidget::obstacles() const
{
    return obstacles_;
}

QVector<Waypoint> MapWidget::waypoints() const
{
    return waypoints_;
}

QVector<QPointF> MapWidget::plannedPath() const
{
    return plannedPath_;
}

QPointF MapWidget::robotPixel() const
{
    return robotPixel_;
}

bool MapWidget::hasRobotPose() const
{
    return robotSet_;
}

double MapWidget::robotYawDeg() const
{
    return robotYawDeg_;
}

void MapWidget::applyCalibration(double distanceCm)
{
    if (!calibASet_ || !calibBSet_ || distanceCm <= 0.0) {
        emit statusText(QStringLiteral("标定需要两个点和一个大于 0 的真实距离"));
        return;
    }

    const double pixelDistance = distance(calibA_, calibB_);
    if (pixelDistance <= 0.001) {
        emit statusText(QStringLiteral("两个标定点距离太近"));
        return;
    }

    cmPerPixel_ = distanceCm / pixelDistance;
    originPixel_ = boundary_.isEmpty() ? QPointF(0.0, mapImage_.height()) : boundary_.boundingRect().bottomLeft();
    emit statusText(QStringLiteral("标定已应用：%1 cm/px").arg(cmPerPixel_, 0, 'f', 4));
    update();
    emit dataChanged();
}

void MapWidget::setRobotYaw(double yawDeg)
{
    robotYawDeg_ = std::fmod(yawDeg + 360.0, 360.0);
    update();
    emit dataChanged();
    emit robotYawEdited(robotYawDeg_);
}

void MapWidget::moveRobotByCm(double dxCm, double dyCm, double yawDeg)
{
    if (!robotSet_ || !isCalibrated()) {
        return;
    }
    robotPixel_ += QPointF(dxCm / cmPerPixel_, -dyCm / cmPerPixel_);
    robotYawDeg_ = yawDeg;
    update();
    emit dataChanged();
}

void MapWidget::setPlannedPath(const QVector<QPointF> &path)
{
    plannedPath_ = path;
    update();
    emit dataChanged();
}

void MapWidget::clearPlannedPath()
{
    plannedPath_.clear();
    update();
    emit dataChanged();
}

void MapWidget::addRectangleObstacle()
{
    if (!isCalibrated()) {
        emit statusText(QStringLiteral("请先完成地图标定，再添加障碍物"));
        return;
    }
    Obstacle obstacle;
    obstacle.type = ObstacleType::Rectangle;
    obstacle.center = boundary_.isEmpty() ? QPointF(mapImage_.width() / 2.0, mapImage_.height() / 2.0)
                                          : boundary_.boundingRect().center();
    obstacle.size = QSizeF(pxFromCm(kDefaultObstacleCm), pxFromCm(kDefaultObstacleCm));
    obstacles_ << obstacle;
    selectedObstacle_ = obstacles_.size() - 1;
    selectedWaypoint_ = -1;
    update();
    emit selectionChanged();
    emit dataChanged();
}

void MapWidget::addCircleObstacle()
{
    if (!isCalibrated()) {
        emit statusText(QStringLiteral("请先完成地图标定，再添加障碍物"));
        return;
    }
    Obstacle obstacle;
    obstacle.type = ObstacleType::Circle;
    obstacle.center = boundary_.isEmpty() ? QPointF(mapImage_.width() / 2.0, mapImage_.height() / 2.0)
                                          : boundary_.boundingRect().center();
    obstacle.radius = pxFromCm(kDefaultObstacleCm / 2.0);
    obstacles_ << obstacle;
    selectedObstacle_ = obstacles_.size() - 1;
    selectedWaypoint_ = -1;
    update();
    emit selectionChanged();
    emit dataChanged();
}

void MapWidget::deleteSelected()
{
    if (selectedObstacle_ >= 0 && selectedObstacle_ < obstacles_.size()) {
        obstacles_.removeAt(selectedObstacle_);
        selectedObstacle_ = -1;
    } else if (selectedWaypoint_ >= 0 && selectedWaypoint_ < waypoints_.size()) {
        waypoints_.removeAt(selectedWaypoint_);
        selectedWaypoint_ = -1;
    }
    plannedPath_.clear();
    update();
    emit selectionChanged();
    emit dataChanged();
}

void MapWidget::setSelectedObstacleWidthCm(double widthCm)
{
    if (selectedObstacle_ < 0 || selectedObstacle_ >= obstacles_.size() || !isCalibrated()) {
        return;
    }
    Obstacle &obstacle = obstacles_[selectedObstacle_];
    if (obstacle.type == ObstacleType::Rectangle) {
        obstacle.size.setWidth(pxFromCm(widthCm));
    }
    update();
    emit dataChanged();
}

void MapWidget::setSelectedObstacleHeightCm(double heightCm)
{
    if (selectedObstacle_ < 0 || selectedObstacle_ >= obstacles_.size() || !isCalibrated()) {
        return;
    }
    Obstacle &obstacle = obstacles_[selectedObstacle_];
    if (obstacle.type == ObstacleType::Rectangle) {
        obstacle.size.setHeight(pxFromCm(heightCm));
    }
    update();
    emit dataChanged();
}

void MapWidget::setSelectedObstacleRadiusCm(double radiusCm)
{
    if (selectedObstacle_ < 0 || selectedObstacle_ >= obstacles_.size() || !isCalibrated()) {
        return;
    }
    Obstacle &obstacle = obstacles_[selectedObstacle_];
    if (obstacle.type == ObstacleType::Circle) {
        obstacle.radius = pxFromCm(radiusCm);
    }
    update();
    emit dataChanged();
}

int MapWidget::selectedObstacleIndex() const
{
    return selectedObstacle_;
}

int MapWidget::selectedWaypointIndex() const
{
    return selectedWaypoint_;
}

bool MapWidget::selectedWaypointWorldCm(QPointF *world) const
{
    if (selectedWaypoint_ < 0 || selectedWaypoint_ >= waypoints_.size() || !isCalibrated()) {
        return false;
    }
    if (world != nullptr) {
        *world = pixelToWorldCm(waypoints_.at(selectedWaypoint_).position);
    }
    return true;
}

void MapWidget::setSelectedWaypointHeading(bool enabled, double headingDeg)
{
    if (selectedWaypoint_ < 0 || selectedWaypoint_ >= waypoints_.size()) {
        return;
    }
    waypoints_[selectedWaypoint_].hasHeading = enabled;
    waypoints_[selectedWaypoint_].headingDeg = headingDeg;
    update();
    emit dataChanged();
}

void MapWidget::setSelectedWaypointWorldCm(double xCm, double yCm)
{
    if (selectedWaypoint_ < 0 || selectedWaypoint_ >= waypoints_.size() || !isCalibrated()) {
        return;
    }
    waypoints_[selectedWaypoint_].position = worldCmToPixel(QPointF(xCm, yCm));
    plannedPath_.clear();
    update();
    emit selectionChanged();
    emit dataChanged();
}

void MapWidget::nudgeSelectedWaypointCm(double dxCm, double dyCm)
{
    QPointF world;
    if (!selectedWaypointWorldCm(&world)) {
        return;
    }
    setSelectedWaypointWorldCm(world.x() + dxCm, world.y() + dyCm);
}

void MapWidget::deleteSelectedWaypoint()
{
    if (selectedWaypoint_ < 0 || selectedWaypoint_ >= waypoints_.size()) {
        return;
    }
    waypoints_.removeAt(selectedWaypoint_);
    selectedWaypoint_ = -1;
    plannedPath_.clear();
    update();
    emit selectionChanged();
    emit dataChanged();
}

QJsonObject MapWidget::toJson() const
{
    QJsonObject json;
    json[QStringLiteral("mapPath")] = mapPath_;
    json[QStringLiteral("cmPerPixel")] = cmPerPixel_;
    json[QStringLiteral("originPixel")] = pointToJson(originPixel_);
    json[QStringLiteral("calibASet")] = calibASet_;
    json[QStringLiteral("calibBSet")] = calibBSet_;
    json[QStringLiteral("calibA")] = pointToJson(calibA_);
    json[QStringLiteral("calibB")] = pointToJson(calibB_);
    json[QStringLiteral("boundary")] = pointsToJson(boundary_);
    json[QStringLiteral("robotSet")] = robotSet_;
    json[QStringLiteral("robotPixel")] = pointToJson(robotPixel_);
    json[QStringLiteral("robotYawDeg")] = robotYawDeg_;

    QJsonArray obstaclesJson;
    for (const Obstacle &obstacle : obstacles_) {
        QJsonObject object;
        object[QStringLiteral("type")] = obstacle.type == ObstacleType::Rectangle
            ? QStringLiteral("rectangle")
            : (obstacle.type == ObstacleType::Circle ? QStringLiteral("circle") : QStringLiteral("polygon"));
        object[QStringLiteral("center")] = pointToJson(obstacle.center);
        object[QStringLiteral("width")] = obstacle.size.width();
        object[QStringLiteral("height")] = obstacle.size.height();
        object[QStringLiteral("radius")] = obstacle.radius;
        object[QStringLiteral("polygon")] = pointsToJson(obstacle.polygon);
        obstaclesJson.append(object);
    }
    json[QStringLiteral("obstacles")] = obstaclesJson;

    QJsonArray waypointsJson;
    for (const Waypoint &waypoint : waypoints_) {
        QJsonObject object;
        object[QStringLiteral("position")] = pointToJson(waypoint.position);
        object[QStringLiteral("hasHeading")] = waypoint.hasHeading;
        object[QStringLiteral("headingDeg")] = waypoint.headingDeg;
        waypointsJson.append(object);
    }
    json[QStringLiteral("waypoints")] = waypointsJson;
    return json;
}

void MapWidget::fromJson(const QJsonObject &json)
{
    cmPerPixel_ = json.value(QStringLiteral("cmPerPixel")).toDouble(0.0);
    originPixel_ = pointFromJson(json.value(QStringLiteral("originPixel")).toObject());
    calibASet_ = json.value(QStringLiteral("calibASet")).toBool(false);
    calibBSet_ = json.value(QStringLiteral("calibBSet")).toBool(false);
    calibA_ = pointFromJson(json.value(QStringLiteral("calibA")).toObject());
    calibB_ = pointFromJson(json.value(QStringLiteral("calibB")).toObject());
    boundary_ = pointsFromJson(json.value(QStringLiteral("boundary")).toArray());
    robotSet_ = json.value(QStringLiteral("robotSet")).toBool(false);
    robotPixel_ = pointFromJson(json.value(QStringLiteral("robotPixel")).toObject());
    robotYawDeg_ = json.value(QStringLiteral("robotYawDeg")).toDouble(0.0);

    obstacles_.clear();
    const QJsonArray obstaclesJson = json.value(QStringLiteral("obstacles")).toArray();
    for (const QJsonValue &value : obstaclesJson) {
        const QJsonObject object = value.toObject();
        Obstacle obstacle;
        const QString type = object.value(QStringLiteral("type")).toString();
        if (type == QStringLiteral("circle")) {
            obstacle.type = ObstacleType::Circle;
        } else if (type == QStringLiteral("polygon")) {
            obstacle.type = ObstacleType::Polygon;
        } else {
            obstacle.type = ObstacleType::Rectangle;
        }
        obstacle.center = pointFromJson(object.value(QStringLiteral("center")).toObject());
        obstacle.size = QSizeF(object.value(QStringLiteral("width")).toDouble(),
                               object.value(QStringLiteral("height")).toDouble());
        obstacle.radius = object.value(QStringLiteral("radius")).toDouble();
        obstacle.polygon = pointsFromJson(object.value(QStringLiteral("polygon")).toArray());
        obstacles_ << obstacle;
    }

    waypoints_.clear();
    const QJsonArray waypointsJson = json.value(QStringLiteral("waypoints")).toArray();
    for (const QJsonValue &value : waypointsJson) {
        const QJsonObject object = value.toObject();
        Waypoint waypoint;
        waypoint.position = pointFromJson(object.value(QStringLiteral("position")).toObject());
        waypoint.hasHeading = object.value(QStringLiteral("hasHeading")).toBool(false);
        waypoint.headingDeg = object.value(QStringLiteral("headingDeg")).toDouble(0.0);
        waypoints_ << waypoint;
    }

    plannedPath_.clear();
    selectedObstacle_ = -1;
    selectedWaypoint_ = -1;
    update();
    emit selectionChanged();
    emit dataChanged();
}

void MapWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.fillRect(rect(), QColor(247, 248, 250));
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (mapImage_.isNull()) {
        painter.setPen(Qt::black);
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("未加载地图"));
        return;
    }

    const QRectF target = imageTargetRect();
    painter.drawImage(target, mapImage_);
    painter.save();
    painter.translate(target.topLeft());
    painter.scale(target.width() / mapImage_.width(), target.height() / mapImage_.height());

    painter.setPen(QPen(QColor(190, 35, 35), 2.0));
    painter.setBrush(QColor(190, 35, 35, 25));
    if (boundary_.size() >= 3) {
        painter.drawPolygon(boundary_);
    }
    painter.setPen(QPen(QColor(190, 80, 30), 1.5, Qt::DashLine));
    if (boundaryDraft_.size() >= 2) {
        painter.drawPolyline(boundaryDraft_);
    }

    for (int i = 0; i < obstacles_.size(); ++i) {
        drawObstacle(painter, obstacles_.at(i), i == selectedObstacle_);
    }

    painter.setPen(QPen(QColor(33, 120, 75), 2.0));
    painter.setBrush(Qt::NoBrush);
    if (plannedPath_.size() >= 2) {
        painter.drawPolyline(QPolygonF(plannedPath_));
    }

    painter.setPen(QPen(QColor(23, 83, 180), 2.0));
    painter.setBrush(QColor(23, 83, 180));
    for (int i = 0; i < waypoints_.size(); ++i) {
        const Waypoint &waypoint = waypoints_.at(i);
        const double r = (i == selectedWaypoint_) ? 6.0 : 4.0;
        painter.drawEllipse(waypoint.position, r, r);
        painter.drawText(waypoint.position + QPointF(6.0, -6.0), QString::number(i + 1));
        if (waypoint.hasHeading) {
            const double rad = waypoint.headingDeg * kPi / 180.0;
            const QPointF end = waypoint.position + QPointF(std::cos(rad) * 18.0, -std::sin(rad) * 18.0);
            painter.drawLine(waypoint.position, end);
        }
    }

    if (polygonDraft_.size() >= 1) {
        painter.setPen(QPen(QColor(160, 80, 0), 1.5, Qt::DashLine));
        painter.drawPolyline(polygonDraft_);
    }

    if (calibASet_ || calibBSet_) {
        painter.setPen(QPen(QColor(100, 30, 140), 1.5));
        painter.setBrush(QColor(100, 30, 140));
        if (calibASet_) {
            painter.drawEllipse(calibA_, 4.0, 4.0);
            painter.drawText(calibA_ + QPointF(5.0, -5.0), QStringLiteral("A"));
        }
        if (calibBSet_) {
            painter.drawEllipse(calibB_, 4.0, 4.0);
            painter.drawText(calibB_ + QPointF(5.0, -5.0), QStringLiteral("B"));
        }
        if (calibASet_ && calibBSet_) {
            painter.drawLine(calibA_, calibB_);
        }
    }

    if (robotSet_) {
        const double radiusPx = isCalibrated() ? pxFromCm(kRobotDiameterCm / 2.0) : 14.0;
        painter.setPen(QPen(QColor(30, 30, 30), 2.0));
        painter.setBrush(QColor(250, 205, 70, 180));
        painter.drawEllipse(robotPixel_, radiusPx, radiusPx);
        const double rad = robotYawDeg_ * kPi / 180.0;
        const QPointF front = robotPixel_ + QPointF(std::cos(rad) * radiusPx, -std::sin(rad) * radiusPx);
        painter.drawLine(robotPixel_, front);
    }

    painter.restore();
}

void MapWidget::mousePressEvent(QMouseEvent *event)
{
    if (mapImage_.isNull()) {
        return;
    }

    const QPointF imagePoint = widgetToImage(event->position());
    if (imagePoint.x() < 0.0 || imagePoint.y() < 0.0 ||
        imagePoint.x() >= mapImage_.width() || imagePoint.y() >= mapImage_.height()) {
        return;
    }

    if (mode_ == Mode::CalibA) {
        calibA_ = imagePoint;
        calibASet_ = true;
        emit statusText(QStringLiteral("标定点 A 已设置"));
    } else if (mode_ == Mode::CalibB) {
        calibB_ = imagePoint;
        calibBSet_ = true;
        emit statusText(QStringLiteral("标定点 B 已设置"));
    } else if (mode_ == Mode::RobotPose) {
        robotPixel_ = imagePoint;
        robotSet_ = true;
        emit statusText(QStringLiteral("小车位姿已设置"));
    } else if (mode_ == Mode::RobotHeading) {
        if (!robotSet_) {
            emit statusText(QStringLiteral("请先设置小车中心，再设置车头方向"));
            return;
        }
        const QPointF delta = imagePoint - robotPixel_;
        if (distance(robotPixel_, imagePoint) < 1.0) {
            emit statusText(QStringLiteral("车头方向点距离小车中心太近"));
            return;
        }
        robotYawDeg_ = std::atan2(-delta.y(), delta.x()) * 180.0 / kPi;
        robotYawDeg_ = std::fmod(robotYawDeg_ + 360.0, 360.0);
        emit statusText(QStringLiteral("小车车头朝向已设置：%1 deg").arg(robotYawDeg_, 0, 'f', 1));
        emit robotYawEdited(robotYawDeg_);
    } else if (mode_ == Mode::AddWaypoint) {
        Waypoint waypoint;
        waypoint.position = imagePoint;
        waypoints_ << waypoint;
        selectedWaypoint_ = waypoints_.size() - 1;
        selectedObstacle_ = -1;
        emit statusText(QStringLiteral("路径点已添加"));
    } else if (mode_ == Mode::AddPolygon) {
        if (event->button() == Qt::RightButton) {
            if (polygonDraft_.size() >= 3) {
                Obstacle obstacle;
                obstacle.type = ObstacleType::Polygon;
                obstacle.polygon = polygonDraft_;
                obstacle.center = polygonDraft_.boundingRect().center();
                obstacles_ << obstacle;
                selectedObstacle_ = obstacles_.size() - 1;
                polygonDraft_.clear();
                emit statusText(QStringLiteral("多边形障碍物已添加"));
            }
        } else {
            polygonDraft_ << imagePoint;
        }
    } else if (mode_ == Mode::Boundary) {
        if (event->button() == Qt::RightButton) {
            if (boundaryDraft_.size() >= 3) {
                boundary_ = boundaryDraft_;
                boundaryDraft_.clear();
                originPixel_ = boundary_.boundingRect().bottomLeft();
                emit statusText(QStringLiteral("边界已更新"));
            }
        } else {
            boundaryDraft_ << imagePoint;
        }
    } else {
        selectedObstacle_ = hitTestObstacle(imagePoint);
        selectedWaypoint_ = (selectedObstacle_ >= 0) ? -1 : hitTestWaypoint(imagePoint);
        dragging_ = selectedObstacle_ >= 0 || selectedWaypoint_ >= 0;
        lastMouseImage_ = imagePoint;
    }

    update();
    emit selectionChanged();
    emit dataChanged();
}

void MapWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (mapImage_.isNull()) {
        return;
    }

    const QPointF imagePoint = widgetToImage(event->position());
    if (isCalibrated()) {
        const QPointF world = pixelToWorldCm(imagePoint);
        QToolTip::showText(event->globalPosition().toPoint(),
                           QStringLiteral("x=%1 cm, y=%2 cm")
                               .arg(world.x(), 0, 'f', 1)
                               .arg(world.y(), 0, 'f', 1),
                           this);
    }

    if (!dragging_) {
        return;
    }

    const QPointF delta = imagePoint - lastMouseImage_;
    lastMouseImage_ = imagePoint;
    if (selectedObstacle_ >= 0 && selectedObstacle_ < obstacles_.size()) {
        moveObstacle(obstacles_[selectedObstacle_], delta);
    } else if (selectedWaypoint_ >= 0 && selectedWaypoint_ < waypoints_.size()) {
        waypoints_[selectedWaypoint_].position += delta;
        plannedPath_.clear();
        update();
        emit selectionChanged();
        emit dataChanged();
        return;
    }
    plannedPath_.clear();
    update();
    emit dataChanged();
}

void MapWidget::mouseReleaseEvent(QMouseEvent *)
{
    dragging_ = false;
}

QSize MapWidget::sizeHint() const
{
    return QSize(720, 860);
}

QPointF MapWidget::widgetToImage(const QPointF &point) const
{
    const QRectF target = imageTargetRect();
    if (target.width() <= 0.0 || target.height() <= 0.0) {
        return {};
    }
    return QPointF((point.x() - target.left()) * mapImage_.width() / target.width(),
                   (point.y() - target.top()) * mapImage_.height() / target.height());
}

QPointF MapWidget::imageToWidget(const QPointF &point) const
{
    const QRectF target = imageTargetRect();
    return QPointF(target.left() + (point.x() * target.width() / mapImage_.width()),
                   target.top() + (point.y() * target.height() / mapImage_.height()));
}

QRectF MapWidget::imageTargetRect() const
{
    if (mapImage_.isNull()) {
        return {};
    }
    const QSizeF imageSize(mapImage_.size());
    QSizeF scaled = imageSize;
    scaled.scale(size(), Qt::KeepAspectRatio);
    return QRectF(QPointF((width() - scaled.width()) / 2.0, (height() - scaled.height()) / 2.0), scaled);
}

void MapWidget::resetDefaultBoundary()
{
    const QRect darkBounds = findDarkPixelBounds();
    QRectF bounds = darkBounds.isValid() ? QRectF(darkBounds) : QRectF(0.0, 0.0, mapImage_.width(), mapImage_.height());
    boundary_.clear();
    boundary_ << bounds.topLeft() << bounds.topRight() << bounds.bottomRight() << bounds.bottomLeft();
    originPixel_ = bounds.bottomLeft();
}

QRect MapWidget::findDarkPixelBounds() const
{
    if (mapImage_.isNull()) {
        return {};
    }
    int minX = mapImage_.width();
    int minY = mapImage_.height();
    int maxX = -1;
    int maxY = -1;
    for (int y = 0; y < mapImage_.height(); ++y) {
        const QRgb *line = reinterpret_cast<const QRgb *>(mapImage_.constScanLine(y));
        for (int x = 0; x < mapImage_.width(); ++x) {
            const QColor color(line[x]);
            if (color.red() < 80 && color.green() < 80 && color.blue() < 80) {
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
            }
        }
    }
    if (maxX < minX || maxY < minY) {
        return {};
    }
    return QRect(QPoint(minX, minY), QPoint(maxX, maxY));
}

void MapWidget::drawObstacle(QPainter &painter, const Obstacle &obstacle, bool selected)
{
    painter.setPen(QPen(selected ? QColor(215, 80, 30) : QColor(160, 55, 55), selected ? 2.5 : 1.5));
    painter.setBrush(selected ? QColor(230, 95, 45, 110) : QColor(190, 55, 55, 85));
    if (obstacle.type == ObstacleType::Rectangle) {
        const QRectF rect(obstacle.center - QPointF(obstacle.size.width() / 2.0, obstacle.size.height() / 2.0),
                          obstacle.size);
        painter.drawRect(rect);
    } else if (obstacle.type == ObstacleType::Circle) {
        painter.drawEllipse(obstacle.center, obstacle.radius, obstacle.radius);
    } else {
        painter.drawPolygon(obstacle.polygon);
    }
}

int MapWidget::hitTestObstacle(const QPointF &imagePoint) const
{
    for (int i = obstacles_.size() - 1; i >= 0; --i) {
        const Obstacle &obstacle = obstacles_.at(i);
        if (obstacle.type == ObstacleType::Rectangle) {
            const QRectF rect(obstacle.center - QPointF(obstacle.size.width() / 2.0, obstacle.size.height() / 2.0),
                              obstacle.size);
            if (rect.adjusted(-5.0, -5.0, 5.0, 5.0).contains(imagePoint)) {
                return i;
            }
        } else if (obstacle.type == ObstacleType::Circle) {
            if (distance(obstacle.center, imagePoint) <= obstacle.radius + 5.0) {
                return i;
            }
        } else if (obstacle.polygon.containsPoint(imagePoint, Qt::OddEvenFill)) {
            return i;
        }
    }
    return -1;
}

int MapWidget::hitTestWaypoint(const QPointF &imagePoint) const
{
    for (int i = waypoints_.size() - 1; i >= 0; --i) {
        if (distance(waypoints_.at(i).position, imagePoint) <= 8.0) {
            return i;
        }
    }
    return -1;
}

void MapWidget::moveObstacle(Obstacle &obstacle, const QPointF &delta)
{
    obstacle.center += delta;
    if (obstacle.type == ObstacleType::Polygon) {
        for (int i = 0; i < obstacle.polygon.size(); ++i) {
            obstacle.polygon[i] += delta;
        }
    }
}

double MapWidget::pxFromCm(double cm) const
{
    return isCalibrated() ? (cm / cmPerPixel_) : cm;
}

double MapWidget::cmFromPx(double px) const
{
    return isCalibrated() ? (px * cmPerPixel_) : px;
}

QJsonArray MapWidget::pointsToJson(const QPolygonF &points) const
{
    QJsonArray array;
    for (const QPointF &point : points) {
        array.append(pointToJson(point));
    }
    return array;
}

QPolygonF MapWidget::pointsFromJson(const QJsonArray &array) const
{
    QPolygonF points;
    for (const QJsonValue &value : array) {
        points << pointFromJson(value.toObject());
    }
    return points;
}

} // namespace rcj
