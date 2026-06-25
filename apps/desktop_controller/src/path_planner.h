#pragma once

#include <QtCore/QPointF>
#include <QtCore/QSizeF>
#include <QtCore/QVector>
#include <QtGui/QPolygonF>

namespace rcj {

enum class ObstacleType {
    Rectangle,
    Circle,
    Polygon,
};

struct Obstacle {
    ObstacleType type = ObstacleType::Rectangle;
    QPointF center;
    QSizeF size;
    double radius = 20.0;
    QPolygonF polygon;
};

struct Waypoint {
    QPointF position;
    bool hasHeading = false;
    double headingDeg = 0.0;
};

class PathPlanner {
public:
    static QVector<QPointF> plan(const QPolygonF &boundary,
                                 const QVector<Obstacle> &obstacles,
                                 const QPointF &start,
                                 const QPointF &goal,
                                 double cmPerPixel);

    static bool pointIsFree(const QPolygonF &boundary,
                            const QVector<Obstacle> &obstacles,
                            const QPointF &point,
                            double inflatePx);

private:
    static double distanceToBoundary(const QPolygonF &boundary, const QPointF &point);
    static bool obstacleContains(const Obstacle &obstacle, const QPointF &point, double inflatePx);
    static double pointSegmentDistance(const QPointF &point, const QPointF &a, const QPointF &b);
    static QVector<QPointF> simplifyPath(const QPolygonF &boundary,
                                         const QVector<Obstacle> &obstacles,
                                         const QVector<QPointF> &path,
                                         double inflatePx,
                                         double stepPx);
    static bool lineIsFree(const QPolygonF &boundary,
                           const QVector<Obstacle> &obstacles,
                           const QPointF &a,
                           const QPointF &b,
                           double inflatePx,
                           double stepPx);
};

} // namespace rcj
