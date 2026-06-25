#include "path_planner.h"

#include <QtCore/QPoint>
#include <QtCore/QRectF>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace rcj {
namespace {

struct QueueItem {
    int index = 0;
    double score = 0.0;
};

bool operator<(const QueueItem &a, const QueueItem &b)
{
    return a.score > b.score;
}

double distance(const QPointF &a, const QPointF &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return std::sqrt((dx * dx) + (dy * dy));
}

} // namespace

QVector<QPointF> PathPlanner::plan(const QPolygonF &boundary,
                                   const QVector<Obstacle> &obstacles,
                                   const QPointF &start,
                                   const QPointF &goal,
                                   double cmPerPixel)
{
    if (boundary.size() < 3 || cmPerPixel <= 0.0) {
        return {};
    }

    const double gridStepPx = std::max(2.0, 2.0 / cmPerPixel);
    const double inflatePx = 12.5 / cmPerPixel;
    if (!pointIsFree(boundary, obstacles, start, inflatePx) ||
        !pointIsFree(boundary, obstacles, goal, inflatePx)) {
        return {};
    }

    const QRectF bounds = boundary.boundingRect().adjusted(-gridStepPx, -gridStepPx, gridStepPx, gridStepPx);
    const int cols = static_cast<int>(std::ceil(bounds.width() / gridStepPx)) + 1;
    const int rows = static_cast<int>(std::ceil(bounds.height() / gridStepPx)) + 1;
    if (cols <= 0 || rows <= 0 || cols * rows > 2000000) {
        return {};
    }

    auto indexOf = [cols](int x, int y) { return y * cols + x; };
    auto pointOf = [&bounds, gridStepPx](int x, int y) {
        return QPointF(bounds.left() + (static_cast<double>(x) * gridStepPx),
                       bounds.top() + (static_cast<double>(y) * gridStepPx));
    };
    auto gridOf = [&bounds, gridStepPx, cols, rows](const QPointF &point) {
        int x = static_cast<int>(std::round((point.x() - bounds.left()) / gridStepPx));
        int y = static_cast<int>(std::round((point.y() - bounds.top()) / gridStepPx));
        x = std::clamp(x, 0, cols - 1);
        y = std::clamp(y, 0, rows - 1);
        return QPoint(x, y);
    };

    const QPoint startGrid = gridOf(start);
    const QPoint goalGrid = gridOf(goal);
    const int startIndex = indexOf(startGrid.x(), startGrid.y());
    const int goalIndex = indexOf(goalGrid.x(), goalGrid.y());
    const int count = cols * rows;

    QVector<double> gScore(count, std::numeric_limits<double>::infinity());
    QVector<int> parent(count, -1);
    QVector<char> closed(count, 0);
    std::priority_queue<QueueItem> open;

    gScore[startIndex] = 0.0;
    open.push({startIndex, distance(start, goal)});

    const int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };

    while (!open.empty()) {
        const QueueItem item = open.top();
        open.pop();
        if (closed[item.index] != 0) {
            continue;
        }
        closed[item.index] = 1;

        if (item.index == goalIndex) {
            break;
        }

        const int cx = item.index % cols;
        const int cy = item.index / cols;
        const QPointF currentPoint = pointOf(cx, cy);

        for (const auto &dir : dirs) {
            const int nx = cx + dir[0];
            const int ny = cy + dir[1];
            if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) {
                continue;
            }

            const int nextIndex = indexOf(nx, ny);
            if (closed[nextIndex] != 0) {
                continue;
            }

            const QPointF nextPoint = pointOf(nx, ny);
            if (!pointIsFree(boundary, obstacles, nextPoint, inflatePx)) {
                continue;
            }

            const double stepCost = distance(currentPoint, nextPoint);
            const double nextScore = gScore[item.index] + stepCost;
            if (nextScore >= gScore[nextIndex]) {
                continue;
            }

            parent[nextIndex] = item.index;
            gScore[nextIndex] = nextScore;
            open.push({nextIndex, nextScore + distance(nextPoint, goal)});
        }
    }

    if (parent[goalIndex] < 0 && goalIndex != startIndex) {
        return {};
    }

    QVector<QPointF> path;
    int cursor = goalIndex;
    path.prepend(goal);
    while (cursor != startIndex && cursor >= 0) {
        const int x = cursor % cols;
        const int y = cursor / cols;
        path.prepend(pointOf(x, y));
        cursor = parent[cursor];
    }
    path.prepend(start);

    return simplifyPath(boundary, obstacles, path, inflatePx, gridStepPx);
}

bool PathPlanner::pointIsFree(const QPolygonF &boundary,
                              const QVector<Obstacle> &obstacles,
                              const QPointF &point,
                              double inflatePx)
{
    if (!boundary.containsPoint(point, Qt::OddEvenFill)) {
        return false;
    }
    if (distanceToBoundary(boundary, point) < inflatePx) {
        return false;
    }
    for (const Obstacle &obstacle : obstacles) {
        if (obstacleContains(obstacle, point, inflatePx)) {
            return false;
        }
    }
    return true;
}

double PathPlanner::distanceToBoundary(const QPolygonF &boundary, const QPointF &point)
{
    if (boundary.size() < 2) {
        return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < boundary.size(); ++i) {
        const QPointF a = boundary.at(i);
        const QPointF b = boundary.at((i + 1) % boundary.size());
        best = std::min(best, pointSegmentDistance(point, a, b));
    }
    return best;
}

bool PathPlanner::obstacleContains(const Obstacle &obstacle, const QPointF &point, double inflatePx)
{
    if (obstacle.type == ObstacleType::Rectangle) {
        QRectF rect(obstacle.center - QPointF(obstacle.size.width() / 2.0, obstacle.size.height() / 2.0),
                    obstacle.size);
        rect = rect.adjusted(-inflatePx, -inflatePx, inflatePx, inflatePx);
        return rect.contains(point);
    }

    if (obstacle.type == ObstacleType::Circle) {
        return distance(obstacle.center, point) <= (obstacle.radius + inflatePx);
    }

    if (obstacle.polygon.size() >= 3) {
        if (obstacle.polygon.containsPoint(point, Qt::OddEvenFill)) {
            return true;
        }
        for (int i = 0; i < obstacle.polygon.size(); ++i) {
            if (pointSegmentDistance(point,
                                     obstacle.polygon.at(i),
                                     obstacle.polygon.at((i + 1) % obstacle.polygon.size())) <= inflatePx) {
                return true;
            }
        }
    }

    return false;
}

double PathPlanner::pointSegmentDistance(const QPointF &point, const QPointF &a, const QPointF &b)
{
    const double vx = b.x() - a.x();
    const double vy = b.y() - a.y();
    const double wx = point.x() - a.x();
    const double wy = point.y() - a.y();
    const double len2 = (vx * vx) + (vy * vy);
    if (len2 <= 0.000001) {
        return distance(point, a);
    }

    double t = ((wx * vx) + (wy * vy)) / len2;
    t = std::clamp(t, 0.0, 1.0);
    return distance(point, QPointF(a.x() + (t * vx), a.y() + (t * vy)));
}

QVector<QPointF> PathPlanner::simplifyPath(const QPolygonF &boundary,
                                           const QVector<Obstacle> &obstacles,
                                           const QVector<QPointF> &path,
                                           double inflatePx,
                                           double stepPx)
{
    if (path.size() <= 2) {
        return path;
    }

    QVector<QPointF> simplified;
    int anchor = 0;
    simplified << path.first();

    while (anchor < path.size() - 1) {
        int best = anchor + 1;
        for (int candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
            if (lineIsFree(boundary, obstacles, path.at(anchor), path.at(candidate), inflatePx, stepPx)) {
                best = candidate;
                break;
            }
        }
        simplified << path.at(best);
        anchor = best;
    }

    return simplified;
}

bool PathPlanner::lineIsFree(const QPolygonF &boundary,
                             const QVector<Obstacle> &obstacles,
                             const QPointF &a,
                             const QPointF &b,
                             double inflatePx,
                             double stepPx)
{
    const double length = distance(a, b);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / std::max(1.0, stepPx))));
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const QPointF p(a.x() + ((b.x() - a.x()) * t),
                        a.y() + ((b.y() - a.y()) * t));
        if (!pointIsFree(boundary, obstacles, p, inflatePx)) {
            return false;
        }
    }
    return true;
}

} // namespace rcj
