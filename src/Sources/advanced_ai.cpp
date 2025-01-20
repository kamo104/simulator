#include "advanced_ai.h"


// Utility function to compute a circle center
Circlecenter computeCircleCenter(const Waypoint& wp, double radius, bool isLeft) {
  double angleOffset = isLeft ? PI / 2 : -PI / 2;
  double angle = wp.heading + angleOffset;
  return { { std::cos(angle) * radius + wp.x, std::sin(angle) * radius + wp.y }, isLeft };
}

// Function to compute tangents between two circles
std::pair<Point, Point> computeTangentPoints(const Circlecenter& c1, const Circlecenter& c2, double radius) {
  std::vector<std::pair<Point, Point>> tangents;
  double dx = c2.center.x - c1.center.x;
  double dy = c2.center.y - c1.center.y;
  double d2 = dx * dx + dy * dy;
  double d = std::sqrt(d2);

  if (d < 2 * radius) {
    // No valid tangents
    return {};
  }

  double r = radius;

  double dr, k;

  if (c1.isLeft && c2.isLeft) {
    k = 1.0;
    dr = 0.0;
  }

  if (c1.isLeft && !c2.isLeft) {
    k = -1.0;
    dr = 2 * r;
  }

  if (!c1.isLeft && c2.isLeft) {
    k = 1.0;
    dr = 2 * r;
  }

  if (!c1.isLeft && !c2.isLeft) {
    k = -1.0;
    dr = 0.0;
  }

  double X = dx / d;
  double Y = dy / d;
  double R = dr / d;


  double a = R * X - k * Y * sqrt(1 - R * R);
  double b = R * Y + k * X * sqrt(1 - R * R);
  double c = r - a * c1.center.x - b * c1.center.y;

  auto y = [=](double x) -> double {
    return -(a / b) * (x - dx) - c / b + dy;
    };

  double xt1 = a * a * c2.center.x - a * a * c1.center.x - 2 * a * b * c1.center.y + a * b * c2.center.y - a * c + b * b * c1.center.x;
  double xt2 = a * a * c2.center.x - a * a * c1.center.x - a * b * c1.center.y - a * c + b * b * c2.center.x;

  Point tangent1 = { xt1, y(xt1) };
  Point tangent2 = { xt2, y(xt2) };

  return { tangent1, tangent2 };


}

// Function to calculate distance between two points
double distance(const Point& p1, const Point& p2) {
  return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Function to generate arc points
std::vector<Point> generateArcPoints(const Circlecenter& c, const Point& start, const Point& end, double radius, int steps) {
  std::vector<Point> arcPoints;
  bool clockwise = !c.isLeft;
  double startAngle = std::atan2(start.y - c.center.y, start.x - c.center.x);
  double endAngle = std::atan2(end.y - c.center.y, end.x - c.center.x);

  // Normalize angles
  if (clockwise && startAngle < endAngle) {
    startAngle += 2 * PI;
  }
  else if (!clockwise && endAngle < startAngle) {
    endAngle += 2 * PI;
  }

  double stepAngle = (endAngle - startAngle) / steps;

  for (int i = 0; i <= steps; ++i) {
    double angle = startAngle + i * stepAngle;
    arcPoints.push_back({
        c.center.x + radius * std::cos(angle),
        c.center.y + radius * std::sin(angle)
      });
  }

  return arcPoints;
}

// Function to calculate the total path distance
double calculatePathDistance(const Waypoint& start, const Circlecenter& start_c, const Waypoint& end, const Circlecenter& end_c, const Point& tangent1, const Point& tangent2, double radius, int arcSteps) {
  // First arc distance
  std::vector<Point> firstArc = generateArcPoints(start_c, { start.x, start.y }, tangent1, radius, arcSteps);

  double firstArcLength = 0.0;
  for (size_t i = 1; i < firstArc.size(); ++i) {
    firstArcLength += distance(firstArc[i - 1], firstArc[i]);
  }

  // Second arc distance
  std::vector<Point> secondArc = generateArcPoints(end_c, tangent2, { end.x, end.y }, radius, arcSteps);

  double secondArcLength = 0.0;
  for (size_t i = 1; i < secondArc.size(); ++i) {
    secondArcLength += distance(secondArc[i - 1], secondArc[i]);
  }

  // Tangent line distance
  double lineDistance = distance(tangent1, tangent2);

  return firstArcLength + lineDistance + secondArcLength;
}

Segment findBestSegment(const std::vector<Segment>& segments, double radius, int arcSteps) {
  Segment bestSegment;
  double minDistance = std::numeric_limits<double>::max();
  for (const auto& segm : segments) {
    double pathDistance = calculatePathDistance(segm.start, segm.start_c, segm.end, segm.end_c, segm.tangents.first, segm.tangents.second, radius, arcSteps);
    if (pathDistance < minDistance) {
      minDistance = pathDistance;
      bestSegment = segm;
    }
  }
  return bestSegment;
}

// Function to generate the shortest route
std::vector<Point> generateShortestRoute(const Waypoint& start, const Waypoint& end, double radius, int arcSteps) {
  std::vector<Point> route;

  // Compute circle centers for start and end points
  Circlecenter startCenter1 = computeCircleCenter(start, radius, true);
  Circlecenter startCenter2 = computeCircleCenter(start, radius, false);
  Circlecenter endCenter1 = computeCircleCenter(end, radius, true);
  Circlecenter endCenter2 = computeCircleCenter(end, radius, false);

  // Compute all tangents
  std::vector<Segment> segments;
  segments.push_back({ start, startCenter1, computeTangentPoints(startCenter1, endCenter1, radius), end, endCenter1 });
  segments.push_back({ start, startCenter1, computeTangentPoints(startCenter1, endCenter2, radius), end, endCenter2 });
  segments.push_back({ start, startCenter2, computeTangentPoints(startCenter2, endCenter1, radius), end, endCenter1 });
  segments.push_back({ start, startCenter2, computeTangentPoints(startCenter2, endCenter2, radius), end, endCenter2 });

  //no path found
  if (segments.size() == 0) {
    return route;
  }

  // Find the shortest path
  Segment bestSegment = findBestSegment(segments, radius, arcSteps);

  // Construct the route using the shortest path
  route.push_back({ start.x, start.y });

  std::vector<Point> firstArc = generateArcPoints(bestSegment.start_c, { start.x, start.y }, bestSegment.tangents.first, radius, arcSteps);
  route.insert(route.end(), firstArc.begin(), firstArc.end());

  route.push_back(bestSegment.tangents.first);
  route.push_back(bestSegment.tangents.second);

  std::vector<Point> secondArc = generateArcPoints(bestSegment.end_c, bestSegment.tangents.second, { end.x, end.y }, radius, arcSteps);
  route.insert(route.end(), secondArc.begin(), secondArc.end());

  route.push_back({ end.x, end.y });

  return route;
}