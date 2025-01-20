struct Waypoint {
    double x;
    double y;
    double heading;
};

struct Point {
    double x;
    double y;
};

struct Circlecenter {
    Point center;
    bool isLeft;
};

struct Segment {
    Waypoint start;
    Circlecenter start_c;
    std::pair<Point, Point> tangents;
    Waypoint end;
    Circlecenter end_c;
};

Circlecenter computeCircleCenter(const Waypoint& wp, double radius, bool isLeft);
std::pair<Point, Point> computeTangentPoints(const Circlecenter& c1, const Circlecenter& c2, double radius);
double distance(const Point& p1, const Point& p2);
std::vector<Point> generateArcPoints(const Circlecenter& c, const Point& start, const Point& end, double radius, int steps);
double calculatePathDistance(const Waypoint& start, const Circlecenter& start_c, const Waypoint& end, const Circlecenter& end_c, const Point& tangent1, const Point& tangent2, double radius, int arcSteps);
Segment findBestSegment(const std::vector<Segment>& segments, double radius, int arcSteps);
std::vector<Point> generateShortestRoute(const Waypoint& start, const Waypoint& end, double radius, int arcSteps);