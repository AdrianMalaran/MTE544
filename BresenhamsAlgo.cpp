#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>

using std::cout;
using std::cerr;
using std::endl;

struct Coordinate {
    int x;
    int y;
};

std::vector<Coordinate> findAllPointsAlongLine(int x1, int y1, int x2, int y2) {
    std::vector<Coordinate> ptsAlongLine;

    int slope_x1 = 0;
    int slope_y1 = 0;
    int slope_x2 = 0;
    int slope_y2 = 0;

    int width = x2 - x1;
    int height = y2 - y1;
    int xTracker = x1;
    int yTracker = y1;

    if (width < 0) {
        slope_x1 = -1;
        slope_x2 = -1;
    } else if (width > 0) {
        slope_x1 = 1;
        slope_x2 = 1;
    }

    if (height < 0) {
        slope_y1 = -1;
    } else if (height > 0) {
        slope_y1 = 1;
    }

    int longest;
    int shortest;

    if (!(abs(width) > abs(height))) {
        longest = abs(height);
        shortest = abs(width);
        if (height < 0) {
            slope_y2 = -1;
        } else if (height > 0) {
            slope_y2 = 1;
        }
        slope_x2 = 0;
    } else {
        longest = abs(width);
        shortest = abs(height);
    }

    int numerator = longest >> 1;

    for (int i=0; i<=longest; i++) {
        Coordinate c;
        c.x = xTracker;
        c.y = yTracker;
        ptsAlongLine.push_back(c);

        numerator += shortest;

        if (!(numerator < longest)) {
            numerator -= longest;
            xTracker += slope_x1;
            yTracker += slope_y1;
        } else {
            xTracker += slope_x2;
            yTracker += slope_y2;
        }
    }

    return ptsAlongLine;
}

int main(int argc, char const *argv[]) {
    cerr << "begin" << endl;
    int x1 = 0;
    int y1 = 0;

    int x2 = 4;
    int y2 = 2;

    std::vector<Coordinate> solutions = findAllPointsAlongLine(x1, y1, x2, y2);
    int i;
    int x;
    int y;
    for(i = 0; i < solutions.size(); i++) {
        x = solutions[i].x;
        y = solutions[i].y;

        std::cout << "coordinate: " << endl;
        std::cout << "x:" << x << endl;
        std::cout << "y:" << y << endl;
    }

    cerr << "end" << endl;

    return 0;
}
