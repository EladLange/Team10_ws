// Full C++ main program with tangent vector export (dx, dy, dz)

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

using Vec3 = array<double, 3>;
using Matrix4x3 = array<Vec3, 4>;
using PointList = vector<Vec3>;

// Cubic B-spline basis matrix (4x4)
const double M[4][4] = {
    {-1.0 / 6,  0.5,  -0.5, 1.0 / 6},
    { 0.5,   -1.0,  0.5, 0.0},
    {-0.5,    0.0,  0.5, 0.0},
    { 1.0 / 6,  2.0 / 3, 1.0 / 6, 0.0}
};

/**
 * Computes a point on a cubic B-spline curve at parameter u.
 * param u            Parameter in [0, 1] along the segment.
 * param controlPoints 4 control points defining the segment (4x3 matrix).
 * return A 3D point on the curve.
 */
Vec3 computeBSplinePoint(double u, const Matrix4x3& controlPoints) {
    double U[4] = { pow(u,3), pow(u,2), u, 1.0 };
    Vec3 result = { 0, 0, 0 };
    for (int d = 0; d < 3; ++d) {
        for (int i = 0; i < 4; ++i) {
            double coeff = 0.0;
            for (int j = 0; j < 4; ++j)
                coeff += U[j] * M[j][i];
            result[d] += coeff * controlPoints[i][d];
        }
    }
    return result;
}

/**
 * Computes the first derivative (tangent vector) of a cubic B-spline segment.
 * param u            Parameter in [0, 1] along the segment.
 * param controlPoints 4 control points defining the segment (4x3 matrix).
 * return A 3D tangent vector.
 */
Vec3 computeBSplineTangent(double u, const Matrix4x3& controlPoints) {
    double dU[4] = { 3 * pow(u,2), 2 * u, 1.0, 0.0 };
    Vec3 result = { 0, 0, 0 };
    for (int d = 0; d < 3; ++d) {
        for (int i = 0; i < 4; ++i) {
            double coeff = 0.0;
            for (int j = 0; j < 4; ++j)
                coeff += dU[j] * M[j][i];
            result[d] += coeff * controlPoints[i][d];
        }
    }
    return result;
}

/**
 * Computes the Euclidean norm (magnitude) of a 3D vector.
 * param v A 3D vector.
 * return The magnitude (length) of the vector.
 */
double norm(const Vec3& v) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * Loads control points from a CSV file.
 * Expected format: header followed by rows with X,Y[,Z] values.
 * If Z is missing, it is assumed to be 0 (2D).
 * param filename Path to the CSV file.
 * return A list of 3D control points.
 */
PointList loadControlPointsFromCSV(const string& filename) {
    PointList points;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        exit(1);
    }
    string line;
    bool headerSkipped = false;
    while (getline(file, line)) {
        if (!headerSkipped) { headerSkipped = true; continue; }
        if (line.empty()) continue;
        istringstream ss(line);
        string val;
        Vec3 point = { 0, 0, 0 };
        try {
            getline(ss, val, ','); point[0] = stod(val);
            getline(ss, val, ','); point[1] = stod(val);
            if (getline(ss, val, ',')) point[2] = val.empty() ? 0.0 : stod(val);
            else point[2] = 0.0;
            points.push_back(point);
        }
        catch (const std::exception& e) {
            cerr << "Warning: Skipping invalid line: " << line << endl;
            continue;
        }
    }
    file.close();
    return points;
}

/**
 * Main execution function: loads control points, computes B-spline curve and tangents,
 * and exports the results to a CSV file.
 */
int main() {
    PointList originalCP = loadControlPointsFromCSV("YM_controlPoints.csv");

    // Add clamping (repeat endpoints 3 times)
    PointList controlPoints;
    controlPoints.push_back(originalCP[1]);
    controlPoints.push_back(originalCP[1]);
    controlPoints.push_back(originalCP[1]);
    controlPoints.insert(controlPoints.end(), originalCP.begin(), originalCP.end());
    controlPoints.push_back(originalCP.back());
    controlPoints.push_back(originalCP.back());
    controlPoints.push_back(originalCP.back());

    int resolution = 100;
    vector<Vec3> curve;
    vector<Vec3> tangents;
    vector<double> tangentMagnitudes;

    for (size_t k = 1; k + 3 < controlPoints.size(); ++k) {
        Matrix4x3 segment;
        for (int i = 0; i < 4; ++i)
            segment[i] = controlPoints[k + i];

        for (int r = 0; r <= resolution; ++r) {
            double u = double(r) / resolution;
            Vec3 pt = computeBSplinePoint(u, segment);
            Vec3 tg = computeBSplineTangent(u, segment);
            curve.push_back(pt);
            tangents.push_back(tg);
            tangentMagnitudes.push_back(norm(tg));
        }
    }

    // Output to CSV with full tangent vectors
    ofstream fout("bspline_curve.csv");
    fout << "X,Y,Z,dX,dY,dZ,Tangent\n";
    for (size_t i = 0; i < curve.size(); ++i) {
        fout << curve[i][0] << "," << curve[i][1] << "," << curve[i][2] << ","
            << tangents[i][0] << "," << tangents[i][1] << "," << tangents[i][2] << ","
            << tangentMagnitudes[i] << "\n";
    }
    fout.close();

    cout << "Curve computed and saved to bspline_curve.csv" << endl;
    return 0;
}
