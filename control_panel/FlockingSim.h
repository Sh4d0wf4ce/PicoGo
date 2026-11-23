#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <random>

#if __has_include(<opencv2/geometry.hpp>)
#include <opencv2/geometry.hpp>
#endif

struct FlockingParams {
    // Ruch [cm, s] - odpowiednik flocking_simple.py
    double speedCmS = 19.6;            // 60.0 / 3.06
    double minTurnRadiusCm = 20.0;
    int substeps = 6;

    // Reguły Reynoldsa (Boids)
    double rSepCm = 35.0;
    double rAlignCm = 170.0;
    double rCohCm = 90.0;

    double wKeep = 0.55;
    double wSep = 4.0;
    double wAlign = 12.8;
    double wCoh = 0.5;

    // Gładki szum Ornsteina-Uhlenbecka
    double noiseGain = 0.18;
    double noiseTauS = 1.5;

    // Ściany areny [cm]
    double wallRangeCm = 55.0;
    double wallSafetyCm = 12.0;
    double wallGain = 0.20;
    double wallPower = 3.0;
    double wWall = 8.0;
    double wallLookaheadS = 1.5;
    double wallLookaheadGain = 4.0;

    // Roboty i unikanie kolizji [cm]
    double robotRadiusCm = 5.0;        // średnica 10 cm
    double robotSafetyGapCm = 5.0;     // minimalny odstęp krawędzi
    double collisionRangeCm = 40.0;
    double collisionLookaheadS = 1.3;
    double wCollision = 7.0;
};

struct FlockingBoid {
    int markerId = -1;     // ID ArUco (lub 0..N-1 w trybie podglądu)
    double x = 0.0;        // [cm] w układzie areny (środek w 0,0)
    double y = 0.0;        // [cm]
    double theta = 0.0;    // [rad]
    double noiseX = 0.0;
    double noiseY = 0.0;
};

class FlockingSim {
public:
    FlockingParams params;

    FlockingSim() : rng(std::random_device{}()) {
        // Domyślna arena: 80% kadru, 200x150 cm (do czasu kalibracji 4 rogów)
        cornersNorm = {
            cv::Point2f(0.10f, 0.10f),
            cv::Point2f(0.90f, 0.10f),
            cv::Point2f(0.90f, 0.90f),
            cv::Point2f(0.10f, 0.90f)
        };
        arenaWidthCm = 200.0;
        arenaHeightCm = 150.0;
        rebuildHomography();
    }

    void configureArena(const std::array<cv::Point2f, 4> &corners, double widthCm, double heightCm) {
        if (widthCm <= 1.0 || heightCm <= 1.0) return;
        cornersNorm = corners;
        arenaWidthCm = widthCm;
        arenaHeightCm = heightCm;
        rebuildHomography();
    }

    std::array<cv::Point2f, 4> getCornersNorm() const { return cornersNorm; }
    double getArenaWidthCm() const { return arenaWidthCm; }
    double getArenaHeightCm() const { return arenaHeightCm; }
    const std::vector<FlockingBoid>& getBoids() const { return boids; }

    // Przeliczenie: centymetry areny -> znormalizowane [0,1] obrazu
    cv::Point2d ownToNorm(double xCm, double yCm) const {
        const double *h0 = hArenaToNorm.ptr<double>(0);
        const double *h1 = hArenaToNorm.ptr<double>(1);
        const double *h2 = hArenaToNorm.ptr<double>(2);
        double w = h2[0] * xCm + h2[1] * yCm + h2[2];
        if (std::abs(w) < 1e-12) return {0.5, 0.5};
        double u = (h0[0] * xCm + h0[1] * yCm + h0[2]) / w;
        double v = (h1[0] * xCm + h1[1] * yCm + h1[2]) / w;
        return {std::clamp(u, 0.0, 1.0), std::clamp(v, 0.0, 1.0)};
    }

    // Przeliczenie: znormalizowane [0,1] obrazu -> centymetry areny
    cv::Point2d normToOwn(double u, double v) const {
        const double *h0 = hNormToArena.ptr<double>(0);
        const double *h1 = hNormToArena.ptr<double>(1);
        const double *h2 = hNormToArena.ptr<double>(2);
        double w = h2[0] * u + h2[1] * v + h2[2];
        if (std::abs(w) < 1e-12) return {0.0, 0.0};
        return {
            (h0[0] * u + h0[1] * v + h0[2]) / w,
            (h1[0] * u + h1[1] * v + h1[2]) / w
        };
    }

    // Lokalna transformacja kąta obrazu przez odwrotną homografię
    double imageHeadingToOwnRad(double xNorm, double yNorm, double headingDeg) const {
        double a = headingDeg * kPi / 180.0;
        constexpr double eps = 0.005;
        cv::Point2d p0 = normToOwn(xNorm, yNorm);
        cv::Point2d p1 = normToOwn(xNorm + eps * std::cos(a), yNorm + eps * std::sin(a));
        if (std::hypot(p1.x - p0.x, p1.y - p0.y) < 1e-9) return a;
        return std::atan2(p1.y - p0.y, p1.x - p0.x);
    }

    // Losowy start (tryb podglądu bez robotów)
    void resetRandom(int n) {
        boids.clear();
        if (n <= 0) return;

        double halfW = arenaWidthCm * 0.5;
        double halfH = arenaHeightCm * 0.5;
        double spawnX = std::max(params.robotRadiusCm + 1.0, halfW - params.wallRangeCm - params.robotRadiusCm);
        double spawnY = std::max(params.robotRadiusCm + 1.0, halfH - params.wallRangeCm - params.robotRadiusCm);
        spawnX = std::max(1.0, std::min(spawnX, halfW - params.robotRadiusCm - 1.0));
        spawnY = std::max(1.0, std::min(spawnY, halfH - params.robotRadiusCm - 1.0));

        std::uniform_real_distribution<double> distX(-spawnX, spawnX);
        std::uniform_real_distribution<double> distY(-spawnY, spawnY);
        std::uniform_real_distribution<double> distAng(-kPi, kPi);
        std::normal_distribution<double> gauss(0.0, 1.0);

        double minCenterDist = 2.0 * params.robotRadiusCm + params.robotSafetyGapCm;
        double desiredGap = std::max(minCenterDist, params.rSepCm);

        for (int i = 0; i < n; ++i) {
            double bestX = 0.0, bestY = 0.0, bestDist = -1.0;
            for (int attempt = 0; attempt < 300; ++attempt) {
                double x = distX(rng);
                double y = distY(rng);
                double nearest = 1e9;
                for (const auto &b : boids) {
                    nearest = std::min(nearest, std::hypot(x - b.x, y - b.y));
                }
                if (nearest > bestDist) {
                    bestDist = nearest;
                    bestX = x;
                    bestY = y;
                }
                if (nearest >= desiredGap) break;
            }
            FlockingBoid b;
            b.markerId = i;
            b.x = bestX;
            b.y = bestY;
            b.theta = distAng(rng);
            b.noiseX = gauss(rng);
            b.noiseY = gauss(rng);
            boids.push_back(b);
        }
    }

    // Start z rzeczywistych pozycji robotów: (markerId, xNorm, yNorm, headingDeg)
    void initFromRobots(const std::vector<std::tuple<int, double, double, double>> &robots) {
        boids.clear();
        std::normal_distribution<double> gauss(0.0, 1.0);
        double xmax = std::max(0.0, arenaWidthCm * 0.5 - params.robotRadiusCm);
        double ymax = std::max(0.0, arenaHeightCm * 0.5 - params.robotRadiusCm);

        for (const auto &[mId, xn, yn, headingDeg] : robots) {
            cv::Point2d p = normToOwn(xn, yn);
            FlockingBoid b;
            b.markerId = mId;
            b.x = std::clamp(p.x, -xmax, xmax);
            b.y = std::clamp(p.y, -ymax, ymax);
            b.theta = wrapAngle(imageHeadingToOwnRad(xn, yn, headingDeg));
            b.noiseX = gauss(rng);
            b.noiseY = gauss(rng);
            boids.push_back(b);
        }
    }

    // Jeden krok symulacji (zwraca pary: markerId -> cel w znormalizowanych [0,1])
    std::vector<std::pair<int, cv::Point2d>> step(double dt) {
        dt = std::clamp(dt, 0.0, 0.25);
        std::vector<std::pair<int, cv::Point2d>> out;
        if (boids.empty()) return out;

        int steps = std::max(1, params.substeps);
        double subDt = dt / steps;
        double omegaMax = params.speedCmS / std::max(5.0, params.minTurnRadiusCm);

        std::vector<std::pair<double, double>> desired(boids.size());

        for (int s = 0; s < steps; ++s) {
            updateNoise(subDt);
            for (size_t i = 0; i < boids.size(); ++i) {
                desired[i] = computeForces(i);
            }
            for (size_t i = 0; i < boids.size(); ++i) {
                double targetAng = std::atan2(desired[i].second, desired[i].first);
                double err = wrapAngle(targetAng - boids[i].theta);
                double maxTurn = omegaMax * subDt;
                err = std::clamp(err, -maxTurn, maxTurn);
                boids[i].theta = wrapAngle(boids[i].theta + err);
            }
            for (auto &b : boids) {
                b.x += params.speedCmS * std::cos(b.theta) * subDt;
                b.y += params.speedCmS * std::sin(b.theta) * subDt;
                wallBackstop(b);
            }
            robotBackstop();
        }

        out.reserve(boids.size());
        for (const auto &b : boids) {
            out.emplace_back(b.markerId, ownToNorm(b.x, b.y));
        }
        return out;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    static double wrapAngle(double a) {
        double r = std::fmod(a + kPi, 2.0 * kPi);
        if (r < 0.0) r += 2.0 * kPi;
        return r - kPi;
    }

    static std::pair<double, double> normalizeVec(double x, double y) {
        double n = std::hypot(x, y);
        if (n < 1e-12) return {0.0, 0.0};
        return {x / n, y / n};
    }

    void rebuildHomography() {
        double halfW = arenaWidthCm * 0.5;
        double halfH = arenaHeightCm * 0.5;
        std::vector<cv::Point2f> src = {
            cv::Point2f(static_cast<float>(-halfW), static_cast<float>(-halfH)),
            cv::Point2f(static_cast<float>( halfW), static_cast<float>(-halfH)),
            cv::Point2f(static_cast<float>( halfW), static_cast<float>( halfH)),
            cv::Point2f(static_cast<float>(-halfW), static_cast<float>( halfH))
        };
        std::vector<cv::Point2f> dst(cornersNorm.begin(), cornersNorm.end());
        cv::Mat H = cv::getPerspectiveTransform(src, dst);
        cv::Mat Hinv;
        if (!H.empty() && cv::invert(H, Hinv) > 0.0) {
            hArenaToNorm = H;
            hNormToArena = Hinv;
        } else {
            hArenaToNorm = cv::Mat::eye(3, 3, CV_64F);
            hNormToArena = cv::Mat::eye(3, 3, CV_64F);
        }
    }

    void updateNoise(double dt) {
        if (dt <= 0.0 || params.noiseTauS <= 1e-6) return;
        double a = std::exp(-dt / params.noiseTauS);
        double b = std::sqrt(std::max(0.0, 1.0 - a * a));
        std::normal_distribution<double> gauss(0.0, 1.0);
        for (auto &v : boids) {
            v.noiseX = a * v.noiseX + b * gauss(rng);
            v.noiseY = a * v.noiseY + b * gauss(rng);
        }
    }

    double wallForce(double edgeDistCm) const {
        if (edgeDistCm >= params.wallRangeCm) return 0.0;
        double d = std::max(edgeDistCm - params.wallSafetyCm, 2.0);
        double d0 = std::max(2.0, params.wallRangeCm - params.wallSafetyCm);
        return params.wallGain * (std::pow(d0 / d, params.wallPower) - 1.0);
    }

    std::pair<double, double> computeForces(size_t i) const {
        const auto &v = boids[i];
        double hx = std::cos(v.theta), hy = std::sin(v.theta);

        double sepX = 0.0, sepY = 0.0;
        double aliX = 0.0, aliY = 0.0, aliW = 0.0;
        double cohX = 0.0, cohY = 0.0, cohW = 0.0;
        double colX = 0.0, colY = 0.0;

        double vix = params.speedCmS * hx;
        double viy = params.speedCmS * hy;

        for (size_t j = 0; j < boids.size(); ++j) {
            if (j == i) continue;
            const auto &u = boids[j];
            double dx = u.x - v.x;
            double dy = u.y - v.y;
            double d = std::hypot(dx, dy);
            if (d < 1e-9) continue;

            double nx = dx / d, ny = dy / d;

            if (d < params.rSepCm) {
                double s = (params.rSepCm - d) / params.rSepCm;
                sepX -= s * nx;
                sepY -= s * ny;
            }
            if (d < params.rAlignCm) {
                double w = 1.0 - d / params.rAlignCm;
                aliX += w * std::cos(u.theta);
                aliY += w * std::sin(u.theta);
                aliW += w;
            }
            if (d < params.rCohCm) {
                double w = 1.0 - d / params.rCohCm;
                cohX += w * u.x;
                cohY += w * u.y;
                cohW += w;
            }

            // Predykcyjne unikanie kolizji robot-robot
            double vjx = params.speedCmS * std::cos(u.theta);
            double vjy = params.speedCmS * std::sin(u.theta);
            double rvx = vjx - vix, rvy = vjy - viy;
            double vv = rvx * rvx + rvy * rvy;
            if (vv > 1e-9) {
                double t = std::clamp(-(dx * rvx + dy * rvy) / vv, 0.0, params.collisionLookaheadS);
                double fx = dx + rvx * t;
                double fy = dy + rvy * t;
                double fd = std::hypot(fx, fy);
                if (fd < params.collisionRangeCm) {
                    double ax = (fd > 1e-9) ? (-fx / fd) : (-nx);
                    double ay = (fd > 1e-9) ? (-fy / fd) : (-ny);
                    double s = (params.collisionRangeCm - fd) / params.collisionRangeCm;
                    colX += s * ax;
                    colY += s * ay;
                }
            }
        }

        if (aliW > 1e-12) {
            std::tie(aliX, aliY) = normalizeVec(aliX / aliW, aliY / aliW);
        }
        if (cohW > 1e-12) {
            std::tie(cohX, cohY) = normalizeVec(cohX / cohW - v.x, cohY / cohW - v.y);
        } else {
            cohX = cohY = 0.0;
        }
        std::tie(sepX, sepY) = normalizeVec(sepX, sepY);
        std::tie(colX, colY) = normalizeVec(colX, colY);

        // Odpychanie od ścian + lookahead
        double halfW = arenaWidthCm * 0.5;
        double halfH = arenaHeightCm * 0.5;
        double hardX = halfW - params.robotRadiusCm;
        double hardY = halfH - params.robotRadiusCm;

        double left   = v.x + hardX;
        double right  = hardX - v.x;
        double top    = v.y + hardY;
        double bottom = hardY - v.y;

        double wallX = wallForce(left) - wallForce(right);
        double wallY = wallForce(top)  - wallForce(bottom);

        double futureX = v.x + hx * params.speedCmS * params.wallLookaheadS;
        double futureY = v.y + hy * params.speedCmS * params.wallLookaheadS;
        if (futureX < -hardX + params.wallSafetyCm) wallX += params.wallLookaheadGain;
        if (futureX >  hardX - params.wallSafetyCm) wallX -= params.wallLookaheadGain;
        if (futureY < -hardY + params.wallSafetyCm) wallY += params.wallLookaheadGain;
        if (futureY >  hardY - params.wallSafetyCm) wallY -= params.wallLookaheadGain;

        double desiredX = params.wKeep * hx
                        + params.wSep * sepX
                        + params.wAlign * aliX
                        + params.wCoh * cohX
                        + params.wCollision * colX
                        + params.wWall * wallX
                        + params.noiseGain * v.noiseX;

        double desiredY = params.wKeep * hy
                        + params.wSep * sepY
                        + params.wAlign * aliY
                        + params.wCoh * cohY
                        + params.wCollision * colY
                        + params.wWall * wallY
                        + params.noiseGain * v.noiseY;

        if (std::hypot(desiredX, desiredY) < 1e-12) return {hx, hy};
        return normalizeVec(desiredX, desiredY);
    }

    void wallBackstop(FlockingBoid &v) const {
        double xmax = std::max(0.0, arenaWidthCm * 0.5 - params.robotRadiusCm);
        double ymax = std::max(0.0, arenaHeightCm * 0.5 - params.robotRadiusCm);
        bool hitX = false, hitY = false;
        if (v.x < -xmax) { v.x = -xmax; hitX = true; }
        else if (v.x > xmax) { v.x = xmax; hitX = true; }
        if (v.y < -ymax) { v.y = -ymax; hitY = true; }
        else if (v.y > ymax) { v.y = ymax; hitY = true; }
        if (hitX) v.theta = kPi - v.theta;
        if (hitY) v.theta = -v.theta;
        v.theta = wrapAngle(v.theta);
    }

    void robotBackstop() {
        double minCenterDist = 2.0 * params.robotRadiusCm + params.robotSafetyGapCm;
        for (int iter = 0; iter < 8; ++iter) {
            bool changed = false;
            for (size_t i = 0; i < boids.size(); ++i) {
                for (size_t j = i + 1; j < boids.size(); ++j) {
                    double dx = boids[j].x - boids[i].x;
                    double dy = boids[j].y - boids[i].y;
                    double d = std::hypot(dx, dy);
                    if (d >= minCenterDist) continue;
                    changed = true;
                    double nx = (d < 1e-9) ? 1.0 : (dx / d);
                    double ny = (d < 1e-9) ? 0.0 : (dy / d);
                    double shift = 0.5 * (minCenterDist - d);
                    boids[i].x -= shift * nx;
                    boids[i].y -= shift * ny;
                    boids[j].x += shift * nx;
                    boids[j].y += shift * ny;
                    wallBackstop(boids[i]);
                    wallBackstop(boids[j]);
                }
            }
            if (!changed) break;
        }
    }

    std::array<cv::Point2f, 4> cornersNorm;
    double arenaWidthCm = 200.0;
    double arenaHeightCm = 150.0;
    cv::Mat hArenaToNorm;
    cv::Mat hNormToArena;
    std::vector<FlockingBoid> boids;
    std::mt19937 rng;
};