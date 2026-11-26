#pragma once

#include <opencv2/opencv.hpp>
#if __has_include(<opencv2/geometry.hpp>)
#include <opencv2/geometry.hpp>
#endif

#include <array>
#include <vector>
#include <tuple>
#include <string>
#include <cmath>
#include <algorithm>
#include <random>
#include <limits>

struct FlockingParams {
    // Ruch [cm, s] - port 1:1 z flocking_2.py (Stado - model A)
    double speedCmS = 60.0 / 3.06;          // ~19.61 cm/s
    double minTurnRadiusCm = 36.0;
    double maxSteerTargetAngleDeg = 38.0;
    double turnResponseTauS = 0.70;
    double angularAccelMaxRadS2 = 0.60;

    // Zmienna prędkość (robot nigdy się nie zatrzymuje, min. 30% V_max)
    double speedMinFraction = 0.45;
    double maxLeadCm = 14.0;
    double linearAccelCmS2 = 10.0;
    double linearDecelCmS2 = 24.0;
    double minPathRadiusCm = 12.0;
    double wallStopMarginCm = 6.0;

    // Omijanie robot-robot po łukach (TTC)
    double robotGapStopCm = 17.0;
    double robotTtcStopS = 0.2;
    double robotTtcSlowS = 0.8;
    int robotPredictSteps = 12;
    double escapeTtcS = 2.0;
    double robotGapTowardCos = 0.2;
    double robotFollowGain = 0.0;
    double escapeGain = 1.5;

    // Ucieczka od ściany
    double wallEscapeEarly = 1.4;
    double wallEscapeInward = 0.25;
    double wallEscapeCornerCm = 36.0;

    int substeps = 6;

    // Reguły Boids (Alignment jako łagodna korekta kursu)
    double rSepCm = 35.0;
    double rAlignCm = 70.0;
    double rCohCm = 90.0;

    double wKeep = 1.0;
    double wSep = 4.0;
    double wAlign = 49.2;
    double alignSelfWeight = 1.8;
    double alignMaxCorrection = 0.45;
    double wCoh = 0.5;

    // Szum Ornsteina-Uhlenbecka
    double noiseGain = 0.18;
    double noiseTauS = 1.5;

    // Ściany (smoothstep) i narożniki z pamięcią
    double wallClearanceCm = 0.0;
    double wallRangeCm = 75.0;
    double softWallPenetrationCm = 3.0;
    double hardWallPenetrationCm = 6.0;
    double wallForceMax = 1.15;
    double wWall = 4.5;
    double wallLookaheadS = 2.1;
    double wallLookaheadGain = 1.15;

    double cornerLookaheadS = 4.5;
    double cornerReleaseFactor = 1.5;
    double cornerMaxHoldS = 6.0;
    double cornerCooldownS = 2.0;
    double cornerProbeS = 3.0;
    int cornerProbeSteps = 24;

    // Roboty / kolizje
    double robotRadiusCm = 5.0;             // średnica 10 cm
    double robotSafetyGapCm = 5.0;          // MIN_CENTER_DISTANCE = 15 cm
    double backstopGapCm = 2.0;             // BACKSTOP_CENTER_DISTANCE = 12 cm
    double collisionRangeCm = 65.0;
    double collisionLookaheadS = 4.5;
    double wCollision = 6.0;
    double collisionRightBias = 0.65;
    double separationPower = 1.6;
    double sepCloseStartCm = 28.0;
    double wSepClose = 2.5;
    double headOnDeg = 15.0;

    double spawnFraction = 0.75;
    double spawnMinSpacingCm = 50.0;
};

struct FlockingBoid {
    int markerId = -1;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double omega = 0.0;
    double speed = 0.0;
    double noiseX = 0.0;
    double noiseY = 0.0;
    int cornerTurnSign = 0;
    double cornerHoldS = 0.0;
    double cornerCooldownS = 0.0;
};

struct ArenaOverlayPoly {
    std::vector<cv::Point2d> pointsNorm;
    cv::Scalar colorBgr;
    int thickness = 1;
    std::string label;
};

class FlockingSim {
public:
    FlockingParams params;

    FlockingSim() : rng(std::random_device{}()) {
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

    bool configureArena(const std::array<cv::Point2f, 4> &corners, double widthCm, double heightCm,
                        std::string *errMsg = nullptr) {
        if (widthCm <= 1.0 || heightCm <= 1.0) {
            if (errMsg) *errMsg = "Szerokosc i wysokosc areny musza byc dodatnie.";
            return false;
        }
        for (const auto &c : corners) {
            if (c.x < 0.0f || c.x > 1.0f || c.y < 0.0f || c.y > 1.0f) {
                if (errMsg) *errMsg = "Rogi areny musza lezec w zakresie [0,1].";
                return false;
            }
        }
        // Test wypukłości i kolejności CW (LG -> PG -> PD -> LD) z flocking_2.py
        for (int k = 0; k < 4; ++k) {
            const auto &a = corners[k];
            const auto &b = corners[(k + 1) % 4];
            const auto &c = corners[(k + 2) % 4];
            double cross = double(b.x - a.x) * double(c.y - b.y) - double(b.y - a.y) * double(c.x - b.x);
            if (cross <= 1e-9) {
                if (errMsg) {
                    *errMsg = "Rogi w zlej kolejnosci lub czworokat niewypukly (wymagane: LG -> PG -> PD -> LD).";
                }
                return false;
            }
        }

        auto prevCorners = cornersNorm;
        double prevW = arenaWidthCm, prevH = arenaHeightCm;
        cornersNorm = corners;
        arenaWidthCm = widthCm;
        arenaHeightCm = heightCm;
        if (!rebuildHomography()) {
            cornersNorm = prevCorners;
            arenaWidthCm = prevW;
            arenaHeightCm = prevH;
            rebuildHomography();
            if (errMsg) *errMsg = "Nie udalo sie wyznaczyc odwrotnej homografii areny.";
            return false;
        }
        return true;
    }

    std::array<cv::Point2f, 4> getCornersNorm() const { return cornersNorm; }
    double getArenaWidthCm() const { return arenaWidthCm; }
    double getArenaHeightCm() const { return arenaHeightCm; }
    const std::vector<FlockingBoid>& getBoids() const { return boids; }

    // Średnia skala px/cm z 4 boków skalibrowanej areny (z swarm_controller_2.py)
    double getArenaPxPerCm(int frameW, int frameH) const {
        if (frameW <= 0 || frameH <= 0 || arenaWidthCm <= 1.0 || arenaHeightCm <= 1.0) return 0.0;
        auto pt = [&](int idx) {
            return cv::Point2d(cornersNorm[idx].x * frameW, cornersNorm[idx].y * frameH);
        };
        auto dist = [](const cv::Point2d &a, const cv::Point2d &b) {
            return std::hypot(a.x - b.x, a.y - b.y);
        };
        double r0 = dist(pt(0), pt(1)) / arenaWidthCm;
        double r1 = dist(pt(3), pt(2)) / arenaWidthCm;
        double r2 = dist(pt(0), pt(3)) / arenaHeightCm;
        double r3 = dist(pt(1), pt(2)) / arenaHeightCm;
        return 0.25 * (r0 + r1 + r2 + r3);
    }

    // Ramki strefy bezpieczeństwa (odpowiednik overlay_polygons() z flocking_2.py)
    std::vector<ArenaOverlayPoly> getOverlayPolygons() const {
        auto makeRect = [&](double hw, double hh) {
            return std::vector<cv::Point2d>{
                ownToNorm(-hw, -hh), ownToNorm(hw, -hh), ownToNorm(hw, hh), ownToNorm(-hw, hh)
            };
        };
        std::vector<ArenaOverlayPoly> out;
        out.push_back({
            makeRect(innerW() + params.hardWallPenetrationCm, innerH() + params.hardWallPenetrationCm),
            cv::Scalar(160, 160, 160), 1, "granica srodka"
        });
        double hw = innerW() - params.wallRangeCm;
        double hh = innerH() - params.wallRangeCm;
        if (hw > 0.0 && hh > 0.0) {
            out.push_back({
                makeRect(hw, hh),
                cv::Scalar(0, 165, 255), 2, "strefa scian"
            });
        }
        return out;
    }

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

    double imageHeadingToOwnRad(double xNorm, double yNorm, double headingDeg) const {
        double a = headingDeg * kPi / 180.0;
        constexpr double eps = 0.005;
        cv::Point2d p0 = normToOwn(xNorm, yNorm);
        cv::Point2d p1 = normToOwn(xNorm + eps * std::cos(a), yNorm + eps * std::sin(a));
        if (std::hypot(p1.x - p0.x, p1.y - p0.y) < 1e-9) return a;
        return std::atan2(p1.y - p0.y, p1.x - p0.x);
    }

    void resetRandom(int n) {
        boids.clear();
        if (n <= 0) return;

        double spawnX = std::max(1.0, params.spawnFraction * innerW());
        double spawnY = std::max(1.0, params.spawnFraction * innerH());

        std::uniform_real_distribution<double> distX(-spawnX, spawnX);
        std::uniform_real_distribution<double> distY(-spawnY, spawnY);
        std::uniform_real_distribution<double> distAng(-kPi, kPi);
        std::normal_distribution<double> gauss(0.0, 1.0);

        double desiredGap = std::max(minCenterDistanceCm(), params.spawnMinSpacingCm);

        for (int i = 0; i < n; ++i) {
            double bestX = 0.0, bestY = 0.0, bestDist = -1.0;
            for (int attempt = 0; attempt < 400; ++attempt) {
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
            b.omega = 0.0;
            b.speed = speedMinCmS();
            b.noiseX = gauss(rng);
            b.noiseY = gauss(rng);
            boids.push_back(b);
        }
    }

    void initFromRobots(const std::vector<std::tuple<int, double, double, double>> &robots) {
        boids.clear();
        std::normal_distribution<double> gauss(0.0, 1.0);
        double xmax = innerW() + params.hardWallPenetrationCm;
        double ymax = innerH() + params.hardWallPenetrationCm;

        for (const auto &[mId, xn, yn, headingDeg] : robots) {
            cv::Point2d p = normToOwn(xn, yn);
            FlockingBoid b;
            b.markerId = mId;
            b.x = std::clamp(p.x, -xmax, xmax);
            b.y = std::clamp(p.y, -ymax, ymax);
            b.theta = wrapAngle(imageHeadingToOwnRad(xn, yn, headingDeg));
            b.omega = 0.0;
            b.speed = speedMinCmS();
            b.noiseX = gauss(rng);
            b.noiseY = gauss(rng);
            boids.push_back(b);
        }
    }

    std::vector<std::pair<int, cv::Point2d>> step(
        double dt,
        const std::map<int, cv::Point2d> *realPosNorm = nullptr) {
        dt = std::clamp(dt, 0.0, 0.25);
        std::vector<std::pair<int, cv::Point2d>> out;
        if (boids.empty()) return out;

        // Przeliczenie aktualnych pozycji fizycznych robotów na układ areny [cm]
        std::map<int, cv::Point2d> realOwn;
        if (realPosNorm != nullptr) {
            for (const auto &[mId, ptNorm] : *realPosNorm) {
                realOwn[mId] = normToOwn(ptNorm.x, ptNorm.y);
            }
        }

        const size_t n = boids.size();
        const int steps = std::max(1, params.substeps);
        const double subDt = dt / steps;

        std::vector<std::pair<double, double>> desired(n);
        PairwiseCache pw;

        for (int s = 0; s < steps; ++s) {
            updateNoise(subDt);
            computePairwise(pw);

            for (size_t i = 0; i < n; ++i) {
                desired[i] = computeForces(i, pw);
            }

            // 1. Zmienna prędkość + elastyczna smycz (kropka nie ucieka od fizycznego robota)
            const double vMin = speedMinCmS();
            const double maxLead = std::max(5.0, params.maxLeadCm);
            const double softLead = maxLead * 0.50;

            for (size_t i = 0; i < n; ++i) {
                auto &v = boids[i];
                double lagScale = 1.0;

                if (realPosNorm != nullptr) {
                    auto it = realOwn.find(v.markerId);
                    if (it == realOwn.end()) {
                        // Robot chwilowo zgubiony -> jego wirtualny boid staje, inne go omijają
                        lagScale = 0.0;
                    } else {
                        double lag = std::hypot(v.x - it->second.x, v.y - it->second.y);
                        if (lag > softLead) {
                            lagScale = std::clamp((maxLead - lag) / std::max(1.0, maxLead - softLead), 0.0, 1.0);
                        }
                        // Awaryjne dociągnięcie tylko przy dużym poślizgu/przestawieniu robota ręką
                        if (lag > maxLead * 2.2) {
                            v.x += 0.08 * (it->second.x - v.x);
                            v.y += 0.08 * (it->second.y - v.y);
                            physicalWallBackstop(v);
                        }
                    }
                }

                double vWall = effectiveWallSpeedLimit(v);
                double vRob = pw.robotSpeedLimit[i];
                double vBase = std::max(vMin, std::min({params.speedCmS, vWall, vRob}));
                double vTarget = vBase * lagScale;

                double decel = (lagScale < 0.5) ? (params.linearDecelCmS2 * 2.0) : params.linearDecelCmS2;
                double dv = std::clamp(vTarget - v.speed, -decel * subDt, params.linearAccelCmS2 * subDt);
                double effectiveMin = (lagScale < 0.99) ? 0.0 : vMin;
                v.speed = std::clamp(v.speed + dv, effectiveMin, params.speedCmS);
            }

            // 2. Fizycznie gładki skręt z ograniczonym przyspieszeniem kątowym
            const double maxDomega = params.angularAccelMaxRadS2 * subDt;
            for (size_t i = 0; i < n; ++i) {
                auto &v = boids[i];
                double target = std::atan2(desired[i].second, desired[i].first);
                double err = wrapAngle(target - v.theta);
                double wLim = omegaLimit(v.speed);
                double omegaTarget = std::clamp(err / std::max(1e-6, params.turnResponseTauS), -wLim, wLim);
                double domega = std::clamp(omegaTarget - v.omega, -maxDomega, maxDomega);
                v.omega = std::clamp(v.omega + domega, -wLim, wLim);
                v.theta = wrapAngle(v.theta + v.omega * subDt);
            }

            for (auto &v : boids) {
                if (v.cornerTurnSign != 0) v.cornerHoldS += subDt;
                if (v.cornerCooldownS > 0.0) v.cornerCooldownS = std::max(0.0, v.cornerCooldownS - subDt);
            }

            // 3. Integracja położenia + awaryjne przycięcie bez skoku kąta
            for (auto &v : boids) {
                v.x += v.speed * std::cos(v.theta) * subDt;
                v.y += v.speed * std::sin(v.theta) * subDt;
                physicalWallBackstop(v);
            }
            robotBackstop();
        }

        out.reserve(n);
        for (const auto &v : boids) {
            out.emplace_back(v.markerId, ownToNorm(v.x, v.y));
        }
        return out;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    struct PairwiseCache {
        std::vector<double> robotSpeedLimit;
        std::vector<double> escX, escY, escapeUrgency;
        std::vector<double> sepX, sepY;
        std::vector<double> aliX, aliY, aliW;
        std::vector<double> cohX, cohY, cohW;
        std::vector<double> colX, colY;
    };

    double innerW() const { return std::max(0.0, arenaWidthCm * 0.5 - params.robotRadiusCm - params.wallClearanceCm); }
    double innerH() const { return std::max(0.0, arenaHeightCm * 0.5 - params.robotRadiusCm - params.wallClearanceCm); }
    double speedMinCmS() const { return params.speedMinFraction * params.speedCmS; }
    double omegaMaxRadS() const { return params.speedCmS / std::max(5.0, params.minTurnRadiusCm); }
    double minCenterDistanceCm() const { return 2.0 * params.robotRadiusCm + params.robotSafetyGapCm; }
    double backstopCenterDistanceCm() const { return 2.0 * params.robotRadiusCm + params.backstopGapCm; }

    double omegaLimit(double speed) const {
        return std::min(omegaMaxRadS(), std::max(0.0, speed) / std::max(2.0, params.minPathRadiusCm));
    }

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

    static double smoothstep(double q) {
        q = std::clamp(q, 0.0, 1.0);
        return q * q * (3.0 - 2.0 * q);
    }

    bool rebuildHomography() {
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
        if (!H.empty() && cv::invert(H, Hinv) > 1e-9) {
            hArenaToNorm = H;
            hNormToArena = Hinv;
            return true;
        }
        hArenaToNorm = cv::Mat::eye(3, 3, CV_64F);
        hNormToArena = cv::Mat::eye(3, 3, CV_64F);
        return false;
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
        double span = std::max(1e-9, params.wallRangeCm + params.softWallPenetrationCm);
        double q = (params.wallRangeCm - edgeDistCm) / span;
        return params.wallForceMax * smoothstep(q);
    }

    double clearanceToArena(double x, double y) const {
        double xmax = innerW() + params.hardWallPenetrationCm;
        double ymax = innerH() + params.hardWallPenetrationCm;
        return std::min({x + xmax, xmax - x, y + ymax, ymax - y});
    }

    double scoreTurnArc(const FlockingBoid &v, int sign) const {
        double spd = std::max(v.speed, params.speedCmS);
        double omegaTarget = double(sign) * omegaLimit(spd);
        int steps = std::max(1, params.cornerProbeSteps);
        double dt = params.cornerProbeS / steps;
        double maxDomega = params.angularAccelMaxRadS2 * dt;
        double x = v.x, y = v.y, th = v.theta, om = v.omega;
        double minClr = clearanceToArena(x, y);

        for (int k = 0; k < steps; ++k) {
            om += std::clamp(omegaTarget - om, -maxDomega, maxDomega);
            th = wrapAngle(th + om * dt);
            x += spd * std::cos(th) * dt;
            y += spd * std::sin(th) * dt;
            minClr = std::min(minClr, clearanceToArena(x, y));
        }
        return minClr;
    }

    int cornerAvoidanceTurn(FlockingBoid &v) const {
        double hx = std::cos(v.theta), hy = std::sin(v.theta);
        double left = v.x + innerW(), right = innerW() - v.x;
        double top = v.y + innerH(), bottom = innerH() - v.y;
        const double inf = std::numeric_limits<double>::infinity();

        double tx = inf, ty = inf;
        if (hx > 1e-6)       tx = right / (params.speedCmS * hx);
        else if (hx < -1e-6) tx = left  / (params.speedCmS * (-hx));
        if (hy > 1e-6)       ty = bottom / (params.speedCmS * hy);
        else if (hy < -1e-6) ty = top    / (params.speedCmS * (-hy));

        bool danger = (tx < params.cornerLookaheadS && ty < params.cornerLookaheadS);

        if (v.cornerTurnSign != 0) {
            double horizon = params.cornerReleaseFactor * params.cornerLookaheadS;
            if (tx < horizon && ty < horizon && v.cornerHoldS < params.cornerMaxHoldS) {
                return v.cornerTurnSign;
            }
            if (v.cornerHoldS >= params.cornerMaxHoldS) {
                v.cornerCooldownS = params.cornerCooldownS;
            }
            v.cornerTurnSign = 0;
            v.cornerHoldS = 0.0;
        }

        if (v.cornerCooldownS > 0.0 || !danger) return 0;

        double scoreL = scoreTurnArc(v, -1);
        double scoreR = scoreTurnArc(v, +1);
        int sign = (std::abs(scoreL - scoreR) < 1e-6) ? +1 : ((scoreL > scoreR) ? -1 : +1);
        v.cornerTurnSign = sign;
        return sign;
    }

    double turnRoomNeeded(double speed, double outward, double omegaNowToward, double currentSpeed) const {
        double tangent = std::sqrt(std::max(0.0, 1.0 - outward * outward));
        double wLim = std::max(1e-6, omegaLimit(speed));
        double radius = speed / wLim;
        double rampS = std::max(0.0, wLim - std::max(0.0, omegaNowToward)) / std::max(1e-6, params.angularAccelMaxRadS2);
        double ramp = speed * outward * 0.5 * rampS;
        double v0 = std::max(speed, currentSpeed);
        double brake = outward * (v0 * v0 - speed * speed) / std::max(1e-6, 2.0 * params.linearDecelCmS2);
        return radius * (1.0 - tangent) + ramp + brake;
    }

    double wallSpeedLimit(const FlockingBoid &v) const {
        double hx = std::cos(v.theta), hy = std::sin(v.theta);
        double xmax = innerW() + params.hardWallPenetrationCm - params.wallStopMarginCm;
        double ymax = innerH() + params.hardWallPenetrationCm - params.wallStopMarginCm;
        double limit = params.speedCmS;

        struct WallInfo { double room, outward, nx, ny; };
        WallInfo walls[4] = {
            {xmax - v.x,  hx,  1.0,  0.0},
            {v.x + xmax, -hx, -1.0,  0.0},
            {ymax - v.y,  hy,  0.0,  1.0},
            {v.y + ymax, -hy,  0.0, -1.0}
        };

        for (const auto &w : walls) {
            if (w.outward <= 1e-6) continue;
            double cross = hx * w.ny - hy * w.nx;
            double toward = (cross > 0.0) ? -v.omega : v.omega;
            if (turnRoomNeeded(params.speedCmS, w.outward, toward, v.speed) <= w.room) continue;
            double lo = 0.0, hi = params.speedCmS;
            for (int it = 0; it < 14; ++it) {
                double mid = 0.5 * (lo + hi);
                if (turnRoomNeeded(mid, w.outward, toward, v.speed) <= w.room) lo = mid;
                else hi = mid;
            }
            limit = std::min(limit, lo);
        }
        return limit;
    }

    double effectiveWallSpeedLimit(const FlockingBoid &v) const {
        double vWall = wallSpeedLimit(v);
        if (vWall < params.speedCmS) {
            double bestArc = (v.cornerTurnSign != 0)
                                 ? scoreTurnArc(v, v.cornerTurnSign)
                                 : std::max(scoreTurnArc(v, -1), scoreTurnArc(v, +1));
            if (bestArc >= params.wallStopMarginCm) vWall = params.speedCmS;
        }
        return vWall;
    }

    std::tuple<double, double, double> wallEscape(const FlockingBoid &v) const {
        double hx = std::cos(v.theta), hy = std::sin(v.theta);
        double xmax = innerW() + params.hardWallPenetrationCm - params.wallStopMarginCm;
        double ymax = innerH() + params.hardWallPenetrationCm - params.wallStopMarginCm;

        double bestU = 0.0, bestTx = hx, bestTy = hy;
        struct WallInfo { double room, outward, nx, ny; };
        WallInfo walls[4] = {
            {xmax - v.x,  hx,  1.0,  0.0},
            {v.x + xmax, -hx, -1.0,  0.0},
            {ymax - v.y,  hy,  0.0,  1.0},
            {v.y + ymax, -hy,  0.0, -1.0}
        };

        for (const auto &w : walls) {
            if (w.outward <= 1e-6) continue;
            double start = params.wallEscapeEarly * turnRoomNeeded(params.speedCmS, w.outward, 0.0, params.speedCmS);
            double crit  = turnRoomNeeded(speedMinCmS(), w.outward, 0.0, v.speed);
            if (w.room >= start) continue;
            double u = smoothstep((start - w.room) / std::max(1e-6, start - crit));
            if (u <= bestU) continue;

            double px = -w.ny, py = w.nx;
            double along = hx * px + hy * py;
            double sgn = (along > 1e-3) ? 1.0 : ((along < -1e-3) ? -1.0 : ((v.omega >= 0.0) ? 1.0 : -1.0));

            auto roomAlong = [&](double sg) {
                double dxs = sg * px, dys = sg * py;
                if (std::abs(dxs) > 0.5) return (dxs > 0.0) ? (xmax - v.x) : (v.x + xmax);
                return (dys > 0.0) ? (ymax - v.y) : (v.y + ymax);
            };
            if (roomAlong(sgn) < params.wallEscapeCornerCm && roomAlong(-sgn) > roomAlong(sgn)) {
                sgn = -sgn;
            }
            auto [tx, ty] = normalizeVec(sgn * px - params.wallEscapeInward * w.nx,
                                         sgn * py - params.wallEscapeInward * w.ny);
            bestU = u;
            bestTx = tx;
            bestTy = ty;
        }
        return {bestU, bestTx, bestTy};
    }

    void computePairwise(PairwiseCache &pw) const {
        const size_t n = boids.size();
        pw.robotSpeedLimit.assign(n, params.speedCmS);
        pw.escX.assign(n, 0.0); pw.escY.assign(n, 0.0); pw.escapeUrgency.assign(n, 0.0);
        pw.sepX.assign(n, 0.0); pw.sepY.assign(n, 0.0);
        pw.aliX.assign(n, 0.0); pw.aliY.assign(n, 0.0); pw.aliW.assign(n, 0.0);
        pw.cohX.assign(n, 0.0); pw.cohY.assign(n, 0.0); pw.cohW.assign(n, 0.0);
        pw.colX.assign(n, 0.0); pw.colY.assign(n, 0.0);
        if (n < 2) return;

        const double headOnSin = std::sin(params.headOnDeg * kPi / 180.0);
        const double minCenter = minCenterDistanceCm();
        const int predSteps = std::max(1, params.robotPredictSteps);
        const double horizon = std::max(params.robotTtcSlowS, params.escapeTtcS);
        const double inf = std::numeric_limits<double>::infinity();

        // Prekalkulacja przewidywanych pozycji po aktualnych łukach (v, omega)
        std::vector<double> hx(n), hy(n), vx(n), vy(n);
        for (size_t i = 0; i < n; ++i) {
            hx[i] = std::cos(boids[i].theta);
            hy[i] = std::sin(boids[i].theta);
            vx[i] = boids[i].speed * hx[i];
            vy[i] = boids[i].speed * hy[i];
        }

        std::vector<std::vector<double>> predX(predSteps, std::vector<double>(n));
        std::vector<std::vector<double>> predY(predSteps, std::vector<double>(n));
        for (int k = 0; k < predSteps; ++k) {
            double t = horizon * double(k + 1) / predSteps;
            for (size_t i = 0; i < n; ++i) {
                double om = boids[i].omega;
                if (std::abs(om) < 1e-6) {
                    predX[k][i] = boids[i].x + vx[i] * t;
                    predY[k][i] = boids[i].y + vy[i] * t;
                } else {
                    double thT = boids[i].theta + om * t;
                    predX[k][i] = boids[i].x + (boids[i].speed / om) * (std::sin(thT) - std::sin(boids[i].theta));
                    predY[k][i] = boids[i].y - (boids[i].speed / om) * (std::cos(thT) - std::cos(boids[i].theta));
                }
            }
        }

        auto lateralAway = [&](double px, double py, double rx, double ry) -> std::pair<double, double> {
            double pn = std::hypot(px, py);
            double side = (pn > 1e-9) ? ((px * rx + py * ry) / pn) : 0.0;
            double sgn = (std::abs(side) < headOnSin) ? 1.0 : ((side > 0.0) ? -1.0 : 1.0);
            return {sgn * rx, sgn * ry};
        };

        for (size_t i = 0; i < n; ++i) {
            const double rightX = -hy[i];
            const double rightY =  hx[i];

            for (size_t j = 0; j < n; ++j) {
                if (i == j) continue;
                double dx = boids[j].x - boids[i].x;
                double dy = boids[j].y - boids[i].y;
                double d = std::hypot(dx, dy);
                if (d < 1e-9) continue;

                double nx = dx / d, ny = dy / d;
                double rvx = vx[j] - vx[i], rvy = vy[j] - vy[i];
                double rv2 = rvx * rvx + rvy * rvy;

                double tCa = (rv2 > 1e-9) ? std::clamp(-(dx * rvx + dy * rvy) / rv2, 0.0, params.escapeTtcS) : 0.0;
                auto [latX, latY] = lateralAway(dx + rvx * tCa, dy + rvy * tCa, rightX, rightY);

                // --- Separation ---
                double s = 0.0;
                if (d < params.rSepCm) {
                    double q = std::clamp((params.rSepCm - d) / params.rSepCm, 0.0, 1.0);
                    s = std::pow(q, params.separationPower);
                }
                double qClose = std::clamp((params.sepCloseStartCm - d) / std::max(1e-6, params.sepCloseStartCm - minCenter), 0.0, 1.0);
                s += (params.wSepClose / std::max(params.wSep, 1e-9)) * qClose * qClose;

                if (s > 0.0) {
                    bool inFront = (nx * hx[i] + ny * hy[i]) > 0.0;
                    if (inFront) {
                        auto [fsx, fsy] = normalizeVec(0.68 * latX - 0.32 * nx, 0.68 * latY - 0.32 * ny);
                        pw.sepX[i] += s * fsx;
                        pw.sepY[i] += s * fsy;
                    } else {
                        pw.sepX[i] -= s * nx;
                        pw.sepY[i] -= s * ny;
                    }
                }

                // --- Alignment ---
                if (d < params.rAlignCm) {
                    double wa = 1.0 - d / params.rAlignCm;
                    pw.aliX[i] += wa * hx[j];
                    pw.aliY[i] += wa * hy[j];
                    pw.aliW[i] += wa;
                }

                // --- Cohesion ---
                if (d < params.rCohCm) {
                    double wc = 1.0 - d / params.rCohCm;
                    pw.cohX[i] += wc * boids[j].x;
                    pw.cohY[i] += wc * boids[j].y;
                    pw.cohW[i] += wc;
                }

                // --- Predykcja kolizji liniowa ---
                if (rv2 > 1e-9) {
                    double rawT = -(dx * rvx + dy * rvy) / rv2;
                    if (rawT > 0.0 && rawT < params.collisionLookaheadS) {
                        double fx = dx + rvx * rawT;
                        double fy = dy + rvy * rawT;
                        double fd = std::hypot(fx, fy);
                        if (fd < params.collisionRangeCm) {
                            double awayX = (fd > 1e-9) ? (-fx / fd) : (-nx);
                            double awayY = (fd > 1e-9) ? (-fy / fd) : (-ny);
                            double fwd = awayX * hx[i] + awayY * hy[i];
                            if (fwd < 0.0) {
                                awayX -= fwd * hx[i];
                                awayY -= fwd * hy[i];
                            }
                            std::tie(awayX, awayY) = normalizeVec(awayX, awayY);
                            auto [clatX, clatY] = lateralAway(fx, fy, rightX, rightY);
                            auto [ax, ay] = normalizeVec(
                                params.collisionRightBias * clatX + (1.0 - params.collisionRightBias) * awayX,
                                params.collisionRightBias * clatY + (1.0 - params.collisionRightBias) * awayY
                            );
                            double sc = smoothstep((params.collisionRangeCm - fd) / params.collisionRangeCm)
                                      * smoothstep(1.0 - rawT / params.collisionLookaheadS);
                            pw.colX[i] += sc * ax;
                            pw.colY[i] += sc * ay;
                        }
                    }
                }

                // --- Przewidywanie po łukach (TTC) -> ucieczka w bok + hamowanie ---
                double cc = d * d - params.robotGapStopCm * params.robotGapStopCm;
                double tHit = (cc <= 0.0) ? 0.0 : inf;
                if (std::isinf(tHit)) {
                    for (int k = 0; k < predSteps; ++k) {
                        double dk = std::hypot(predX[k][j] - predX[k][i], predY[k][j] - predY[k][i]);
                        if (dk < params.robotGapStopCm) {
                            tHit = horizon * double(k + 1) / predSteps;
                            break;
                        }
                    }
                }

                if (!std::isinf(tHit)) {
                    double urg = smoothstep((params.escapeTtcS - tHit) / std::max(1e-6, params.escapeTtcS - params.robotTtcStopS));
                    pw.escX[i] += urg * latX;
                    pw.escY[i] += urg * latY;
                    pw.escapeUrgency[i] = std::max(pw.escapeUrgency[i], urg);
                }

                bool toward = (nx * hx[i] + ny * hy[i]) > params.robotGapTowardCos;
                if (toward && !std::isinf(tHit)) {
                    double frac = smoothstep((tHit - params.robotTtcStopS) / std::max(1e-6, params.robotTtcSlowS - params.robotTtcStopS));
                    double follow = (cc > 0.0) ? (params.robotFollowGain * std::clamp(vx[j] * hx[i] + vy[j] * hy[i], 0.0, params.speedCmS)) : 0.0;
                    double vPair = std::max(params.speedCmS * frac, follow);
                    pw.robotSpeedLimit[i] = std::min(pw.robotSpeedLimit[i], vPair);
                }
            }
        }
    }

    std::pair<double, double> computeForces(size_t i, const PairwiseCache &pw) {
        auto &v = boids[i];
        double hx = std::cos(v.theta), hy = std::sin(v.theta);

        double sepX = pw.sepX[i], sepY = pw.sepY[i];
        double aliX = pw.aliX[i], aliY = pw.aliY[i], aliW = pw.aliW[i];
        double cohX = pw.cohX[i], cohY = pw.cohY[i], cohW = pw.cohW[i];
        double colX = pw.colX[i], colY = pw.colY[i];

        // Alignment jako łagodna korekta kursu (flocking_2.py)
        if (aliW > 1e-12) {
            double avgW = params.alignSelfWeight + aliW;
            double avgX = (params.alignSelfWeight * hx + aliX) / avgW;
            double avgY = (params.alignSelfWeight * hy + aliY) / avgW;
            if (std::hypot(avgX, avgY) > 1e-12) {
                std::tie(avgX, avgY) = normalizeVec(avgX, avgY);
                aliX = avgX - hx;
                aliY = avgY - hy;
                double amag = std::hypot(aliX, aliY);
                if (amag > params.alignMaxCorrection) {
                    double sc = params.alignMaxCorrection / amag;
                    aliX *= sc;
                    aliY *= sc;
                }
            } else {
                aliX = aliY = 0.0;
            }
        } else {
            aliX = aliY = 0.0;
        }

        if (cohW > 1e-12) {
            std::tie(cohX, cohY) = normalizeVec(cohX / cohW - v.x, cohY / cohW - v.y);
        } else {
            cohX = cohY = 0.0;
        }

        // Zachowanie amplitudy separation i collision (tylko przycięcie > 1.0)
        double sepMag = std::hypot(sepX, sepY);
        if (sepMag > 1.0) { sepX /= sepMag; sepY /= sepMag; }
        double colMag = std::hypot(colX, colY);
        if (colMag > 1.0) { colX /= colMag; colY /= colMag; }

        // Ściany (smoothstep) + predykcja
        double iw = innerW(), ih = innerH();
        double left = v.x + iw, right = iw - v.x;
        double top  = v.y + ih, bottom = ih - v.y;

        double wallX = wallForce(left) - wallForce(right);
        double wallY = wallForce(top)  - wallForce(bottom);

        double futureX = v.x + hx * params.speedCmS * params.wallLookaheadS;
        double futureY = v.y + hy * params.speedCmS * params.wallLookaheadS;

        auto predStr = [&](double distWall) {
            if (distWall >= params.wallRangeCm) return 0.0;
            double span = std::max(1e-9, params.wallRangeCm + params.softWallPenetrationCm);
            return smoothstep((params.wallRangeCm - distWall) / span);
        };
        wallX += params.wallLookaheadGain * (predStr(futureX + iw) - predStr(iw - futureX));
        wallY += params.wallLookaheadGain * (predStr(futureY + ih) - predStr(ih - futureY));

        // Tłumienie szumu przy ścianie
        double minWallDist = std::min({left, right, top, bottom});
        double noiseScale = (minWallDist >= params.wallRangeCm)
                                ? 1.0
                                : (0.15 + 0.85 * smoothstep(minWallDist / std::max(1e-6, params.wallRangeCm)));

        double desiredX = params.wKeep * hx
                        + params.wSep * sepX
                        + params.wAlign * aliX
                        + params.wCoh * cohX
                        + params.wCollision * colX
                        + params.wWall * wallX
                        + noiseScale * params.noiseGain * v.noiseX;
        double desiredY = params.wKeep * hy
                        + params.wSep * sepY
                        + params.wAlign * aliY
                        + params.wCoh * cohY
                        + params.wCollision * colY
                        + params.wWall * wallY
                        + noiseScale * params.noiseGain * v.noiseY;

        if (std::hypot(desiredX, desiredY) < 1e-12) {
            desiredX = hx; desiredY = hy;
        } else {
            std::tie(desiredX, desiredY) = normalizeVec(desiredX, desiredY);
        }

        // Ucieczka w bok przed kolizją z robotem
        double uEsc = std::min(1.0, params.escapeGain * pw.escapeUrgency[i]);
        double ex = pw.escX[i], ey = pw.escY[i];
        if (uEsc > 0.0 && std::hypot(ex, ey) > 1e-9) {
            std::tie(ex, ey) = normalizeVec(ex, ey);
            double bx = (1.0 - uEsc) * desiredX + uEsc * ex;
            double by = (1.0 - uEsc) * desiredY + uEsc * ey;
            if (std::hypot(bx, by) > 1e-9) std::tie(desiredX, desiredY) = normalizeVec(bx, by);
        }

        // Ucieczka od ściany (ma pierwszeństwo przed robotem)
        auto [uW, wtx, wty] = wallEscape(v);
        if (uW > 0.0) {
            double bx = (1.0 - uW) * desiredX + uW * wtx;
            double by = (1.0 - uW) * desiredY + uW * wty;
            if (std::hypot(bx, by) < 1e-9) { desiredX = wtx; desiredY = wty; }
            else std::tie(desiredX, desiredY) = normalizeVec(bx, by);
        }

        // Pamięć kierunku omijania narożnika
        const double maxSteerRad = params.maxSteerTargetAngleDeg * kPi / 180.0;
        int cornerSign = cornerAvoidanceTurn(v);
        if (cornerSign != 0) {
            double target = v.theta + cornerSign * maxSteerRad;
            return {std::cos(target), std::sin(target)};
        }

        // Ograniczenie chwilowego celu do stożka +/- MAX_STEER_TARGET_ANGLE_DEG
        double rawTarget = std::atan2(desiredY, desiredX);
        double headingErr = wrapAngle(rawTarget - v.theta);
        if (std::abs(headingErr) > (150.0 * kPi / 180.0) && std::abs(v.omega) > 1e-3) {
            headingErr = std::copysign(std::abs(headingErr), v.omega);
        }
        headingErr = std::clamp(headingErr, -maxSteerRad, maxSteerRad);
        double target = v.theta + headingErr;
        return {std::cos(target), std::sin(target)};
    }

    void physicalWallBackstop(FlockingBoid &v) const {
        double xmax = innerW() + params.hardWallPenetrationCm;
        double ymax = innerH() + params.hardWallPenetrationCm;
        v.x = std::clamp(v.x, -xmax, xmax);
        v.y = std::clamp(v.y, -ymax, ymax);
        v.theta = wrapAngle(v.theta);
    }

    void robotBackstop() {
        if (boids.size() < 2) return;
        const double minDist = backstopCenterDistanceCm();
        for (int iter = 0; iter < 8; ++iter) {
            bool changed = false;
            for (size_t i = 0; i < boids.size(); ++i) {
                for (size_t j = i + 1; j < boids.size(); ++j) {
                    double dx = boids[j].x - boids[i].x;
                    double dy = boids[j].y - boids[i].y;
                    double d = std::hypot(dx, dy);
                    if (d >= minDist) continue;
                    changed = true;
                    double nx = (d < 1e-9) ? 1.0 : (dx / d);
                    double ny = (d < 1e-9) ? 0.0 : (dy / d);
                    double shift = 0.5 * (minDist - d);
                    boids[i].x -= shift * nx; boids[i].y -= shift * ny;
                    boids[j].x += shift * nx; boids[j].y += shift * ny;
                    physicalWallBackstop(boids[i]);
                    physicalWallBackstop(boids[j]);
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