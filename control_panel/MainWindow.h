#pragma once

#include <QMainWindow>
#include <QUdpSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QThread>
#include <QTabWidget>
#include <QTableWidget>
#include <QPlainTextEdit>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QAction>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QImage>
#include <QMap>
#include <QVector>
#include <QPointF>
#include <QDateTime>
#include <QNetworkInterface>
#include <QMutex>
#include <QRectF>
#include <map>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "FlockingSim.h"

struct DetectedMarker {
    int markerId = -1;
    double centerX = 0.0;
    double centerY = 0.0;
    double rawAngleDeg = 0.0;
    int missedFrames = 0;
    bool isHeld() const { return missedFrames > 0; }
};
Q_DECLARE_METATYPE(DetectedMarker)
Q_DECLARE_METATYPE(QVector<DetectedMarker>)

// Osobny wątek przechwytywania wideo i detekcji ArUco (zero lagów w GUI)
class CameraThread : public QThread {
    Q_OBJECT
public:
    CameraThread(int camIdx, int reqW, int reqH, QObject *parent = nullptr);
    ~CameraThread() override { stopCapture(); }

    void stopCapture();
    void markGuiReady() { guiBusy.store(false, std::memory_order_release); }
    void setDebugRejected(bool enabled) { debugRejected.store(enabled, std::memory_order_relaxed); }
    void setArenaRoi(bool enabled, const QRectF &roiNorm);

signals:
    void cameraOpened(const QString &infoText, int realW, int realH);
    void cameraError(const QString &errMsg);
    void frameProcessed(const QImage &previewRgb, int origW, int origH,const QVector<DetectedMarker> &markers, double fps);

protected:
    void run() override;

private:
    int cameraIndex;
    int requestedWidth;
    int requestedHeight;
    std::atomic<bool> running{false};
    std::atomic<bool> guiBusy{false};
    std::atomic<bool> debugRejected{false};
    QMutex roiMutex;
    bool useArenaRoi = false;
    QRectF arenaRoiNorm;
};

class ClickableVideoLabel : public QLabel {
    Q_OBJECT
public:
    explicit ClickableVideoLabel(const QString &text, QWidget *parent = nullptr)
        : QLabel(text, parent) {
        setCursor(Qt::CrossCursor);
    }
    void setSourceSize(int w, int h) {
        sourceWidth = w;
        sourceHeight = h;
    }

signals:
    void imageClicked(double x, double y);
    void imageDragged(double x, double y);
    void wheelScrolled(int delta);

protected:
    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            QPointF pt;
            if (mapToSource(event->position(), pt))
                emit imageClicked(pt.x(), pt.y());
        } else {
            QLabel::mousePressEvent(event);
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        if (event->buttons() & Qt::LeftButton) {
            QPointF pt;
            if (mapToSource(event->position(), pt))
                emit imageDragged(pt.x(), pt.y());
        } else {
            QLabel::mouseMoveEvent(event);
        }
    }

    void wheelEvent(QWheelEvent *event) override {
        if (event->angleDelta().y() != 0) {
            emit wheelScrolled(event->angleDelta().y());
            event->accept();
        } else {
            QLabel::wheelEvent(event);
        }
    }

private:
    bool mapToSource(const QPointF &pos, QPointF &out) const {
        QPixmap p = pixmap();
        if (p.isNull() || sourceWidth <= 0 || sourceHeight <= 0) return false;
        double left = (width() - p.width()) / 2.0;
        double top  = (height() - p.height()) / 2.0;
        double x = pos.x() - left;
        double y = pos.y() - top;
        if (x < 0 || y < 0 || x >= p.width() || y >= p.height()) return false;
        out = QPointF(x * sourceWidth / p.width(), y * sourceHeight / p.height());
        return true;
    }

    int sourceWidth = 0;
    int sourceHeight = 0;
};

struct RobotInfo {
    QString uid;
    QString name;
    QString ip;
    QString version;
    QString mode = "IDLE";
    double freespeed = 35.0;
    double acc = 0.5;
    double brake = 0.8;
    double dist = 0.0;
    double spd = 0.0;
    qint64 lastSeenMs = 0;
    bool selected = true;
};

struct VisionTrack {
    int markerId = -1;
    double centerX = 0.0;
    double centerY = 0.0;
    double angleDeg = 0.0;
    int missedFrames = 0;
    qint64 lastSeenMs = 0;

    // Pamięć regulatora PD + wygładzania
    double smoothedSpeed = 0.0;
    double smoothedTurn = 0.0;
    double prevError = 0.0;
    double filteredErrorRate = 0.0;
    qint64 lastControlMs = 0;
    bool hasPrevError = false;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    // Sieć
    void onUdpReadyRead();
    void onNewHttpConnection();
    void onDhcpReadyRead();
    void toggleDhcpServer(bool enabled);
    void refreshTableStatus();

    // Zakładka 1: IDM, PID i OTA
    void selectAllRobots();
    void deselectAllRobots();
    void selectOnlineRobots();
    void onTableItemChanged(QTableWidgetItem *item);
    void sendStartCommand();
    void sendStopCommand();
    void sendCalibrateCommand();
    void sendSpinCommand();
    void sendParametersCommand();
    void sendPidCommand();
    void triggerPhantomJam();
    void browseOtaFile();
    void sendOtaCommand();

    // Zakładka 2: Kamera ArUco (z wątku), Koło i Ławica ryb
    void refreshCameras();
    void toggleCamera();
    void onCameraOpened(const QString &infoText, int realW, int realH);
    void onCameraError(const QString &errMsg);
    void onCameraFrameReady(const QImage &previewRgb, int origW, int origH, const QVector<DetectedMarker> &markers, double fps);
    void onVideoImageClicked(double x, double y);
    void onVideoImageDragged(double x, double y);
    void onVideoWheelScrolled(int delta);
    void onVisionTableSelectionChanged();
    void startVisionFigureEight();
    void startCircleSetup();
    void startVisionCircleAll();
    void startArenaCalibration();
    void startFlockingPreview();
    void startFlockingRobots();
    void updateFlockingParamsFromUi();
    void stopVisionAll();

private:
    void setupUi();
    QWidget* createIdmTab();
    QWidget* createVisionTab();
    void setupMenuBar();
    void loadRobotDatabase();
    void saveRobotDatabase();
    void loadArenaConfig();
    void saveArenaConfig();
    void ensureRobotRowExists(const QString &uid);
    void rebuildVisionTable();
    void logMessage(const QString &msg);

    void sendJsonToTargets(const QJsonObject &payload, bool forceAll = false);
    void sendRawUdp(const QByteArray &data, const QString &ip);
    void sendVisionDriveCmd(int markerId, double speed, double turn);
    QList<QString> getTargetUids(bool forceAll = false) const;
    QString allocateIpForMac(const QString &mac);
    void updateNetworkCache();
    void checkNetworkSetup();

    QString findRobotUidByMarkerId(int markerId) const;
    static int extractNumberFromName(const QString &name);

    void stepVisionControl(int frameW, int frameH);
    bool driveMarkerToPoint(VisionTrack &tr, QPointF target, double speedCap, double pxScale, bool continuousMode = false);
    void updateCameraRoi();
    void resetTrackMemory(VisionTrack &tr);
    static double normalizeAngleDeg(double a);

    // Stałe portów i adresacji
    static constexpr quint16 UDP_COMMAND_PORT = 5005;
    static constexpr quint16 UDP_LISTEN_PORT  = 5006;
    static constexpr quint16 HTTP_OTA_PORT    = 8000;
    static constexpr quint16 DHCP_SERVER_PORT = 67;
    static constexpr quint16 DHCP_CLIENT_PORT = 68;
    const QString SUBNET_PREFIX  = "192.168.1.";
    const QString SERVER_ETH_IP  = SUBNET_PREFIX + "1";
    const QString DB_FILENAME    = "znane_roboty.json";
    const QString ARENA_FILENAME = "arena_config.json";

    // Gniazda sieciowe i timery
    QUdpSocket *udpListenSocket = nullptr;
    QUdpSocket *udpCommandSocket = nullptr;
    QUdpSocket *udpDhcpSocket = nullptr;
    QTcpServer *httpOtaServer = nullptr;
    QTimer *statusTimer = nullptr;

    // Cache interfejsów sieciowych (odświeżany co 3 s zamiast przy każdym pakiecie DHCP!)
    bool cachedServerValid = false;
    int cachedServerIfaceIdx = -1;
    QHostAddress cachedServerBroadcast;
    QStringList cachedAllBroadcasts;
    int netCacheTickCounter = 0;

    // Dane floty oraz pula adresów serwera DHCP
    QMap<QString, RobotInfo> fleet;
    QMap<QString, QString> uidToName;
    QMap<QString, QString> macToIp;
    int nextDhcpOctet = 21;
    bool updatingTableProgrammatically = false;
    int otaDownloadsCount = 0;
    quint32 commandSeq = 0;

    // Menu DHCP
    QAction *actDhcpEnable = nullptr;

    // Główne zakładki
    QTabWidget *mainTabs = nullptr;

    // --- Zakładka 1 (IDM, PID & OTA) ---
    QTableWidget *fleetTable = nullptr;
    QLabel *fleetSummaryLabel = nullptr;

    QCheckBox *chkFreespeed = nullptr;
    QDoubleSpinBox *spinFreespeed = nullptr;
    QCheckBox *chkAcc = nullptr;
    QDoubleSpinBox *spinAcc = nullptr;
    QCheckBox *chkBrake = nullptr;
    QDoubleSpinBox *spinBrake = nullptr;

    QCheckBox *chkKp = nullptr;
    QDoubleSpinBox *spinKp = nullptr;
    QCheckBox *chkKi = nullptr;
    QDoubleSpinBox *spinKi = nullptr;
    QCheckBox *chkKd = nullptr;
    QDoubleSpinBox *spinKd = nullptr;

    QDoubleSpinBox *spinJamSpeed = nullptr;
    QDoubleSpinBox *spinJamDuration = nullptr;

    QLineEdit *editOtaFilePath = nullptr;
    QCheckBox *chkAutoVersionTag = nullptr;
    QLineEdit *editCustomVersionTag = nullptr;
    QLabel *lblOtaStats = nullptr;
    QPlainTextEdit *logConsole = nullptr;

    // --- Zakładka 2 (Kamera ArUco, Koło & Ławica ryb) ---
    QComboBox *comboCamera = nullptr;
    QComboBox *comboResolution = nullptr;
    QPushButton *btnRefreshCam = nullptr;
    QPushButton *btnToggleCamera = nullptr;
    QCheckBox *chkDebugAruco = nullptr;
    QLabel *lblCameraFps = nullptr;
    QSpinBox *spinHeadingOffset = nullptr;
    QComboBox *comboTurnSign = nullptr;
    QSpinBox *spinCircleRadius = nullptr;
    QComboBox *comboCircleDir = nullptr;
    QPushButton *btnArenaCalib = nullptr;
    ClickableVideoLabel *videoLabel = nullptr;
    QTableWidget *visionTable = nullptr;
    QLabel *lblVisionStatus = nullptr;

    // Parametry ławicy ryb (flocking_2.py) i areny
    QDoubleSpinBox *spinArenaW = nullptr;
    QDoubleSpinBox *spinArenaH = nullptr;
    QDoubleSpinBox *spinFlockSpeed = nullptr;
    QDoubleSpinBox *spinFlockSep = nullptr;
    QDoubleSpinBox *spinFlockAlign = nullptr;
    QDoubleSpinBox *spinFlockCoh = nullptr;
    QDoubleSpinBox *spinFlockTurnR = nullptr;
    QSpinBox *spinPreviewCount = nullptr;

    CameraThread *cameraThread = nullptr;
    QMap<int, VisionTrack> visionTracks;
    QList<int> lastVisionTableIds;
    qint64 lastVisionTableUpdateMs = 0;
    int lastFrameW = 1920;
    int lastFrameH = 1080;

    enum class VisionMode {
        None,
        Point,
        FigureEight,
        CircleSetup,
        Circle,
        FlockingPreview,
        FlockingRobots
    };
    VisionMode visionMode = VisionMode::None;
    int selectedMarkerId = -1;
    QPointF visionPointTarget;
    QVector<QPointF> figure8Path;
    int figure8Index = 0;

    QPointF circleCenter;
    bool circleCenterInitialized = false;
    double circleRadius = 180.0;

    FlockingSim flockingSim;
    bool arenaCalibrating = false;
    QVector<QPointF> arenaClickPoints;
    qint64 lastFlockingMs = 0;
    QMap<int, QPointF> flockingRobotTargetsPx;

    QCheckBox *chkUseArenaRoi = nullptr;
    QDoubleSpinBox *spinMaxLeadCm = nullptr;
};