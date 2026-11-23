#pragma once

#include <QMainWindow>
#include <QUdpSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
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
#include <QMap>
#include <QVector>
#include <QPointF>
#include <QDateTime>
#include <QNetworkInterface>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "FlockingSim.h"

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
    qint64 lastSeenMs = 0;

    double smoothedSpeed = 0.0;
    double smoothedTurn = 0.0;
    double prevError = 0.0;
    double filteredErrorRate = 0.0;
    qint64 lastControlMs = 0;
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

    // Zakładka 1: Zaznaczanie, sterowanie IDM, PID i OTA
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

    // Zakładka 2: Kamera ArUco, interaktywne koło i ławica ryb (Boids)
    void refreshCameras();
    void toggleCamera();
    void processCameraFrame();
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
    void updateTableRow(const QString &uid);
    void rebuildVisionTable();
    void logMessage(const QString &msg);

    void sendJsonToTargets(const QJsonObject &payload, bool forceAll = false);
    void sendRawUdp(const QByteArray &data, const QString &ip);
    void sendVisionDriveCmd(int markerId, double speed, double turn);
    QList<QString> getTargetUids(bool forceAll = false) const;
    QString allocateIpForMac(const QString &mac);
    bool findServerInterface(QNetworkAddressEntry *entry = nullptr, int *ifaceIndex = nullptr) const;
    void checkNetworkSetup();

    QString findRobotUidByMarkerId(int markerId) const;
    static int extractNumberFromName(const QString &name);

    void stepVisionControl(int frameW, int frameH);
    bool driveMarkerToPoint(VisionTrack &tr, QPointF target, double speedCap);
    static double normalizeAngleDeg(double a);

    // Stałe portów i adresacji
    static constexpr quint16 UDP_COMMAND_PORT = 5005;
    static constexpr quint16 UDP_LISTEN_PORT  = 5006;
    static constexpr quint16 HTTP_OTA_PORT    = 8000;
    static constexpr quint16 DHCP_SERVER_PORT = 67;
    static constexpr quint16 DHCP_CLIENT_PORT = 68;
    const QString SUBNET_PREFIX = "192.168.1.";
    const QString SERVER_ETH_IP = SUBNET_PREFIX + "1";
    const QString DB_FILENAME   = "znane_roboty.json";
    const QString ARENA_FILENAME = "arena_config.json";

    // Gniazda sieciowe i timery
    QUdpSocket *udpListenSocket = nullptr;
    QUdpSocket *udpCommandSocket = nullptr;
    QUdpSocket *udpDhcpSocket = nullptr;
    QTcpServer *httpOtaServer = nullptr;
    QTimer *statusTimer = nullptr;

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
    QPushButton *btnToggleCamera = nullptr;
    QSpinBox *spinHeadingOffset = nullptr;
    QComboBox *comboTurnSign = nullptr;
    QSpinBox *spinCircleRadius = nullptr;
    QComboBox *comboCircleDir = nullptr;
    QPushButton *btnArenaCalib = nullptr;
    ClickableVideoLabel *videoLabel = nullptr;
    QTableWidget *visionTable = nullptr;
    QLabel *lblVisionStatus = nullptr;

    // Parametry ławicy ryb (Boids) i areny
    QDoubleSpinBox *spinArenaW = nullptr;
    QDoubleSpinBox *spinArenaH = nullptr;
    QDoubleSpinBox *spinFlockSpeed = nullptr;
    QDoubleSpinBox *spinFlockSep = nullptr;
    QDoubleSpinBox *spinFlockAlign = nullptr;
    QDoubleSpinBox *spinFlockCoh = nullptr;
    QDoubleSpinBox *spinFlockTurnR = nullptr;
    QSpinBox *spinPreviewCount = nullptr;

    cv::VideoCapture cameraCap;
    QTimer *cameraTimer = nullptr;
    cv::aruco::ArucoDetector arucoDetector;
    QMap<int, VisionTrack> visionTracks;
    QList<int> lastVisionTableIds;
    int lastFrameW = 1280;
    int lastFrameH = 720;

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

    // Kalibracja 4 rogów i instancja symulacji ławicy
    FlockingSim flockingSim;
    bool arenaCalibrating = false;
    QVector<QPointF> arenaClickPoints;
    qint64 lastFlockingMs = 0;
    QMap<int, QPointF> flockingRobotTargetsPx;
};